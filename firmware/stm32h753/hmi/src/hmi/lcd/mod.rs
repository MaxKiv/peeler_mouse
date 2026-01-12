pub mod setup;

use defmt::*;
use display_interface_i2c::I2CInterface;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::{
    Drawable,
    image::{Image, ImageRawLE},
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    text::{Baseline, Text},
};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::Point};
use heapless::format;
use oled_async::{displays::ssd1309::Ssd1309_128_64, mode::GraphicsMode};

use crate::{
    hmi::lcd::setup::SSD1309_FRAMEBUFFER_SIZE,
    hmi::{button::BUTTON_WATCH_SIZE, encoder::ENCODER_STATE},
    motor::{knife::KNIFE_SETPOINT, linear::LINEAR_SETPOINT, rotation::ROTATION_SETPOINT},
};

pub static LCD_INPUT: Watch<CriticalSectionRawMutex, bool, { BUTTON_WATCH_SIZE }> = Watch::new();

const LCD_PERIOD: Duration = Duration::from_millis(100);
const TEXT_OFFSET_HEIGHT: i32 = 14;

#[embassy_executor::task]
pub async fn manage_display(
    mut display: GraphicsMode<
        Ssd1309_128_64,
        I2CInterface<I2c<'static, Async, Master>>,
        { SSD1309_FRAMEBUFFER_SIZE },
    >,
) {
    info!("Starting to manage display");

    let mut ticker = Ticker::every(LCD_PERIOD);

    let mut knife_rx = KNIFE_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");
    let mut linear_rx = LINEAR_SETPOINT
        .receiver()
        .expect("increase LINEAR_SETPOINT N");
    let mut rotation_rx = ROTATION_SETPOINT
        .receiver()
        .expect("increase ROTATION_SETPOINT N");
    let mut encoder_rx = ENCODER_STATE.receiver().expect("increase ENCODER_STATE N");

    // Initialise display
    while display.init().await.is_err() {
        error!("Unable to initialise display, is it connected?");
        Timer::after(Duration::from_millis(1000)).await;
    }
    display.clear();
    display.flush().await.unwrap();

    // Load image data
    let joris_im: ImageRawLE<BinaryColor> = ImageRawLE::new(
        include_bytes!("/home/max/git/saxion/peeler_mouse/data/joris.raw"),
        128,
    );
    let joris = Image::new(&joris_im, Point::new(0, 0));

    // let rene_im: ImageRawLE<BinaryColor> = ImageRawLE::new(
    //     include_bytes!("/home/max/git/saxion/peeler_mouse/data/rene.raw"),
    //     128,
    // );
    // let rene = Image::new(&rene_im, Point::new(0, 0));
    //
    // let lex_im: ImageRawLE<BinaryColor> = ImageRawLE::new(
    //     include_bytes!("/home/max/git/saxion/peeler_mouse/data/lex.raw"),
    //     128,
    // )
    // let lex = Image::new(&lex_im, Point::new(0, 0));

    joris.draw(&mut display).unwrap();
    if display.flush().await.is_err() {
        error!("Unable to flush display");
    }

    let selected_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();

    let unselected_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Give people time to appreciate the beautiful splash screen
    Timer::after_millis(350).await;

    // Main Display loop
    loop {
        display.clear();

        // get latest setpoints
        let encoder_data = encoder_rx.try_get().unwrap_or_default();
        let knife_cmd = knife_rx.try_get().unwrap_or_default();
        let linear_cmd = linear_rx.try_get().unwrap_or_default();
        let rotation_cmd = rotation_rx.try_get().unwrap_or_default();

        let selected = encoder_data.pos.abs();

        let modu = selected % 5;
        let selection = modu as usize;

        // Format data
        let knife_str = format!(128; "Cut - {}", knife_cmd)
            .expect("knife cmd string doesn't fit heapless string");
        let linear_str = format!(128; "Lin - {}", linear_cmd)
            .expect("linear cmd string doesn't fit heapless string");
        let rotation_str = format!(128; "Rot - {}", rotation_cmd)
            .expect("rotation cmd string doesn't fit heapless string");
        let encoder_str = format!(128; "Encoder - {}", encoder_data)
            .expect("rotation cmd string doesn't fit heapless string");
        let selected_str = format!(128; "Sel # {} - {}", selection, modu)
            .expect("rotation cmd string doesn't fit heapless string");

        // Draw to display
        let to_plot = [
            &knife_str,
            &linear_str,
            &rotation_str,
            &encoder_str,
            &selected_str,
        ];

        // debug!("LCD selected # {}", selection);

        for (idx, data) in to_plot.iter().enumerate() {
            Text::with_baseline(
                data,
                Point::new(10, TEXT_OFFSET_HEIGHT * idx as i32),
                if idx == selection {
                    selected_style
                } else {
                    unselected_style
                },
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();
        }

        // Flush display
        if display.flush().await.is_err() {
            warn!("Unable to flush display");
            while display.init().await.is_err() {
                error!("Unable to initialise display, is it connected?");
                Timer::after(Duration::from_millis(1000)).await;
            }
        }

        ticker.next().await;
    }
}
