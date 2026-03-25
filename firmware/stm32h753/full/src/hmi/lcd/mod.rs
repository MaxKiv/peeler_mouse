pub mod setup;
pub mod startup;

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
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    text::{Baseline, Text},
};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::Point};
use heapless::format;
use messenger_mouse::{
    Report,
    motor::{KnifeManager, MotorCommand, MotorDirection},
};
use oled_async::{displays::ssd1309::Ssd1309_128_64, mode::GraphicsMode};
use uom::si::velocity::millimeter_per_second;

use crate::{
    comms::task::REPORT_WATCH,
    hmi::{
        button::BUTTON_WATCH_SIZE,
        lcd::{setup::SSD1309_FRAMEBUFFER_SIZE, startup::startup_display},
    },
    supervisor::{HmiState, SelectedMotor, appstate::Appstate, task::APPSTATE_WATCH},
};

pub static LCD_INPUT: Watch<CriticalSectionRawMutex, bool, { BUTTON_WATCH_SIZE }> = Watch::new();

const LCD_PERIOD: Duration = Duration::from_millis(100);

// UI related
const TEXT_OFFSET_WIDTH: i32 = 4;
const TEXT_OFFSET_HEIGHT: i32 = 14;
const NUM_SPEED_BARS: usize = 8;

pub enum MotorMovementDirection {
    UpDown,
    LeftRight,
}

struct FontStyles<'a> {
    width: usize,
    selected: embedded_graphics::mono_font::MonoTextStyle<'a, BinaryColor>,
    unselected: embedded_graphics::mono_font::MonoTextStyle<'a, BinaryColor>,
}

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
    let mut appstate_rx = APPSTATE_WATCH
        .receiver()
        .expect("increase APPSTATE_WATCH N");
    let mut report_rx = REPORT_WATCH.receiver().expect("increase REPORT_WATCH N");

    // Set up fonts
    let font = &FONT_6X10;
    let selected = MonoTextStyleBuilder::new()
        .font(&font)
        .text_color(BinaryColor::Off)
        .background_color(BinaryColor::On)
        .build();
    let unselected = MonoTextStyleBuilder::new()
        .font(&font)
        .text_color(BinaryColor::On)
        .build();
    let font_styles = FontStyles {
        width: 6,
        selected,
        unselected,
    };

    // Start display
    startup_display(&mut display).await;

    // Main Display loop
    loop {
        display.clear();

        // Get latest state & report
        let state = appstate_rx.try_get().unwrap_or_default();
        let report = report_rx.try_get().unwrap_or_default();

        // use appstate to draw display
        draw_ui(&mut display, state, report, &font_styles);

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

fn draw_ui(
    mut display: &mut GraphicsMode<
        Ssd1309_128_64,
        I2CInterface<I2c<'static, Async, Master>>,
        { SSD1309_FRAMEBUFFER_SIZE },
    >,
    state: Appstate,
    report: Report,
    font_styles: &FontStyles,
) {
    draw_header(&state, &font_styles, &mut display);

    let rot_str = format!(128; "Rot | {}", get_cmd_str(&state.rotation_setpoint,
        MotorMovementDirection::LeftRight))
    .expect("rot cmd string doesn't fit heapless string");

    let lin_str = format!(128; "Lin | {}", get_cmd_str(&state.translation_setpoint,
        MotorMovementDirection::LeftRight))
    .expect("lin cmd string doesn't fit heapless string");

    let cut_str = match state.knife_manager {
        KnifeManager::Manual => {
            format!(128; "Cut | {}", get_cmd_str(&state.knife_setpoint, MotorMovementDirection::UpDown))
                .expect("cut cmd string doesn't fit heapless string")
        }
        KnifeManager::Vision => {
            format!(128; "Cut | ESP: {}",
                report.measurements.current_knife_state.encoder_state.absolute_count())
            .expect("cut cmd string doesn't fit heapless string")
        }
    };

    let state_str = format!(128; "{}", get_state_str(&state))
        .expect("running state string doesn't fit heapless string");

    let to_plot = [&cut_str, &lin_str, &rot_str, &state_str];
    let selected: usize = match state.selected_motor {
        SelectedMotor::Cut => 0,
        SelectedMotor::Translation => 1,
        SelectedMotor::Rotation => 2,
    };

    for (idx, data) in to_plot.iter().enumerate() {
        Text::with_baseline(
            data,
            Point::new(TEXT_OFFSET_WIDTH, TEXT_OFFSET_HEIGHT * (idx + 1) as i32),
            if idx == selected {
                font_styles.selected
            } else {
                font_styles.unselected
            },
            Baseline::Top,
        )
        .draw(display)
        .unwrap();
    }
}

fn get_cmd_str(
    setpoint: &MotorCommand,
    motor_movement_direction: MotorMovementDirection,
) -> &'static str {
    match setpoint {
        MotorCommand::Halt => format!(128, "HALT  "),
        MotorCommand::Home => format!(128, "HOMING"),
        MotorCommand::MoveVelocity(sp) => format!(128;
            "{}   {}mm/s",
            get_movement_dir_str(motor_movement_direction, sp.dir),
            sp.speed.get::<millimeter_per_second>(),
        )
        .expect("string doesn't fit heapless string"),
        MotorCommand::MovePosition(sp) => format!(128;
            "{}mm-{}mm/s",
            get_movement_dir_str(motor_movement_direction, sp.dir),
            sp.speed.get::<millimeter_per_second>(),
        )
        .expect("string doesn't fit heapless string"),
    }
}

// Kan slechter
fn get_movement_dir_str(
    motor_movement_direction: MotorMovementDirection,
    direction: MotorDirection,
) -> &'static str {
    use MotorDirection::*;

    match motor_movement_direction {
        MotorMovementDirection::UpDown => match direction {
            Forward => ">",
            Reverse => "<",
        },
        MotorMovementDirection::LeftRight => match direction {
            Forward => "^",
            Reverse => "v",
        },
    }
}

fn get_state_str(appstate: &Appstate) -> heapless::String<128> {
    let out = format!(128; "{} - {}",
    match appstate.enable {
            true => "ENABLED ",
            false => "DISABLED",
        }, match appstate.knife_manager {
        KnifeManager::Manual => "Manual",
        KnifeManager::Vision => "Vision",
    })
    .expect("state cmd string doesn't fit heapless string");

    out
}

fn draw_header(
    state: &Appstate,
    font_styles: &FontStyles,
    display: &mut GraphicsMode<
        Ssd1309_128_64,
        I2CInterface<I2c<'static, Async, Master>>,
        { SSD1309_FRAMEBUFFER_SIZE },
    >,
) {
    Text::with_baseline(
        " 1 ",
        Point::new(TEXT_OFFSET_WIDTH, 0),
        if state.hmi_state == HmiState::NoSelection {
            font_styles.selected
        } else {
            font_styles.unselected
        },
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
    Text::with_baseline(
        "|",
        Point::new(TEXT_OFFSET_WIDTH + 3 * 6, 0),
        font_styles.unselected,
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
    Text::with_baseline(
        " 2 ",
        Point::new(TEXT_OFFSET_WIDTH + 4 * 6, 0),
        if state.hmi_state == HmiState::MotorSelected {
            font_styles.selected
        } else {
            font_styles.unselected
        },
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
}
