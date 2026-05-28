pub mod setup;
pub mod startup;

use core::fmt::Write;
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
use heapless::{String, format};
use messenger_mouse::{
    ControlOutput, Esp32Report,
    motor::{ControlMode, MotorAction, MotorDirection, MotorSetpoints},
};
use oled_async::{displays::ssd1309::Ssd1309_128_64, mode::GraphicsMode};
use uom::si::length::millimeter;
use uom::si::velocity::millimeter_per_second;

use crate::{
    comms::task::REPORT_WATCH,
    hmi::{
        button::BUTTON_WATCH_SIZE,
        lcd::{setup::SSD1309_FRAMEBUFFER_SIZE, startup::startup_display},
    },
    supervisor::{HmiState, MotorSelectionTab, MotorTypes, task::HMI_STATE_WATCH},
};

pub static LCD_INPUT: Watch<CriticalSectionRawMutex, bool, { BUTTON_WATCH_SIZE }> = Watch::new();

const LCD_PERIOD: Duration = Duration::from_millis(30);

// UI related
const TEXT_OFFSET_WIDTH: i32 = 4;
const TEXT_OFFSET_HEIGHT: i32 = 14;

pub enum MotorMovementDirection {
    Rotational,
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
    let mut hmi_state_rx = HMI_STATE_WATCH
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
        let state = hmi_state_rx.try_get().unwrap_or_default();
        let report = match report_rx.try_get() {
            Some(report) => report,
            _ => {
                warn!("Didn't get a report from ESP32 -> using default");
                Esp32Report::default()
            }
        };

        // use appstate to draw display
        draw_ui(&mut display, state, report, &font_styles);

        // Flush display
        if let Err(err) = display.flush().await {
            warn!("Unable to flush display: {:?}", err);
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
    state: HmiState,
    report: Esp32Report,
    font_styles: &FontStyles,
) {
    draw_header(&state, &font_styles, &mut display, &report);

    let motor_setpoints = match state.control_mode {
        ControlMode::Manual => state.motor_setpoints.clone(),
        ControlMode::Vision => {
            if let ControlOutput::Vision(effort) = report.control_output {
                effort.motor_setpoints.clone()
            } else {
                MotorSetpoints::reset()
            }
        }
    };

    let rot_str: String<128> = format!(
        128;
        "Rot | {}",
        get_cmd_str::<64>(&motor_setpoints.rotation, MotorMovementDirection::Rotational)
    )
    .unwrap();

    let lin_str: String<128> = format!(
        128;
        "Lin | {}",
        get_cmd_str::<64>(&motor_setpoints.translation, MotorMovementDirection::LeftRight)
    )
    .unwrap();

    let cut_str: String<128> = format!(
        128;
        "Cut | {}",
        get_cmd_str::<64>(&motor_setpoints.knife, MotorMovementDirection::UpDown)
    )
    .unwrap();

    // let mut cut_str: String<128> = String::new();
    // cut_str.write_str("Cut | ").unwrap();
    //
    // let cut_str = match state.control_mode {
    //     ControlMode::Manual => {
    //         format!(128; "Cut | {}", get_cmd_str::<64>(&state.motor_setpoints.knife, MotorMovementDirection::UpDown))
    //             .expect("cut cmd string doesn't fit heapless string")
    //     }
    //     ControlMode::Vision => {
    //         match report.measurements.vision_data {
    //             Some(data) => {
    //                 format!(128; "Cut | {:?} {:>2.1}Hz", data.vision_output, data.camera_fps)
    //                     .expect("cut cmd string doesn't fit heapless string")
    //             },
    //             _ => {
    //                 format!(128; "Cut | ESP: None").expect("cut cmd string doesn't fit heapless string")
    //             },
    //
    //         }
    //
    //     }
    // };

    let state_str = format!(128; "{}", get_state_str(&state))
        .expect("running state string doesn't fit heapless string");

    let to_plot = [&cut_str, &lin_str, &rot_str, &state_str];
    let selected: usize = match state.selected_motor {
        MotorTypes::Cut => 0,
        MotorTypes::Translation => 1,
        MotorTypes::Rotation => 2,
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

fn get_cmd_str<const N: usize>(
    setpoint: &MotorAction,
    motor_movement_direction: MotorMovementDirection,
) -> String<N> {
    match setpoint {
        MotorAction::Hold => format!(N; "HOLD").unwrap(),
        MotorAction::Coast => format!(N; "COAST").unwrap(),
        MotorAction::Home => format!(N; "HOMING").unwrap(),
        MotorAction::MoveVelocity(sp) => format!(
            N;
            "{} {:>4.2}mm/s",
            get_movement_dir_str(motor_movement_direction, sp.dir.clone()),
            sp.speed.get::<millimeter_per_second>(),
        )
        .unwrap(),
        MotorAction::MovePosition(sp) => format!(
            N;

            "{:>2.1}mm {:>4.2}mm/s",
            sp.target.get::<millimeter>(),
            sp.speed.get::<millimeter_per_second>(),
        )
        .unwrap(),
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
            Forward => "^",
            Reverse => "v",
        },
        MotorMovementDirection::LeftRight => match direction {
            Forward => "<",
            Reverse => ">",
        },
        MotorMovementDirection::Rotational => match direction {
            Forward => "L",
            Reverse => "R",
        },
    }
}

fn get_state_str(hmi_state: &HmiState) -> heapless::String<128> {
    let out = format!(128; "{} - {}",
    match hmi_state.enable {
            true => "ENABLED ",
            false => "DISABLED",
        }, match hmi_state.control_mode {
        ControlMode::Manual => "Manual",
        ControlMode::Vision => "Vision",
    })
    .expect("state cmd string doesn't fit heapless string");

    out
}

fn get_vision_header(hmi_state: &HmiState, report: &Esp32Report) -> heapless::String<128> {
    let out = match hmi_state.control_mode {
        ControlMode::Manual => {
            format!(128; "").expect("state cmd string doesn't fit heapless string")
        }
        ControlMode::Vision => {
            if let Some(vision) = &report.measurements.vision_data {
                format!(128; "{} {:>2.1}hz", vision.vision_output , vision.camera_fps)
                    .expect("state cmd string doesn't fit heapless string")
            } else {
                format!(128; "ESP: None").expect("state cmd string doesn't fit heapless string")
            }
        }
    };

    out
}

fn draw_header(
    state: &HmiState,
    font_styles: &FontStyles,
    display: &mut GraphicsMode<
        Ssd1309_128_64,
        I2CInterface<I2c<'static, Async, Master>>,
        { SSD1309_FRAMEBUFFER_SIZE },
    >,
    report: &Esp32Report,
) {
    Text::with_baseline(
        " 1  ",
        Point::new(TEXT_OFFSET_WIDTH, 0),
        if state.motor_selection_tab_state == MotorSelectionTab::NoSelection {
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
        Point::new(TEXT_OFFSET_WIDTH + 4 * 6, 0),
        font_styles.unselected,
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
    Text::with_baseline(
        " 2  ",
        Point::new(TEXT_OFFSET_WIDTH + 5 * 6, 0),
        if state.motor_selection_tab_state == MotorSelectionTab::MotorSelected {
            font_styles.selected
        } else {
            font_styles.unselected
        },
        Baseline::Top,
    )
    .draw(display)
    .unwrap();

    let vision_str = format!(128; "{}", get_vision_header(state, report))
        .expect("vision header string doesn't fit heapless string");
    Text::with_baseline(
        &vision_str,
        Point::new(TEXT_OFFSET_WIDTH + 9 * 6, 0),
        font_styles.unselected,
        Baseline::Top,
    )
    .draw(display)
    .unwrap();
}
