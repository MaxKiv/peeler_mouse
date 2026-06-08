pub mod setup;
pub mod startup;

extern crate alloc;
use alloc::vec::Vec;

use core::fmt::Write;
use defmt::*;
use display_interface_i2c::I2CInterface;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{self, Receiver, Watch},
};
use embassy_time::{Duration, Ticker, Timer};
use embedded_graphics::{
    Drawable,
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    text::{Baseline, Text},
};
use embedded_graphics::{pixelcolor::BinaryColor, prelude::Point};
use heapless::{String, format};
use messenger_mouse::{
    ControlOutput, Esp32Report, VisionData,
    motor::{ControlMode, MotorAction, MotorDirection, MotorSetpoints},
};
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use oled_async::{displays::ssd1309::Ssd1309_128_64, mode::GraphicsMode};
use ratatui::{
    Terminal,
    buffer::Buffer,
    layout::{Alignment, Constraint, Direction, Layout, Rect, Spacing},
    style::{Color, Modifier, Style},
    symbols::{Marker, merge::MergeStrategy},
    widgets::{
        Axis, Block, BorderType, Borders, Chart, Dataset, GraphType, Paragraph, Row, Table,
        TableState,
    },
};
use uom::si::length::millimeter;
use uom::si::velocity::millimeter_per_second;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as Cs;

use crate::{
    hmi::{
        button::BUTTON_WATCH_SIZE,
        lcd::{setup::SSD1309_FRAMEBUFFER_SIZE, startup::startup_display},
    },
    supervisor::{
        HmiState, MotorSelectionTab, MotorTypes,
        appstate::{APP_STATE_WATCH, AppState},
    },
};

pub static LCD_INPUT: Watch<CriticalSectionRawMutex, bool, { BUTTON_WATCH_SIZE }> = Watch::new();

const LCD_PERIOD: Duration = Duration::from_millis(100);

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

pub struct UIState {
    pub motors: TableState,
    pub options: TableState,
}

#[embassy_executor::task]
pub async fn manage_display(
    mut display: GraphicsMode<
        Ssd1309_128_64,
        I2CInterface<I2c<'static, Async, Master>>,
        { SSD1309_FRAMEBUFFER_SIZE },
    >,
    mut appstate_rx: watch::Receiver<'static, Cs, AppState, 3>,
    mut report_rx: watch::Receiver<'static, Cs, Esp32Report, 2>,
) {
    info!("Starting manage_display task");

    // Start display
    startup_display(&mut display).await;

    info!("Display started");

    // Setup Ratatui terminal backend
    let mut backend_cfg = EmbeddedBackendConfig::default();
    backend_cfg.font_regular = embedded_graphics_unicodefonts::mono_5x8_atlas();
    backend_cfg.font_bold = Some(embedded_graphics_unicodefonts::mono_5x8_atlas());
    backend_cfg.font_italic = Some(embedded_graphics_unicodefonts::mono_5x8_atlas());
    let backend = EmbeddedBackend::new(&mut display, backend_cfg);
    let mut terminal = Terminal::new(backend).unwrap();

    // Set up ui state
    let mut ui_state = UIState {
        motors: TableState::new().with_selected(Some(1)),
        options: TableState::new().with_selected(None),
    };
    ui_state.motors.select_first();
    ui_state.motors.select_first_column();

    info!("Starting Display render loop");

    let mut ticker = Ticker::every(LCD_PERIOD);

    // UI render loop
    loop {
        // Get latest HmiState & Esp32Report
        let state = appstate_rx.try_get().unwrap_or_default();
        let report = match report_rx.try_get() {
            Some(report) => report,
            _ => {
                warn!("Didn't get a report from ESP32 -> using default");
                Esp32Report::default()
            }
        };

        // Combine into UIState
        modify_ui_state(&mut ui_state, &state.hmi_state, &report);

        // Ratatui rendering
        if let Err(_) = terminal.draw(|f| render_ui(f, &state, &report, &mut ui_state)) {
            error!("Unable to draw to display");
        }

        // Manually flush the LCD screen
        if terminal.backend_mut().display_mut().flush().await.is_err() {
            error!("Unable to flush display");
        }

        // Timekeeping
        ticker.next().await;
    }
}

/// Modifies UIState based on current HMIState & Esp32Report
fn modify_ui_state(ui_state: &mut UIState, state: &HmiState, report: &Esp32Report) {
    match state.control_mode {
        ControlMode::Manual => {
            // Select motor row
            ui_state.motors.select(state.get_selected_motor_idx());

            ui_state
                .motors
                .select_column(match state.motor_selection_tab_state {
                    MotorSelectionTab::NoSelection => Some(0),
                    MotorSelectionTab::MotorSelected => Some(1),
                });

            ui_state.options.select_cell(None);
        }
        ControlMode::Vision => {
            // Clear motor selection
            ui_state.motors.select_cell(None);

            ui_state.options.select_first();
        }
    }
}

fn get_motor_action_str<const N: usize>(
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
            Forward => "↑",
            Reverse => "↓",
        },
        MotorMovementDirection::LeftRight => match direction {
            Forward => "←",
            Reverse => "→",
        },
        MotorMovementDirection::Rotational => match direction {
            Forward => "L",
            Reverse => "R",
        },
    }
}

fn render_ui(
    f: &mut ratatui::Frame<'_>,
    state: &AppState,
    report: &Esp32Report,
    ui_state: &mut UIState,
) {
    // -------- Layout -----------
    let area = f.area();

    // Split vertically into motor and esp sections
    let [motors, options] = Layout::vertical([
        Constraint::Length(5), // Motor
        Constraint::Fill(1),   // ESP
    ])
    .spacing(Spacing::Overlap(1)) // Overlap borders
    .areas(area);

    if state.hmi_state.graph_overlay_mode && state.hmi_state.control_mode == ControlMode::Vision {
        // ------ Graph -------
        render_graph(f, state, report);
    } else {
        // ------ Motors table -------
        render_motor_table(f, motors, &state.hmi_state, report, ui_state);

        // ------ Options table -------
        render_options_table(f, options, &state.hmi_state, report, ui_state);
    }
}

fn render_graph(f: &mut ratatui::Frame<'_>, state: &AppState, report: &Esp32Report) {
    let area = f.area();

    let datasets = Vec::from([Dataset::default()
        .graph_type(GraphType::Scatter)
        .marker(Marker::Braille)
        .data(state.camera_fps_data.as_slice())]);

    let chart = Chart::new(datasets)
        .x_axis(Axis::default().title("Time").bounds([0.0, 5000.0])) // TODO: sliding window
        .y_axis(Axis::default().title("Camera FPS").bounds([0.0, 10.0]));

    f.render_widget(chart, area);
}

fn render_motor_table(
    f: &mut ratatui::Frame<'_>,
    area: Rect,
    state: &HmiState,
    report: &Esp32Report,
    ui_state: &mut UIState,
) {
    let setpoints = match state.control_mode {
        ControlMode::Manual => &state.motor_setpoints,
        ControlMode::Vision => {
            if let ControlOutput::Vision(effort) = &report.control_output {
                &effort.motor_setpoints
            } else {
                &state.motor_setpoints
            }
        }
    };

    let cut_str = get_motor_action_str::<64>(&setpoints.knife, MotorMovementDirection::UpDown);
    let rot_str: String<64> = format!(
        64;
        "{}",
        get_motor_action_str::<64>(&setpoints.rotation, MotorMovementDirection::Rotational)
    )
    .unwrap();
    let lin_str: String<64> = format!(
        64;
        "{}",
        get_motor_action_str::<64>(&setpoints.translation, MotorMovementDirection::LeftRight)
    )
    .unwrap();

    let rows = [
        Row::new(["Cut", &cut_str]),
        Row::new(["Rot", &rot_str]),
        Row::new(["Lin", &lin_str]),
    ];
    let widths = [Constraint::Length(4), Constraint::Fill(1)];
    let table = Table::new(rows, widths)
        .column_spacing(0)
        .style(Color::White)
        .row_highlight_style(Style::new().on_black().bold())
        // .column_highlight_style(Color::Gray)
        .cell_highlight_style(Style::new().reversed().yellow())
        .block(
            Block::new()
                .title(
                    ratatui::text::Line::from("Setpoints")
                        .alignment(ratatui::layout::HorizontalAlignment::Left)
                        .style(if state.control_mode == ControlMode::Manual {
                            Style::new().reversed()
                        } else {
                            Style::new()
                        }),
                )
                .title(if state.enable {
                    ratatui::text::Line::from("ENABLED")
                        .style(Style::new().add_modifier(Modifier::REVERSED))
                        .alignment(ratatui::layout::HorizontalAlignment::Right)
                } else {
                    ratatui::text::Line::from("DISABLED")
                        .alignment(ratatui::layout::HorizontalAlignment::Right)
                })
                .borders(Borders::TOP | Borders::BOTTOM)
                // .border_style(Style::new().add_modifier(Modifier::REVERSED))
                .border_type(BorderType::Rounded),
        )
        .highlight_spacing(ratatui::widgets::HighlightSpacing::Always)
        .highlight_symbol("≫ ");

    // render table
    f.render_stateful_widget(table, area, &mut ui_state.motors);
}

fn render_options_table(
    f: &mut ratatui::Frame<'_>,
    area: Rect,
    state: &HmiState,
    report: &Esp32Report,
    ui_state: &mut UIState,
) {
    let vision_output_str = if let Some(vision) = &report.measurements.vision_data {
        format!(64; "{}", vision.vision_output).unwrap_or_default()
    } else {
        format!(64; "").unwrap_or_default()
    };

    let cam_stats_str = if let Some(vision) = &report.measurements.vision_data {
        format!(64; "{:>2.1}hz", vision.camera_fps).unwrap_or_default()
    } else {
        format!(64; "").unwrap_or_default()
    };

    let crossing_str = if let Some(vision) = &report.measurements.vision_data {
        format!(64; "todo").unwrap_or_default()
    } else {
        format!(64; "").unwrap_or_default()
    };

    let rows = [
        Row::new(["Mid Pos", "XXXX", &crossing_str]),
        Row::new(["Adj Spd", "YYYY", &vision_output_str]),
        Row::new(["Lead   ", "ZZZZ", &cam_stats_str]),
    ];
    let widths = [
        Constraint::Length(8),
        Constraint::Fill(1),
        Constraint::Length(9),
    ];
    let table = Table::new(rows, widths)
        .column_spacing(0)
        .style(Color::White)
        // .row_highlight_style(Style::new().on_black().bold())
        // .column_highlight_style(Color::Gray)
        .cell_highlight_style(Style::new().reversed().yellow())
        .highlight_symbol("≫ ")
        .highlight_spacing(ratatui::widgets::HighlightSpacing::Always)
        .block(
            Block::new()
                .title(
                    ratatui::text::Line::from("Options")
                        .alignment(ratatui::layout::HorizontalAlignment::Left)
                        .style(if state.control_mode == ControlMode::Vision {
                            Style::new().reversed()
                        } else {
                            Style::new()
                        }),
                )
                .title(if state.control_mode == ControlMode::Vision {
                    ratatui::text::Line::from("VISION")
                        .style(Style::new().add_modifier(Modifier::REVERSED))
                        .alignment(ratatui::layout::HorizontalAlignment::Right)
                } else {
                    ratatui::text::Line::from("MANUAL")
                        .alignment(ratatui::layout::HorizontalAlignment::Right)
                })
                .borders(Borders::TOP)
                // .border_style(Style::new().add_modifier(Modifier::REVERSED))
                .border_type(BorderType::Rounded),
        );

    f.render_stateful_widget(table, area, &mut ui_state.options);
}
