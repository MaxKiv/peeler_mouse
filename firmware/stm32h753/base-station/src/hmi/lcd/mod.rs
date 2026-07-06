pub mod setup;
pub mod startup;

extern crate alloc;
use alloc::vec::Vec;
use embassy_futures::select::{Either, select};

use defmt::*;
use display_interface_i2c::I2CInterface;
use embassy_stm32::{
    i2c::{I2c, Master},
    mode::Async,
};
use embassy_sync::{
    blocking_mutex::raw::ThreadModeRawMutex,
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker};
use embedded_graphics::Drawable;
use embedded_graphics::pixelcolor::BinaryColor;
use heapless::{String, format};
use messenger_mouse::{
    ControlOutput, Esp32Report, FRAME_SIZE,
    motor::{ControlMode, MotorAction, MotorDirection},
};
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use oled_async::{displays::ssd1309::Ssd1309_128_64, mode::GraphicsMode};
use ratatui::{
    Terminal,
    layout::{Constraint, Layout, Rect, Spacing},
    style::{Color, Modifier, Style, Stylize},
    symbols::Marker,
    widgets::{
        Axis, Block, BorderType, Borders, Chart, Dataset, GraphType, Row, Table, TableState,
    },
};
use uom::si::length::millimeter;
use uom::si::velocity::millimeter_per_second;

use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as Cs;

use crate::{
    hmi::{
        button::BUTTON_WATCH_SIZE,
        lcd::{setup::SSD1309_FRAMEBUFFER_SIZE, startup::startup_display},
    },
    ringbuffer::RingBuffer,
    supervisor::{HmiState, OverlayMode, SelectionState, appstate::AppState},
};

pub static LCD_INPUT: Watch<ThreadModeRawMutex, bool, { BUTTON_WATCH_SIZE }> = Watch::new();

const LCD_PERIOD: Duration = Duration::from_millis(200);

// UI related
const TEXT_OFFSET_WIDTH: i32 = 4;
const TEXT_OFFSET_HEIGHT: i32 = 14;
const REPORT_WINDOW_N: usize = 32;

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

pub struct AggregatedMeasurements {
    pub tl_data: RingBuffer<f64, REPORT_WINDOW_N>,
    pub camera_fps_data: RingBuffer<f64, REPORT_WINDOW_N>,
    pub tearing_detected_data: RingBuffer<f64, REPORT_WINDOW_N>,
    pub tl_error_data: RingBuffer<f64, REPORT_WINDOW_N>,
    pub report_cnt: usize,
}

pub struct UIState {
    pub motors: TableState,
    pub parameters: TableState,
    pub aggregated_measurements: AggregatedMeasurements,
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
    backend_cfg.font_regular = embedded_graphics_unicodefonts::MONO_5X8;
    backend_cfg.font_bold = Some(embedded_graphics_unicodefonts::MONO_5X8);
    backend_cfg.font_italic = Some(embedded_graphics_unicodefonts::MONO_5X8);
    let backend = EmbeddedBackend::new(&mut display, backend_cfg);
    let mut terminal = Terminal::new(backend).unwrap();

    // Set up ui state
    let mut ui_state = UIState {
        motors: TableState::new().with_selected(Some(1)),
        parameters: TableState::new().with_selected(None),
        aggregated_measurements: AggregatedMeasurements {
            tl_data: RingBuffer::new(),
            camera_fps_data: RingBuffer::new(),
            tearing_detected_data: RingBuffer::new(),
            tl_error_data: RingBuffer::new(),
            report_cnt: 0,
        },
    };
    ui_state.motors.select_first();
    ui_state.motors.select_first_column();

    info!("Starting Display render loop");

    let mut ticker = Ticker::every(LCD_PERIOD);

    // UI render loop
    loop {
        match select(ticker.next(), report_rx.changed()).await {
            // Ticker expired
            Either::First(_) => {
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
                modify_ui_state(&mut ui_state, &state.hmi_state);

                let start = embassy_time::Instant::now();
                // Ratatui rendering
                if let Err(_) = terminal.draw(|f| render_ui(f, &state, &report, &mut ui_state)) {
                    error!("Unable to draw to display");
                }
                let now = embassy_time::Instant::now();
                let duration = now - start;
                debug!("RENDER: render_ui took {}us", duration.as_micros());

                // Manually flush the LCD screen, important to do async
                if terminal.backend_mut().display_mut().flush().await.is_err() {
                    error!("Unable to flush display");
                }
            }
            // New report -> track latest measurements for displaying
            Either::Second(new_report) => {
                if let Some(vision_data) = new_report.measurements.vision_data {
                    let _ = ui_state
                        .aggregated_measurements
                        .camera_fps_data
                        .push(vision_data.camera_fps);

                    let tl_px = vision_data
                        .vision_output
                        .transition_line_height_px
                        .unwrap_or_default();
                    let zero_px = vision_data.vision_output.zero_line_height_px;

                    let _ = ui_state.aggregated_measurements.tl_data.push(tl_px as f64);

                    let error: i32 = zero_px as i32 - tl_px as i32;

                    let _ = ui_state
                        .aggregated_measurements
                        .tl_error_data
                        .push(error as f64);

                    let _ = ui_state
                        .aggregated_measurements
                        .tearing_detected_data
                        .push(vision_data.vision_output.tearing_detected as usize as f64);
                }
                ui_state.aggregated_measurements.report_cnt += 1;
            }
        }
    }
}

/// Modifies UIState based on current HMIState & Esp32Report
fn modify_ui_state(ui_state: &mut UIState, state: &HmiState) {
    match state.control_mode {
        ControlMode::Manual => {
            // Select motor row
            ui_state.motors.select(state.get_selected_motor_idx());

            ui_state
                .motors
                .select_column(match state.motor_selection_state {
                    SelectionState::NoSelection => Some(0),
                    SelectionState::Selected => Some(1),
                });

            ui_state.parameters.select_cell(None);
        }
        ControlMode::Vision => {
            // Clear motor selection
            ui_state.motors.select_cell(None);

            ui_state
                .parameters
                .select_column(match state.parameter_selection_state {
                    SelectionState::NoSelection => Some(0),
                    SelectionState::Selected => Some(1),
                });

            ui_state
                .parameters
                .select(state.get_selected_parameter_idx());
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

    match state.hmi_state.overlay_mode {
        OverlayMode::Default => {
            // TODO: rendering tables seem to take ~20ms per table, this can f up comms since its synchronous
            // I see two possible fixes:
            // 1. move away from ratatui::table
            // 2. Replace ratatui::Terminal::draw() with its components, insert awaits
            // For now I accept life sucks, increase comms ringbuffer size and move on
            // ------ Motors table -------
            render_motor_table(f, motors, &state.hmi_state, report, ui_state);

            // ------ Options table -------
            render_options_table(f, options, &state.hmi_state, report, ui_state);
        }
        OverlayMode::CameraFPS => render_graph_camera_fps(f, &ui_state.aggregated_measurements),
        OverlayMode::TransitionLine => {
            render_graph_transition_line(f, &ui_state.aggregated_measurements)
        }
        OverlayMode::TransitionError => {
            render_graph_transition_error(f, &ui_state.aggregated_measurements)
        }
        OverlayMode::TearingDetection => {
            render_graph_tearing_detection(f, &ui_state.aggregated_measurements)
        }
    }
}

fn render_graph_tearing_detection(
    f: &mut ratatui::Frame<'_>,
    measurements: &AggregatedMeasurements,
) {
    const TEARING_MAX: f64 = 2.0;
    const TEARING_MIN: f64 = 1.0;
    let area = f.area();

    // construct ratatui dataset bs type
    let mut dataset_scratch = [(0f64, 0f64); REPORT_WINDOW_N];
    let dataset = construct_ratatui_dataset(
        &measurements.tearing_detected_data,
        &mut dataset_scratch[..],
    );

    let datasets = Vec::from([Dataset::default()
        .graph_type(GraphType::Line)
        .marker(Marker::Custom('X'))
        .data(dataset)]);

    let chart = Chart::new(datasets)
        .x_axis(Axis::default().bounds([0.0, measurements.tearing_detected_data.len() as f64]))
        .y_axis(
            Axis::default()
                .title("Tearing Detected?")
                .bounds([TEARING_MIN, TEARING_MAX]),
        );

    f.render_widget(chart, area);
}

fn render_graph_transition_error(
    f: &mut ratatui::Frame<'_>,
    measurements: &AggregatedMeasurements,
) {
    const ERR_MAX: f64 = ((FRAME_SIZE.get_dimensions().1 + 30) / 2) as f64;
    const ERR_MIN: f64 = -(((FRAME_SIZE.get_dimensions().1 + 30) / 2) as f64);
    let area = f.area();

    // construct ratatui dataset bs type
    let mut dataset_scratch = [(0f64, 0f64); REPORT_WINDOW_N];
    let dataset = construct_ratatui_dataset(&measurements.tl_error_data, &mut dataset_scratch[..]);

    let datasets = Vec::from([Dataset::default()
        .graph_type(GraphType::Line)
        .marker(Marker::Braille)
        .data(dataset)]);

    let chart = Chart::new(datasets)
        .x_axis(Axis::default().bounds([0.0, measurements.tl_error_data.len() as f64]))
        .y_axis(
            Axis::default()
                .title("Error [0-240] px)")
                .bounds([ERR_MIN, ERR_MAX]),
        );

    f.render_widget(chart, area);
}

fn render_graph_transition_line(f: &mut ratatui::Frame<'_>, measurements: &AggregatedMeasurements) {
    const TL_MAX: f64 = (FRAME_SIZE.get_dimensions().1 + 30) as f64;
    let area = f.area();

    // construct ratatui dataset bs type
    let mut dataset_scratch = [(0f64, 0f64); REPORT_WINDOW_N];
    let dataset = construct_ratatui_dataset(&measurements.tl_data, &mut dataset_scratch[..]);

    let datasets = Vec::from([Dataset::default()
        .graph_type(GraphType::Scatter)
        .marker(Marker::Braille)
        .data(dataset)]);

    let chart = Chart::new(datasets)
        .x_axis(Axis::default().bounds([0.0, measurements.tl_data.len() as f64]))
        .y_axis(
            Axis::default()
                .title("Transition [0-240] px)")
                .bounds([0.0, TL_MAX]),
        );

    f.render_widget(chart, area);
}

fn render_graph_camera_fps(f: &mut ratatui::Frame<'_>, measurements: &AggregatedMeasurements) {
    let area = f.area();

    // construct ratatui dataset bs type
    let mut cam_data_scratch = [(0f64, 0f64); REPORT_WINDOW_N];
    let cam_dataset =
        construct_ratatui_dataset(&measurements.camera_fps_data, &mut cam_data_scratch[..]);

    let datasets = Vec::from([Dataset::default()
        .graph_type(GraphType::Line)
        .marker(Marker::Braille)
        .data(cam_dataset)]);

    let chart = Chart::new(datasets)
        .x_axis(Axis::default().bounds([0.0, measurements.camera_fps_data.len() as f64]))
        .y_axis(
            Axis::default()
                .title("Camera FPS [1-5] Hz")
                .bounds([1.0, 4.0]),
        );

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
                .border_type(BorderType::Plain),
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
        if let Some(sp) = &vision.vision_output.knife_setpoint {
            format!(64; "{}", sp).unwrap_or_default()
        } else {
            format!(64; "None").unwrap_or_default()
        }
    } else {
        format!(64; "").unwrap_or_default()
    };

    let cam_stats_str = if let Some(vision) = &report.measurements.vision_data {
        format!(64; "{:>2.1}hz", vision.camera_fps).unwrap_or_default()
    } else {
        format!(64; "").unwrap_or_default()
    };

    let zc_str = format!(64; "{}px",
        state.parameter_setpoints.zero_line_px)
    .unwrap_or_default();

    let gain_str = format!(64; "{}",
        state.parameter_setpoints.gain)
    .unwrap_or_default();

    let lead_str = format!(64; "{:>2.1}",
        state.parameter_setpoints.lead)
    .unwrap_or_default();

    let tl_str = if let Some(vision) = &report.measurements.vision_data {
        if let Some(tld) = vision.vision_output.transition_line_height_px {
            format!(64; "TL {}px", tld).unwrap_or_default()
        } else {
            format!(64; "TL None").unwrap_or_default()
        }
    } else {
        format!(64; "").unwrap_or_default()
    };

    let rows = [
        Row::new(["ZC SP ", &zc_str, &tl_str]),
        Row::new(["Gain  ", &gain_str, &vision_output_str]),
        Row::new(["Lead  ", &lead_str, &cam_stats_str]),
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

    f.render_stateful_widget(table, area, &mut ui_state.parameters);
}

fn construct_ratatui_dataset<'a, const N: usize>(
    data: &RingBuffer<f64, N>,
    dataset: &'a mut [(f64, f64)],
) -> &'a [(f64, f64)] {
    for i in 0..data.len() {
        dataset[i].0 = i as f64;
        dataset[i].1 = data.peek(i).unwrap().clone();
    }

    &dataset[..data.len()]
}
