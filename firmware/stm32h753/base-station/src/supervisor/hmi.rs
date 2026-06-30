use defmt::*;
use embassy_futures::select::Either6;
use messenger_mouse::motor::{ControlMode, MotorAction, MotorDirection, MotorVelocitySetpoint};
use uom::si::f32::{Length, Velocity};
use uom::si::length::micrometer;
use uom::si::velocity::millimeter_per_second;

use crate::supervisor::task::{
    BUTTON_A, BUTTON_B, BUTTON_C, BUTTON_D, ENCODER_DATA, ENCODER_PRESSED, HMI_STATE_WATCH,
    MAX_CUT_VELOCITY_MM_PS, MAX_ROTATION_VELOCITY_MM_PS, MAX_TRANSLATION_VELOCITY_MM_PS,
};
use crate::supervisor::{HmiState, MotorSelectionTab, MotorTypes};

/// Main supervisor loop, maps HMI inputs to Hmi state changes
#[embassy_executor::task]
pub async fn supervise_hmi() {
    let mut button_a_selected_rx = BUTTON_A.receiver().expect("Increase BUTTON_A N");
    let mut button_b_selected_rx = BUTTON_B.receiver().expect("Increase BUTTON_B N");
    let mut button_c_selected_rx = BUTTON_C.receiver().expect("Increase BUTTON_C N");
    let mut button_d_selected_rx = BUTTON_D.receiver().expect("Increase BUTTON_D N");
    let mut encoder_pressed_rx = ENCODER_PRESSED
        .receiver()
        .expect("Increase encoder_pressed N");

    let mut encoder_data_rx = ENCODER_DATA.receiver().expect("Increase encoder_data N");

    let hmi_tx = HMI_STATE_WATCH.sender();

    // Initialise hmi state
    let mut hmi_state = HmiState::default();

    // Send default appstate
    hmi_tx.send(hmi_state.clone());

    // Main HMI loop
    // Continously Process HMI inputs into hmi_state changes, which are then picked up elsewhere
    loop {
        // Wait for a HMI input that we need to process
        match embassy_futures::select::select6(
            button_a_selected_rx.changed(),
            button_b_selected_rx.changed(),
            button_c_selected_rx.changed(),
            button_d_selected_rx.changed(),
            encoder_pressed_rx.changed(),
            encoder_data_rx.changed(),
        )
        .await
        {
            // Vision button
            Either6::First(_) => {
                hmi_state.set_control_mode(match hmi_state.control_mode {
                    ControlMode::Manual => ControlMode::Vision,
                    ControlMode::Vision => ControlMode::Manual,
                });
            }

            // Mode button
            Either6::Second(_) => {
                match hmi_state.control_mode {
                    ControlMode::Manual => {
                        // Cycles to next type / mode of MotorAction for currently selected motor
                        let current = hmi_state.get_current_motor_action();
                        hmi_state.set_current_motor_action(current.next());
                    }
                    ControlMode::Vision => {
                        // Cycle between normal and graph overlay mode
                        hmi_state.overlay_mode = hmi_state.overlay_mode.next();
                    }
                }
            }

            // Start button, enables all motors
            Either6::Third(_) => {
                debug!("Supervisor - START ALL");
                hmi_state.start_all();
            }

            // Stop button, stops all motors
            Either6::Fourth(_) => {
                if hmi_state.enable {
                    debug!("Supervisor - STOP ALL");
                    hmi_state.stop_all();
                } else {
                    debug!("Supervisor - RESET ALL");
                    hmi_state.reset_all();
                }
            }

            // Encoder button pressed -> Switch motor selection tab
            Either6::Fifth(_) => {
                hmi_state.next_motor_selection_tab();
            }

            // Encoder count change
            // MotorSelectionTab::NoSelection => select new motor
            // MotorSelectionTab::MotorSelected => change selected motor speed
            Either6::Sixth(encoder_data) => {
                let (encoder_pos, encoder_delta) =
                    get_encoder_pos_delta(encoder_data.count, hmi_state.encoder_pos);
                debug!(
                    "SV: encoder_count: {} - encoder_pos: {} - delta: {}",
                    encoder_data.count, encoder_pos, encoder_delta
                );

                match hmi_state.motor_selection_tab_state {
                    // No motor selected => select new motor
                    MotorSelectionTab::NoSelection => {
                        // Select motor based on encoder position
                        hmi_state.select_new_motor_idx(encoder_pos);
                    }

                    // Motor selected => change speed
                    MotorSelectionTab::MotorSelected => {
                        let current_action = hmi_state.get_current_motor_action();
                        match current_action {
                            MotorAction::Hold | MotorAction::Coast | MotorAction::Home => {
                                // Turning encoder does nothing
                            }
                            MotorAction::MoveVelocity(sp) => {
                                // Change target velocity based on encoder delta
                                let new_setpoint = calculate_new_motor_speed(
                                    sp.speed,
                                    encoder_delta,
                                    hmi_state.get_selected_motor(),
                                );

                                hmi_state.set_current_motor_action(MotorAction::MoveVelocity(
                                    new_setpoint.clone(),
                                ));

                                // Log change in speed
                                debug!(
                                    "Supervisor - Setting {} {}mm/s {}",
                                    hmi_state.selected_motor,
                                    new_setpoint.speed.get::<millimeter_per_second>(),
                                    new_setpoint.dir
                                );
                            }
                            MotorAction::MovePosition(mut sp) => {
                                sp.target +=
                                    Length::new::<micrometer>((encoder_delta * 100).into());
                                sp.speed = Velocity::new::<millimeter_per_second>(
                                    messenger_mouse::motor::POSITION_MODE_VELOCITY_MM_PS,
                                );

                                hmi_state.set_current_motor_action(MotorAction::MovePosition(sp));
                            }
                        }
                    }
                };

                // Keep track of encoder position
                hmi_state.encoder_pos = encoder_pos;
            }
        }

        debug!("HMI STATE: {:?}\n\n", hmi_state);

        // Application state has changed, update downstream actuators & Display
        hmi_tx.send(hmi_state.clone());
    }
}

/// Calculates the new motor speed after a new encoder delta is received
/// This depends on the previous and maximum motor speed.
pub fn calculate_new_motor_speed(
    current_speed: Velocity,
    encoder_delta: i16,
    selected_motor: &MotorTypes,
) -> MotorVelocitySetpoint {
    const ROT_STEP_MM_PS: f32 = 0.1;
    const LIN_STEP_MM_PS: f32 = 0.01;
    const CUT_STEP_MM_PS: f32 = 0.1;

    let (min, max, step) = match selected_motor {
        MotorTypes::Translation => (
            -MAX_TRANSLATION_VELOCITY_MM_PS,
            MAX_TRANSLATION_VELOCITY_MM_PS,
            LIN_STEP_MM_PS,
        ),
        MotorTypes::Rotation => (
            -MAX_ROTATION_VELOCITY_MM_PS,
            MAX_ROTATION_VELOCITY_MM_PS,
            ROT_STEP_MM_PS,
        ),
        MotorTypes::Cut => (
            -MAX_CUT_VELOCITY_MM_PS,
            MAX_CUT_VELOCITY_MM_PS,
            CUT_STEP_MM_PS,
        ),
    };

    let mut speed = (current_speed.get::<millimeter_per_second>() + (step * encoder_delta as f32))
        .clamp(min, max);

    // Hysteresis
    if speed.abs() < step / 2.0 {
        speed = 0.0;
    }

    let dir = match speed {
        _ if speed >= 0.0 => MotorDirection::Forward,
        _ => MotorDirection::Reverse,
    };
    let speed = Velocity::new::<millimeter_per_second>(speed);

    let out = MotorVelocitySetpoint { dir, speed };
    info!("new setpoint for {:?}: {:?}", selected_motor, out);
    out
}

pub fn get_encoder_pos_delta(count: i16, last_pos: i16) -> (i16, i16) {
    // const COUNT_MAP: [i16; 25] = [
    //     0, 4, 8, 12, 16, 20, 23, 27, 31, 35, 39, 43, 47, 51, 55, 59, 63, 67, 71, 75, 79, 83, 87,
    //     91, 95,
    // ];
    const COUNT_MAP: [i16; 25] = [
        0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80, 84, 88,
        92, 96,
    ];
    const LEN: i16 = COUNT_MAP.len() as i16;

    let val = (100 * LEN + count) % COUNT_MAP[COUNT_MAP.len() - 1];

    let new_pos = COUNT_MAP
        .iter()
        .position(|&x| val < x)
        .unwrap_or(COUNT_MAP.len() - 1) as i16;

    let mut delta = new_pos - last_pos;

    if delta > LEN / 2 {
        delta -= LEN;
    } else if delta < -LEN / 2 {
        delta += LEN;
    }

    (new_pos, delta)
}
