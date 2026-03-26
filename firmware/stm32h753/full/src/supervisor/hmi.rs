use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::Either6;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use embassy_time::Timer;
use messenger_mouse::Setpoint;
use messenger_mouse::motor::{KnifeManager, MotorCommand, MotorDirection, MotorVelocitySetpoint};
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

use crate::motor::controller::KNIFE_OPERATIONAL_SPEED_MM_PS;
use crate::supervisor::appstate::Appstate;
use crate::supervisor::task::{
    APPSTATE_WATCH, BUTTON_A, BUTTON_B, BUTTON_C, BUTTON_D, ENCODER_DATA, ENCODER_PRESSED,
    MAX_CUT_VELOCITY_MM_PS, MAX_ROTATION_VELOCITY_MM_PS, MAX_TRANSLATION_VELOCITY_MM_PS,
};
use crate::supervisor::{HmiState, SelectedMotor};

/// Main supervisor loop, manages appstate
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

    let appstate_tx = APPSTATE_WATCH.sender();

    // Initialise appstate
    let mut app_state = Appstate::default();

    // Send default appstate
    appstate_tx.send(app_state.clone());

    // Continously Process HMI inputs into appstate changes, which are then picked up elsewhere
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
            Either6::First(_) => {
                app_state.set_knife_management(match app_state.knife_manager {
                    KnifeManager::Manual => KnifeManager::Vision,
                    KnifeManager::Vision => KnifeManager::Manual,
                });
            }
            Either6::Second(_) => {
                match app_state.hmi_state {
                    HmiState::MotorSelected => {
                        // Cycles to next type of MotorCommand
                        let current_setpoint = app_state.get_current_motor_setpoint();
                        app_state.set_current_motor_setpoint(current_setpoint.next());
                    }
                    _ => (),
                };
            }
            Either6::Third(_) => {
                debug!("Supervisor - START ALL");
                app_state.start_all();
            }
            Either6::Fourth(_) => {
                if app_state.enable {
                    debug!("Supervisor - STOP ALL");
                    app_state.stop_all();
                } else {
                    debug!("Supervisor - RESET ALL");
                    app_state.reset_all();
                }
            }
            // Encoder button pressed -> Switch between HmiState
            Either6::Fifth(_) => {
                let hmi_state = match app_state.hmi_state {
                    HmiState::NoSelection => HmiState::MotorSelected,
                    HmiState::MotorSelected => HmiState::NoSelection,
                };

                app_state.set_hmi_state(hmi_state);
            }
            // Encoder count change -> Change current motor speed
            Either6::Sixth(encoder_data) => {
                match app_state.hmi_state {
                    HmiState::NoSelection => {
                        let encoder_count = encoder_data.count;

                        // Select motor based on encoder position
                        app_state
                            .selected_motor_idx(get_menu_idx_from_encoder_count(encoder_count));
                    }
                    HmiState::MotorSelected => {
                        let current_setpoint = app_state.get_current_motor_setpoint();
                        match current_setpoint {
                            MotorCommand::Halt => {
                                // Halt mode -> turning encoder does nothing
                            }
                            MotorCommand::Home => {
                                // TODO: maybe set homing velocity here?
                            }
                            MotorCommand::MoveVelocity(sp) => {
                                // Change target velocity based on encoder delta
                                let new_setpoint = calculate_new_motor_speed(
                                    sp.speed,
                                    encoder_data.filtered_delta,
                                    app_state.get_selected_motor(),
                                );

                                app_state.set_current_motor_setpoint(MotorCommand::MoveVelocity(
                                    new_setpoint.clone(),
                                ));

                                // Log change in speed
                                debug!(
                                    "Supervisor - Setting {} {}mm/s {}",
                                    app_state.selected_motor,
                                    new_setpoint.speed.get::<millimeter_per_second>(),
                                    new_setpoint.dir
                                );
                            }
                            MotorCommand::MovePosition(_sp) => {
                                // TODO: in/decrease position based on encoder delta
                            }
                        }
                    }
                };

                // Keep track of encoder position
                app_state.last_encoder_pos += encoder_data.filtered_delta;
            }
        }

        info!("APPSTATE: {:?}\n\n", app_state);

        // Application state has changed, update downstream actuators & Display
        appstate_tx.send(app_state.clone());
    }
}

/// Calculates the new motor speed after a new encoder delta is received
/// This depends on the previous and maximum motor speed.
pub fn calculate_new_motor_speed(
    current_speed: Velocity,
    encoder_delta: i16,
    selected_motor: SelectedMotor,
) -> MotorVelocitySetpoint {
    const STEP_MM_PS: f32 = 0.1;

    let (min, max) = match selected_motor {
        SelectedMotor::Translation => (
            -MAX_TRANSLATION_VELOCITY_MM_PS,
            MAX_TRANSLATION_VELOCITY_MM_PS,
        ),
        SelectedMotor::Rotation => (-MAX_ROTATION_VELOCITY_MM_PS, MAX_ROTATION_VELOCITY_MM_PS),
        SelectedMotor::Cut => (-MAX_CUT_VELOCITY_MM_PS, MAX_CUT_VELOCITY_MM_PS),
    };

    let speed = (current_speed.get::<millimeter_per_second>()
        + (STEP_MM_PS * encoder_delta as f32))
        .clamp(min, max);
    let dir = match speed {
        _ if speed >= 0.0 => MotorDirection::Reverse,
        _ => MotorDirection::Forward,
    };
    let speed = Velocity::new::<millimeter_per_second>(speed);

    let out = MotorVelocitySetpoint { dir, speed };
    info!("new setpoint for {:?}: {:?}", selected_motor, out);
    out
}

pub fn get_menu_idx_from_encoder_count(count: u16) -> i16 {
    const COUNT_MAP: [u16; 25] = [
        0, 4, 8, 12, 16, 20, 23, 27, 31, 35, 39, 43, 47, 51, 55, 59, 63, 67, 71, 75, 79, 83, 87,
        91, 95,
    ];

    let val = count % COUNT_MAP[COUNT_MAP.len() - 1];

    COUNT_MAP
        .iter()
        .position(|&x| val < x)
        .unwrap_or(COUNT_MAP.len() - 1) as i16
}
