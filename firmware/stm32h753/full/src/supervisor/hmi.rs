use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::Either6;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use embassy_time::Timer;
use messenger_mouse::motor::{
    KnifeManagementState, MotorCommand, MotorDirection, MotorVelocitySetpoint,
};
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

use crate::motor::controller::KNIFE_OPERATIONAL_SPEED_MM_PS;
use crate::supervisor::appstate::Appstate;
use crate::supervisor::task::{
    APPSTATE_WATCH, BUTTON_A, BUTTON_B, BUTTON_C, BUTTON_D, ENCODER_DATA, ENCODER_PRESSED,
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
            Either6::First(_) => {}
            Either6::Second(_) => {
                app_state.set_knife_management(match app_state.knife_management_state {
                    KnifeManagementState::Manual(_) => {
                        debug!("Supervisor - TOGGLE KNIFE MANAGEMENT -> VISION");
                        KnifeManagementState::Vision
                    }
                    KnifeManagementState::Vision => {
                        debug!("Supervisor - TOGGLE KNIFE MANAGEMENT -> MANUAL");
                        if app_state.enable && app_state.cut_setpoint.enabled {
                            let setpoint = construct_knife_setpoint_from_appstate(&app_state);
                            KnifeManagementState::Manual(MotorCommand::MoveVelocity(setpoint))
                        } else {
                            KnifeManagementState::Manual(MotorCommand::Halt)
                        }
                    }
                })
            }
            Either6::Third(_) => {
                debug!("Supervisor - START ALL");
                app_state.start_all();
            }
            Either6::Fourth(_) => {
                if app_state.enable {
                    debug!("Supervisor - STOP ALL");
                    app_state.stop_all();
                    // Return knife management to Stm32
                    if let KnifeManagementState::Vision = app_state.knife_management_state {
                        app_state
                            .set_knife_management(KnifeManagementState::Manual(MotorCommand::Halt));
                    } else {
                        let setpoint = construct_knife_setpoint_from_appstate(&app_state);
                        app_state.set_knife_management(KnifeManagementState::Manual(
                            MotorCommand::MoveVelocity(setpoint),
                        ));
                    }
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
                        // Calculate new speed for this motor
                        let selected_motor = app_state.get_selected_motor();
                        let mut setpoint = app_state.get_current_motor_setpoint();

                        setpoint.speed_percentage = calculate_new_motor_speed_percentage(
                            selected_motor,
                            setpoint.speed_percentage,
                            encoder_data.filtered_delta,
                        );

                        if setpoint.speed_percentage < 0.0 {
                            // debug!("Supervisor - speed {} > 0.0", setpoint.speed_percentage);
                            setpoint.dir = MotorDirection::Reverse;
                        }

                        // Log change in speed
                        debug!(
                            "Supervisor - Setting {} state {} dir {} speed {}%",
                            app_state.selected_motor,
                            setpoint.enabled,
                            setpoint.dir,
                            setpoint.speed_percentage
                        );

                        app_state.set_current_motor_setpoint(setpoint);
                    }
                };

                // Keep track of encoder position
                app_state.last_encoder_pos += encoder_data.filtered_delta;
            }
        }

        // Application state has changed, update downstream actuators & Display
        appstate_tx.send(app_state.clone());
    }
}

/// Calculates the new motor speed after a new encoder delta is received
/// This depends on the previous and maximum motor speed.
pub fn calculate_new_motor_speed_percentage(
    _selected_motor: SelectedMotor,
    current_speed_percentage: f32,
    encoder_delta: i16,
) -> f32 {
    const STEP: f32 = 1.0;

    (current_speed_percentage + (STEP * encoder_delta as f32)).clamp(-100.0, 100.0)
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
