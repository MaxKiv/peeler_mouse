use crate::{
    comms::task::ESP_SETPOINT_WATCH,
    motor::{rotation::ROTATION_SETPOINT, translation::TRANSLATION_SETPOINT},
    supervisor::appstate::APP_STATE_WATCH,
};
use defmt::*;
use embassy_executor::Spawner;
use messenger_mouse::motor::ControlMode;
use messenger_mouse::{Esp32Setpoint, motor::MotorAction};

pub fn setup(spawner: &Spawner) {
    info!("Setting up Motor Contoller");

    spawner.spawn(control_motors()).unwrap();
}

#[embassy_executor::task]
/// Apstate -> MotorAction Adapter for Translational & Rotation motors
async fn control_motors() {
    let mut appstate_rx = APP_STATE_WATCH.receiver().unwrap();

    let rotation_tx = ROTATION_SETPOINT.sender();
    let translation_tx = TRANSLATION_SETPOINT.sender();
    let esp_tx = ESP_SETPOINT_WATCH.sender();

    // Send known setpoints on boot
    rotation_tx.send(MotorAction::Coast);
    translation_tx.send(MotorAction::Coast);
    esp_tx.send(Esp32Setpoint::default());

    loop {
        // Wait for a new application state to arive
        let new_appstate = appstate_rx.changed().await;

        info!("MOTOR: new appstate {:?}", new_appstate);

        // Is hmi enable toggled on?
        if new_appstate.hmi_enable {
            // Send ESP32 its setpoint
            // For control_mode manual => ESP follows HMI setpoint
            // For control_mode vision => ESP calculates its own setpoint
            esp_tx.send(Esp32Setpoint {
                control_mode: new_appstate.hmi_control_mode.clone(),
                knife_setpoint: new_appstate.hmi_motor_setpoints.knife,
            });

            // Send translational/rotational motors their setpoints
            let (rotation_setpoint, translation_setpoint) =
                if new_appstate.hmi_control_mode == ControlMode::Manual {
                    // For control_mode manual => motors follows HMI setpoint
                    (
                        new_appstate.hmi_motor_setpoints.rotation,
                        new_appstate.hmi_motor_setpoints.translation,
                    )
                } else {
                    // For control_mode manual => motors follows setpoints calculated by ESP
                    (
                        new_appstate.esp_motor_setpoints.rotation,
                        new_appstate.esp_motor_setpoints.translation,
                    )
                };

            info!(
                "MOTOR: Setting ROT {:?} - TRANS {:?}",
                rotation_setpoint, translation_setpoint
            );
            rotation_tx.send(rotation_setpoint);
            translation_tx.send(translation_setpoint);
        } else {
            // HMI disable -> set motors to known safe state
            esp_tx.send(Esp32Setpoint::new_safe());
            rotation_tx.send(MotorAction::Coast);
            translation_tx.send(MotorAction::Coast);
        }
    }
}
