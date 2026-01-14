use crate::{
    motor::{
        MotorCommand, MotorState, knife::KNIFE_SETPOINT, rotation::ROTATION_SETPOINT,
        translation::TRANSLATION_SETPOINT,
    },
    supervisor::{APPSTATE_WATCH, MotorSetpoint},
};
use defmt::*;
use embassy_executor::Spawner;

pub fn setup(spawner: &Spawner) {
    info!("Setting up Motor Contoller");

    spawner.spawn(control_motors()).unwrap();
}

#[embassy_executor::task]
async fn control_motors() {
    let mut appstate_rx = APPSTATE_WATCH
        .receiver()
        .expect("Increase APPSTATE_WATCH N");

    let cut_tx = KNIFE_SETPOINT.sender();
    let rotation_tx = ROTATION_SETPOINT.sender();
    let translation_tx = TRANSLATION_SETPOINT.sender();

    loop {
        // Wait for a new application state to arive
        let appstate = appstate_rx.changed().await;

        let pairs = [
            (&appstate.rotation_setpoint, &rotation_tx),
            (&appstate.translation_setpoint, &translation_tx),
            (&appstate.cut_setpoint, &cut_tx),
        ];

        for (setpoint, tx) in pairs.iter() {
            // Construct the appropriate motor setpoint
            let MotorSetpoint {
                enabled,
                speed,
                dir,
            } = setpoint;

            // Map motor disabled -> coasting
            let state = match enabled {
                true => MotorState::Enabled,
                false => MotorState::Coasting,
            };
            let cmd = MotorCommand {
                speed: *speed,
                state,
                dir: dir.clone(),
            };

            // Send the command to the right motor
            tx.send(cmd)
        }
    }
}
