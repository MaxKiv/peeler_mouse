use crate::{
    motor::{rotation::ROTATION_SETPOINT, translation::TRANSLATION_SETPOINT},
    supervisor::{
        SelectedMotor,
        task::{APPSTATE_WATCH, MAX_ROTATION_VELOCITY_MM_PS, MAX_TRANSLATION_VELOCITY_MM_PS},
    },
};
use defmt::*;
use embassy_executor::Spawner;
use l9110::CUT_MAX_SPEED_MS_PS;
use messenger_mouse::{Setpoint, motor::MotorAction};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

pub const KNIFE_OPERATIONAL_SPEED_MM_PS: f32 = 1.0;

pub fn setup(spawner: &Spawner) {
    info!("Setting up Motor Contoller");

    spawner.spawn(control_motors()).unwrap();
}

#[embassy_executor::task]
async fn control_motors() {
    let mut appstate_rx = APPSTATE_WATCH
        .receiver()
        .expect("Increase APPSTATE_WATCH N");

    // let cut_tx = CUT_SETPOINT.sender();
    let rotation_tx = ROTATION_SETPOINT.sender();
    let translation_tx = TRANSLATION_SETPOINT.sender();

    loop {
        // Wait for a new application state to arive
        let appstate = appstate_rx.changed().await;

        let pairs = [
            (
                if appstate.enable {
                    &appstate.rotation_setpoint
                } else {
                    &MotorAction::Hold
                },
                &rotation_tx,
                SelectedMotor::Rotation,
            ),
            (
                if appstate.enable {
                    &appstate.translation_setpoint
                } else {
                    &MotorAction::Hold
                },
                &translation_tx,
                SelectedMotor::Translation,
            ),
        ];

        for (setpoint, tx, _) in pairs.iter() {
            // Construct the appropriate motor setpoint
            tx.send((*setpoint).clone())
        }
    }
}
