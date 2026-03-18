// The L9110 is a simple H-bridge DC motor driver
// The control delegates the motor actuation to anything that can take a [`MotorCommand`]
// This file implements a task that translates from MotorCommand -> L9110 API

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::Delay;
use l9110::L9110;
use log::info;

use crate::control::actuation::{MotorCommand, MotorDirection, MotorState};

pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();

#[embassy_executor::task]
pub async fn manage_knife_motor(mut l9110: L9110<Delay>) {
    info!("Starting to manage knife motor");

    // start disabled
    l9110.coast();
    let mut rx = KNIFE_MOTOR_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");

    loop {
        let cmd = rx.changed().await;

        // Parse MotorCommand onto the L9110 motor driver specific API
        match &cmd.state {
            MotorState::Braking => l9110.short_break().await,
            MotorState::Coasting => l9110.coast(),
            MotorState::Enabled => {
                if cmd.dir == MotorDirection::Forward {
                    l9110.forward(cmd.speed)
                } else {
                    l9110.reverse(cmd.speed)
                }
            }
        };
    }
}
