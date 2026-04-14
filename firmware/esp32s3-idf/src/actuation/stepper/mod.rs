use embassy_executor::Spawner;
use messenger_mouse::motor::MotorDirection;
use uom::si::f32::{Length, Velocity};

use crate::actuation::stepper::{
    limit_switch_task::manage_limit_switch, low_level::low_level_task::low_lvl_stepper_task,
    motor_task::control_knife_motor, peripherals::MotorPeripherals,
};

pub mod limit_switch_task;
pub mod low_level;
pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

/// Homing status of the motor controller
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HomeStatus {
    Lost,
    Homed { position: i32 },
}

// Position mode status of the motor controller
#[derive(Clone, Debug, PartialEq)]
pub enum PositionModeStatus {
    InProgress,
    Reached,
}

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: MotorPeripherals) -> anyhow::Result<()> {
    log::info!("initialising knife motor tasks");

    spawner.spawn(low_lvl_stepper_task(p.stepper))?;
    spawner.spawn(manage_limit_switch(p.limit_switch))?;
    spawner.spawn(control_knife_motor())?;

    Ok(())
}
