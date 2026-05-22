use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};

use crate::actuation::stepper::{
    low_level::low_level_task::low_lvl_stepper_task, motor_task::control_knife_motor,
    peripherals::MotorPeripherals,
};

pub mod limit_encoder_task;
pub mod limit_switch_task;
pub mod low_level;
pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

pub static LIMIT_EVENT: Watch<CriticalSectionRawMutex, LimitSwitchState, 1> = Watch::new();

#[derive(Clone, PartialEq, Debug)]
pub enum LimitSwitchState {
    Active,
    Inactive,
}

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
    spawner.spawn(control_knife_motor())?;

    #[cfg(feature = "limit_switch")]
    spawner.spawn(manage_limit_switch(p.limit_switch))?;

    #[cfg(feature = "home_encoder_stall")]
    {
        use crate::actuation::stepper::limit_encoder_task::{
            // encoder_limit_switch,
            monitor_encoder_stall,
        };

        spawner.spawn(monitor_encoder_stall())?;
    }

    Ok(())
}
