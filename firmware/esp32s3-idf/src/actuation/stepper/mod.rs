use messenger_mouse::motor::MotorDirection;
use uom::si::f32::{Length, Velocity};

pub mod limit_switch_task;
pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

#[derive(Clone, Copy, PartialEq, PartialOrd, Debug)]
pub struct Steps(pub i32);

/// Actions the main control loop can ask the motor controller to perform
#[derive(Clone, Debug, Default)]
pub enum MotorAction {
    #[default]
    Stop,
    Velocity {
        dir: MotorDirection,
        speed: Velocity,
    },
    Position {
        target: Length,
        speed: Velocity,
    },
    Home,
}

/// Homing status of the motor controller
#[derive(Clone, Debug, PartialEq)]
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
