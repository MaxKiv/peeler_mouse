use messenger_mouse::motor::MotorDirection;
use uom::si::f32::Velocity;

pub mod limit_switch_task;
pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

#[derive(Clone, Debug)]
enum MotorAction {
    Stop,
    Velocity {
        dir: MotorDirection,
        speed: Velocity,
    },
    Home,
}

#[derive(Clone, Debug, PartialEq)]
pub enum HomeStatus {
    Lost,
    Homed { position: i32 },
}
