use messenger_mouse::motor::MotorDirection;
use uom::si::f32::Velocity;

use crate::supervisor::MotorType;

#[derive(Clone)]
pub enum AppCmd {
    SelectMotor(MotorType),
    StopAll,
    SetSpeed(Velocity),
    SetDirection(MotorDirection),
}
