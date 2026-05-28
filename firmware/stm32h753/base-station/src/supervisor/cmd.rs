use messenger_mouse::motor::MotorDirection;
use uom::si::f32::Velocity;

use crate::supervisor::MotorTypes;

#[derive(Clone)]
pub enum AppCmd {
    SelectMotor(MotorTypes),
    StopAll,
    SetSpeed(Velocity),
    SetDirection(MotorDirection),
}
