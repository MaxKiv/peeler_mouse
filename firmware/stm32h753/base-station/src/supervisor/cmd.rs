use messenger_mouse::motor::MotorDirection;
use uom::si::f32::Velocity;

use crate::supervisor::SelectedMotor;

#[derive(Clone)]
pub enum AppCmd {
    SelectMotor(SelectedMotor),
    StopAll,
    SetSpeed(Velocity),
    SetDirection(MotorDirection),
}
