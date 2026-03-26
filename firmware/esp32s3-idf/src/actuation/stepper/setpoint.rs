use uom::si::{
    f32::{Length, Velocity},
    velocity::millimeter_per_second,
};

use crate::actuation::stepper::motor_task::OPERATION_SPEED_MM_PS;
use messenger_mouse::motor::MotorDirection;

/// Velocity movement setpoint
#[derive(Debug, Clone, Default)]
pub struct MotorVelocitySetpoint {
    /// Direction of axis rotation
    pub dir: MotorDirection,
    /// Speed of the motor
    pub speed: Velocity,
}

/// Position movement setpoint
#[derive(Debug, Clone, Default)]
pub struct MotorPositionSetpoint {
    /// Position target wrt home position
    pub target: Length,
    /// Speed of the motor
    pub speed: Velocity,
}

impl MotorVelocitySetpoint {
    // Currently only a single speed is supported
    pub fn new(dir: MotorDirection) -> Self {
        Self {
            dir,
            speed: Velocity::new::<millimeter_per_second>(OPERATION_SPEED_MM_PS),
        }
    }

    pub fn new_forward() -> Self {
        Self::new(MotorDirection::Forward)
    }

    pub fn new_reverse() -> Self {
        Self::new(MotorDirection::Reverse)
    }
}
