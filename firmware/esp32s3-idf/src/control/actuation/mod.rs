use messenger_mouse::VisionAlgorithmOutput;
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

pub mod motor_controller;
pub mod motor_task;

/// Operational states a motor can be in
#[derive(Debug, Clone, Default)]
pub enum MotorState {
    Enabled,
    Braking,
    #[default]
    Coasting,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub enum MotorDirection {
    #[default]
    Forward,
    Backward,
}

impl MotorDirection {
    pub fn reverse(&mut self) {
        use MotorDirection::*;
        *self = match self {
            Forward => Backward,
            Backward => Forward,
        };
    }
}

impl Into<l9110::Direction> for MotorDirection {
    fn into(self) -> l9110::Direction {
        match self {
            Self::Forward => l9110::Direction::Forward,
            Self::Backward => l9110::Direction::Reverse,
        }
    }
}

/// Commands all motors drivers should be able to accept
#[derive(Debug, Clone, Default)]
pub struct MotorCommand {
    /// Operational state of the motor, i.e. is it enabled? Is it braking?
    pub state: MotorState,
    /// Direction of axis rotation
    pub dir: MotorDirection,
    /// Speed of the motor
    pub speed: Velocity,
}

impl MotorCommand {
    pub fn from_vision_output(vision_output: VisionAlgorithmOutput) -> Self {
        let default_knife_motor_speed: Velocity = Velocity::new::<millimeter_per_second>(1.0);

        let state = match vision_output {
            VisionAlgorithmOutput::Hold => MotorState::Coasting,
            _ => MotorState::Enabled,
        };

        let dir = match vision_output {
            VisionAlgorithmOutput::Up => MotorDirection::Forward,
            _ => MotorDirection::Backward,
        };

        Self {
            state,
            dir,
            speed: default_knife_motor_speed,
        }
    }
}
