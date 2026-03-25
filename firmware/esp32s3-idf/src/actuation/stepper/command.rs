use messenger_mouse::VisionAlgorithmOutput;

use crate::actuation::stepper::setpoint::MotorVelocitySetpoint;

#[derive(Debug, Clone)]
pub enum MotorCommand {
    Halt,
    Home,
    MoveVelocity(MotorVelocitySetpoint),
    // MovePosition(MotorPositionSetpoint),
}

impl MotorCommand {
    pub fn from_vision_output(vision_output: VisionAlgorithmOutput) -> Self {
        let cmd = match vision_output {
            VisionAlgorithmOutput::Up => {
                MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_reverse())
            }
            _ => MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_forward()),
        };

        cmd
    }
}

impl Into<l9110::Direction> for MotorDirection {
    fn into(self) -> l9110::Direction {
        match self {
            Self::Forward => l9110::Direction::Forward,
            Self::Reverse => l9110::Direction::Reverse,
        }
    }
}

impl Into<simple_stepper_driver::Direction> for MotorDirection {
    fn into(self) -> simple_stepper_driver::Direction {
        match self {
            Self::Forward => simple_stepper_driver::Direction::Forward,
            Self::Reverse => simple_stepper_driver::Direction::Reverse,
        }
    }
}

impl Into<rmt_stepper_driver::Direction> for MotorDirection {
    fn into(self) -> rmt_stepper_driver::Direction {
        match self {
            Self::Forward => rmt_stepper_driver::Direction::Forward,
            Self::Reverse => rmt_stepper_driver::Direction::Reverse,
        }
    }
}
