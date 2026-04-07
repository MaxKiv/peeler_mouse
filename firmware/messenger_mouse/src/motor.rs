use serde::{Deserialize, Serialize};
use uom::si::{
    f32::{Length, Velocity},
    length::millimeter,
    velocity::millimeter_per_second,
};

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum MotorCommand {
    #[default]
    Halt,
    Home,
    MoveVelocity(MotorVelocitySetpoint),
    MovePosition(MotorPositionSetpoint),
}

impl MotorCommand {
    pub fn next(&self) -> Self {
        match self {
            MotorCommand::Halt => MotorCommand::Home,
            MotorCommand::Home => MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe()),
            MotorCommand::MoveVelocity(_) => {
                MotorCommand::MovePosition(MotorPositionSetpoint::new_safe())
            }
            MotorCommand::MovePosition(_) => MotorCommand::Halt,
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum KnifeManager {
    #[default]
    Manual,
    Vision,
}

/// Velocity movement setpoint
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
pub struct MotorVelocitySetpoint {
    /// Direction of axis rotation
    pub dir: MotorDirection,
    /// Speed of the motor
    pub speed: Velocity,
}

/// Position movement setpoint
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
pub struct MotorPositionSetpoint {
    /// Position target wrt home position
    pub target: Length,
    /// Speed of the motor
    pub speed: Velocity,
}

impl MotorPositionSetpoint {
    pub fn new_safe() -> Self {
        Self {
            target: Length::new::<millimeter>(0.0),
            speed: Velocity::new::<millimeter_per_second>(0.0),
        }
    }
}

impl MotorVelocitySetpoint {
    pub fn new_safe() -> Self {
        Self {
            dir: MotorDirection::Forward,
            speed: Velocity::new::<millimeter_per_second>(0.0),
        }
    }

    pub fn new(dir: MotorDirection, speed: Velocity) -> Self {
        Self { dir, speed }
    }

    pub fn new_forward(speed: Velocity) -> Self {
        Self::new(MotorDirection::Forward, speed)
    }

    pub fn new_reverse(speed: Velocity) -> Self {
        Self::new(MotorDirection::Reverse, speed)
    }
}

#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum MotorDirection {
    #[default]
    Forward,
    Reverse,
}

impl MotorDirection {
    pub fn flip(&mut self) {
        use MotorDirection::*;
        *self = match self {
            Forward => Reverse,
            Reverse => Forward,
        };
    }

    pub fn get_opposite(&self) -> Self {
        use MotorDirection::*;
        match self {
            Forward => Reverse,
            Reverse => Forward,
        }
    }
}

// ---- Format impls ----
impl defmt::Format for MotorVelocitySetpoint {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "VelocitySetpoint({} - {}mm/s",
            match self.dir {
                MotorDirection::Forward => "Forward",
                MotorDirection::Reverse => "Reverse",
            },
            self.speed.get::<millimeter_per_second>(),
        );
    }
}

impl defmt::Format for MotorPositionSetpoint {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "PositionSetpoint({}mm - {}mm/s",
            self.target.get::<millimeter>(),
            self.speed.get::<millimeter_per_second>(),
        );
    }
}
