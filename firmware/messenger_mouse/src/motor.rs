use serde::{Deserialize, Serialize};
use uom::si::{
    f32::{Length, Velocity},
    length::millimeter,
    velocity::millimeter_per_second,
};

pub const POSITION_EPSILON_MM: f32 = 0.03;
pub const VELOCITY_EPSILON_MM_PS: f32 = 0.03;

pub const POSITION_MODE_VELOCITY_MM_PS: f32 = 1.0;

#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// A motor setpoint
pub enum MotorAction {
    #[default]
    Coast,
    Hold,
    Home,
    MoveVelocity(MotorVelocitySetpoint),
    MovePosition(MotorPositionSetpoint),
}

impl MotorAction {
    pub fn next(&self) -> Self {
        match self {
            MotorAction::Coast => MotorAction::Hold,
            MotorAction::Hold => MotorAction::Home,
            MotorAction::Home => MotorAction::MoveVelocity(MotorVelocitySetpoint::new_safe()),
            MotorAction::MoveVelocity(_) => {
                MotorAction::MovePosition(MotorPositionSetpoint::new_safe())
            }
            MotorAction::MovePosition(_) => MotorAction::Coast,
        }
    }

    pub fn new_velocity(dir: MotorDirection, speed: Velocity) -> Self {
        Self::MoveVelocity(MotorVelocitySetpoint { dir, speed })
    }
}

/// State of all the motors on the cable peeler
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct MotorState {
    pub translation: Motor,
    pub rotation: Motor,
    pub knife: Motor,
}

impl MotorState {
    pub fn set_manager(&mut self, manager: MotorManager) {
        self.translation.manager = manager.clone();
        self.rotation.manager = manager.clone();
        self.knife.manager = manager;
    }

    pub fn flip_management(&mut self) {
        self.translation.manager.flip();
        self.rotation.manager.flip();
        self.knife.manager.flip();
    }
}

/// A single cable peeler motor
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct Motor {
    pub setpoint: MotorAction,
    pub manager: MotorManager,
}

#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Conveys responsibility for controlling a motor
pub enum MotorManager {
    #[default]
    Manual,
    Vision,
}

impl MotorManager {
    fn flip(&mut self) {
        *self = match self {
            MotorManager::Manual => MotorManager::Vision,
            MotorManager::Vision => MotorManager::Manual,
        }
    }
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
    /// NOTE: Sign determines direction for velocity movements
    pub speed: Velocity,
}

/// Position movement setpoint
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
pub struct StepperPositionSetpoint {
    /// Position target wrt home position
    pub target: Steps,
    /// Speed of the motor
    /// NOTE: this is considered absolute for position movements
    pub speed: Velocity,
}

#[derive(Copy, PartialEq, PartialOrd, Deserialize, Serialize, Clone, Debug, Default)]
pub struct Steps(pub i32);

impl Steps {
    const TARGET_REACHED_EPSILON: Steps = Steps(10);
    const TARGET_CLOSE_EPSILON: Steps = Steps(500);

    pub fn is_close_to(&self, other: &Steps) -> bool {
        self.is_within_range_of(other, &Self::TARGET_CLOSE_EPSILON)
    }
    pub fn target_is_reached(&self, other: &Steps) -> bool {
        self.is_within_range_of(other, &Self::TARGET_REACHED_EPSILON)
    }
    pub fn is_within_range_of(&self, other: &Steps, epsilon: &Steps) -> bool {
        self.0.abs_diff(other.0) <= epsilon.0.abs() as u32
    }
}

/// Custom PartialEq to avoid f32 rounding issues
impl PartialEq for StepperPositionSetpoint {
    fn eq(&self, other: &Self) -> bool {
        self.target.target_is_reached(&other.target)
            && f32_approx_eq(
                self.speed.get::<millimeter_per_second>(),
                other.speed.get::<millimeter_per_second>(),
                VELOCITY_EPSILON_MM_PS,
            )
    }
}

/// Custom PartialEq to avoid f32 rounding issues
impl PartialEq for MotorPositionSetpoint {
    fn eq(&self, other: &Self) -> bool {
        f32_approx_eq(
            self.target.get::<millimeter>(),
            other.target.get::<millimeter>(),
            POSITION_EPSILON_MM,
        ) && f32_approx_eq(
            self.speed.get::<millimeter_per_second>(),
            other.speed.get::<millimeter_per_second>(),
            VELOCITY_EPSILON_MM_PS,
        )
    }
}

/// Custom PartialEq to avoid f32 rounding issues
impl PartialEq for MotorVelocitySetpoint {
    fn eq(&self, other: &Self) -> bool {
        self.dir == other.dir
            && f32_approx_eq(
                self.speed.get::<millimeter_per_second>(),
                other.speed.get::<millimeter_per_second>(),
                VELOCITY_EPSILON_MM_PS,
            )
    }
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

#[derive(Deserialize, Serialize, Clone, Copy, Debug, Default, PartialEq)]
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

pub fn f32_approx_eq(lhs: f32, rhs: f32, eps: f32) -> bool {
    (lhs - rhs).abs() < eps
}
