use embassy_time::Duration;
use messenger_mouse::motor::{MotorDirection, MotorVelocitySetpoint, StepperPositionSetpoint};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::actuation::stepper::motor_task::{HOMING_DIRECTION, HOMING_SPEED_MM_PS};

pub mod low_level_task;
pub mod state_machine;

/// How often should the stepper be serviced?
const STEP_INTERVAL_HZ: u64 = 100;
const STEP_INTERVAL: Duration = Duration::from_hz(STEP_INTERVAL_HZ); // 10ms per interval
const APPROACHING_STEP_INTERVAL: Duration = Duration::from_millis(10); // 10ms per interval
const ACCEL_SPS_PER_SECOND: u64 = 60000;
const ACCEL_SPS_PER_INTERVAL: u32 = (ACCEL_SPS_PER_SECOND / STEP_INTERVAL_HZ) as u32;

#[derive(Clone, Debug)]
pub struct IntervalConfig {
    /// How many steps in this interval?
    steps: u32,
    /// How many us per step?
    micros_per_step: u64,
}

impl Default for IntervalConfig {
    fn default() -> Self {
        Self {
            steps: Default::default(),
            micros_per_step: 10_000, // Defaults to 100hz STEP pulses
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
/// Actions the low level stepper can perform
pub enum StepperAction {
    #[default]
    Coast,
    Hold,
    SingleStep,
    MoveVelocity(MotorVelocitySetpoint),
    MovePosition(StepperPositionSetpoint),
}

impl StepperAction {
    pub fn new_stopped() -> Self {
        Self::Coast
    }

    pub fn new_homing() -> Self {
        let home_vel = match HOMING_DIRECTION {
            MotorDirection::Forward => HOMING_SPEED_MM_PS,
            MotorDirection::Reverse => -HOMING_SPEED_MM_PS,
        };

        Self::MoveVelocity(MotorVelocitySetpoint {
            dir: HOMING_DIRECTION,
            speed: Velocity::new::<millimeter_per_second>(home_vel),
        })
    }

    fn new_seek(dir: MotorDirection) -> Self {
        let seek_vel = match dir {
            MotorDirection::Forward => HOMING_SPEED_MM_PS,
            MotorDirection::Reverse => -HOMING_SPEED_MM_PS,
        };

        Self::MoveVelocity(MotorVelocitySetpoint {
            dir,
            speed: Velocity::new::<millimeter_per_second>(seek_vel),
        })
    }

    // Seek towards the cable
    pub fn new_seek_forward() -> Self {
        let dir = HOMING_DIRECTION.get_opposite();
        Self::new_seek(dir)
    }

    // Seek away from the cable
    pub fn new_seek_reverse() -> Self {
        let dir = HOMING_DIRECTION;
        Self::new_seek(dir)
    }
}
