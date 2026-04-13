use embassy_time::Duration;

pub mod low_level_task;
pub mod state_machine;

/// How often should the stepper be serviced?
const STEP_INTERVAL: Duration = Duration::from_hz(10); // 100ms per interval

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
            micros_per_step: u64::MAX,
        }
    }
}
