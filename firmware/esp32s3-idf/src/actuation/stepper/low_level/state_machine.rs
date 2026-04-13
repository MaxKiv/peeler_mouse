use crate::actuation::stepper::{
    low_level::{IntervalConfig, STEP_INTERVAL},
    motor_task::{StepperCommand, MAXIMUM_SPS, MINIMUM_SPS, MINIMUM_TRANSITION_SPS},
    Steps,
};
use embassy_time::Duration;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use esp_idf_hal::gpio::{Gpio40, Output, PinDriver};
use log::*;
use messenger_mouse::motor::MotorDirection;
use rmt_stepper_driver::RmtStepper;
use uom::{
    si::{f32::Velocity, velocity::millimeter_per_second},
    ConstZero,
};

pub struct StepperStateMachine<DirPin, Delay> {
    pub state: StepperState,
    driver: RmtStepper<DirPin, Delay>,
    enable: PinDriver<'static, Gpio40, Output>,
    position: Steps,
    pub vel_state: VelocityState,
    interval_cfg: IntervalConfig,
}

impl<DirPin, Delay> StepperStateMachine<DirPin, Delay>
where
    DirPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(
        driver: RmtStepper<DirPin, Delay>,
        enable: PinDriver<'static, Gpio40, Output>,
    ) -> Self {
        Self {
            enable,
            driver,
            state: StepperState::Coast,
            position: Steps(0),
            vel_state: VelocityState::new(),
            interval_cfg: IntervalConfig::default(),
        }
    }

    /// Transition to requested state
    pub async fn transition_to(&mut self, cmd: StepperCommand) {
        let target = if let StepperCommand::Velocity(target) = cmd {
            target
        } else {
            Velocity::ZERO
        };

        match (&cmd, &mut self.state) {
            // Transition:
            // (Target cmd, Current state)
            (StepperCommand::Coast, _) => {
                self.enable.set_high();
            }
            // (Target cmd, Current state)
            (StepperCommand::Holding, _) => {
                self.enable.set_low();
            }
            // (Target cmd, Current state)
            (StepperCommand::SingleStep, _) => {
                // enable driver
                self.enable.set_low();

                // Step once
                let _ = self.driver.step_once().await;
                // track pposition
                self.position.0 += 1;
            }
            // (Target cmd, Current state)
            (_, _) => {}
        }
        self.vel_state.update_target(target);
        self.state = StepperState::from_cmd(cmd);
    }

    pub fn ready_for_direction_switch(&mut self) -> bool {
        // Do we have to switch direction?
        self.vel_state.current_dir != self.vel_state.target_dir &&

        // Are we going slow enough?
        self.vel_state.current_speed.abs() <= MINIMUM_TRANSITION_SPS.0
    }

    // Switch direction
    pub async fn switch_direction(&mut self) {
        if let Ok(()) = self.driver.set_direction(self.vel_state.target_dir).await {
            self.vel_state.current_dir = self.vel_state.target_dir;
        }
    }

    pub fn update_velocity(&mut self) -> SPS {
        const ACCEL_SPS_PER_INTERVAL: u32 = 10;

        match self.vel_state.ramp_state {
            RampState::Accelerating => {
                self.vel_state
                    .current_speed
                    .0
                    .saturating_add(ACCEL_SPS_PER_INTERVAL)
                    .max(MAXIMUM_SPS.0);
            }
            RampState::Decelerating => {
                self.vel_state
                    .current_speed
                    .0
                    .saturating_sub(ACCEL_SPS_PER_INTERVAL)
                    .min(MINIMUM_SPS.0);
            }
            _ => {}
        };

        self.vel_state.current_speed
    }

    pub fn calculate_steps_in_interval(&self, interval_duration: Duration) -> u32 {
        let interval_ticks = interval_duration.as_ticks();
        let ticks_per_step = Duration::from_hz(self.vel_state.current_speed.0 as u64).as_ticks();

        (interval_ticks / ticks_per_step) as u32
    }

    /// Get number of steps and microseconds per step for this interval based on current speed
    pub fn set_interval_config(&self) {
        let interval_ticks = STEP_INTERVAL.as_ticks();
        let micros_per_step = Duration::from_hz(self.vel_state.current_speed.0 as u64).as_micros();

        self.interval_cfg = IntervalConfig {
            micros_per_step,
            steps: (interval_ticks / micros_per_step) as u32,
        };

        self.update_rmt_pulse_period();
    }

    /// Set driver RMT step pulse period for this interval
    fn update_rmt_pulse_period(&mut self) {
        let step_pulse_period_micros: u16 = self
            .interval_cfg
            .micros_per_step
            .try_into()
            .unwrap_or(u16::MAX);

        self.driver
            .set_step_pulse_period_micros(step_pulse_period_micros);
    }

    /// Re-arm RMT step pulses, increment position counter
    pub async fn on_step_timer_expire(&mut self) {
        // Re-arm RMT step pulses
        let steps = self.interval_cfg.steps;
        let _ = self.driver.arm_n_steps(steps).await;

        // Assume we stepped N times; Update position
        self.position.0 += match self.vel_state.current_dir {
            MotorDirection::Forward => steps as i32,
            MotorDirection::Reverse => -(steps as i32),
        };
    }
}

pub enum StepperState {
    Coast,
    Holding,
    SingleStep,
    Velocity,
}

impl StepperState {
    pub fn from_cmd(cmd: StepperCommand) -> Self {
        match cmd {
            StepperCommand::Coast => Self::Coast,
            StepperCommand::Holding => Self::Holding,
            StepperCommand::SingleStep => Self::SingleStep,
            StepperCommand::Velocity(_) => Self::Velocity,
        }
    }
}

/// Steps Per Second (Hertz)
#[derive(PartialEq, Debug, Clone, Copy)]
pub struct SPS(pub u32);

impl SPS {
    pub const ZERO: SPS = SPS(0);

    pub fn from_velocity(vel: Velocity) -> SPS {
        use messenger_mouse::encoder::*;
        let mm_ps = vel.get::<millimeter_per_second>();

        let sps = (mm_ps / KNIFE_AXIS_LEAD_MM)
            * KNIFE_AXIS_GEAR_RATIO
            * KNIFE_AXIS_MICROSTEPS_PER_STEP
            * KNIFE_AXIS_STEPS_PER_ROTATION;
        let sps = sps.abs() as u32;

        info!("velocity_to_interval: target {}mm/s -> {}sps", mm_ps, sps);

        if sps < MINIMUM_SPS.0 {
            SPS::ZERO
        } else {
            SPS(sps)
        }
    }

    pub fn abs(&self) -> u32 {
        self.0.abs()
    }
}

pub struct VelocityState {
    pub current_dir: MotorDirection,
    pub target_dir: MotorDirection,
    pub current_speed: SPS,
    pub target_speed: SPS,
    pub ramp_state: RampState,
}

impl VelocityState {
    pub fn new() -> Self {
        Self {
            current_speed: SPS::ZERO,
            target_speed: SPS::ZERO,
            ramp_state: RampState::Cruising,
            current_dir: MotorDirection::Forward,
            target_dir: MotorDirection::Forward,
        }
    }

    pub fn update_target(&mut self, new_target: Velocity) {
        // Update direction
        if new_target == Velocity::ZERO {
            self.target_dir = self.current_dir
        } else if new_target > Velocity::ZERO {
            self.target_dir = MotorDirection::Forward
        } else {
            self.target_dir = MotorDirection::Reverse
        }

        // Update speed
        let new_target = SPS::from_velocity(new_target);

        self.ramp_state = if self.current_speed == new_target {
            RampState::Cruising
        } else if self.current_speed.abs() < new_target.abs() {
            RampState::Accelerating
        } else {
            RampState::Decelerating
        };

        self.target_speed = new_target;
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum RampState {
    Cruising,
    Accelerating,
    Decelerating,
}
