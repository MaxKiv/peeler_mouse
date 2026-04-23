use crate::actuation::stepper::{
    low_level::{
        low_level_task::position_to_steps, IntervalConfig, StepperAction, ACCEL_SPS_PER_INTERVAL,
        ACCEL_SPS_PER_SECOND, APPROACHING_STEP_INTERVAL, STEP_INTERVAL,
    },
    motor_task::{MAXIMUM_SPS, MINIMUM_SPS, MINIMUM_TRANSITION_SPS},
};
use embassy_time::Duration;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
#[cfg(feature = "pcb")]
use esp_idf_hal::gpio::*;
use log::*;
use messenger_mouse::motor::{MotorDirection, Steps};
use rmt_stepper_driver::RmtStepper;
use uom::{
    si::{f32::Velocity, length::millimeter, velocity::millimeter_per_second},
    ConstZero,
};

/// Giant state machine that manages low level implementation details for controlling a stepper
/// motor using the ESP-IDF RMT TX peripheral
/// I'd be the first to admit this is full of blatant global state,
/// but sometimes one sacrifices clarity for brevity
pub struct StepperStateMachine<DirPin, Delay> {
    pub state: StepperState,
    driver: RmtStepper<DirPin, Delay>,
    #[cfg(feature = "devkit")]
    enable: PinDriver<'static, Gpio40, Output>,

    #[cfg(feature = "pcb")]
    enable: PinDriver<'static, Gpio4, Output>,

    pub target_position: Steps,
    pub current_position: Steps,
    pub vel_state: VelocityState,
    pub interval_cfg: Option<IntervalConfig>,
}

impl<DirPin, Delay> StepperStateMachine<DirPin, Delay>
where
    DirPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(
        driver: RmtStepper<DirPin, Delay>,
        #[cfg(feature = "devkit")] enable: PinDriver<'static, Gpio40, Output>,
        #[cfg(feature = "pcb")] enable: PinDriver<'static, Gpio4, Output>,
    ) -> Self {
        Self {
            enable,
            driver,
            state: StepperState::Coast,
            current_position: Steps(0),
            target_position: Steps(0),
            vel_state: VelocityState::new(),
            interval_cfg: None,
        }
    }

    /// Transition to requested state
    pub async fn transition_to(&mut self, cmd: StepperAction) {
        info!("SM: transition to {:?}", cmd);

        match (&cmd, &mut self.state) {
            // Transition:
            // (Target cmd, Current state)
            (StepperAction::Coast, _) => {
                // We must hold -> Disable driver
                let _ = self.enable.set_high();

                let _ = self.driver.stop_stepping();
                self.reset();
            }
            // (Target cmd, Current state)
            (StepperAction::Hold, _) => {
                // We must hold -> Enable driver
                let _ = self.enable.set_low();

                let _ = self.driver.stop_stepping();

                self.reset();
            }
            // (Target cmd, Current state)
            (StepperAction::SingleStep, _) => {
                // We must step -> enable driver
                let _ = self.enable.set_low();

                let _ = self.driver.stop_stepping();

                // Step once
                let _ = self.driver.step_once().await;
                // track position
                self.current_position.0 += 1;

                let _ = self.driver.stop_stepping();
            }
            // (Target cmd, Current state)
            // Position / Velocity command
            (StepperAction::MoveVelocity(sp), _) => {
                // enable driver
                let _ = self.enable.set_low();

                warn!("MOTOR LOW LVL: New velocity setpoint: {:?}", sp,);

                self.vel_state.update_target_velocity(sp.speed);
            }
            (StepperAction::MovePosition(sp), _) => {
                // enable driver
                let _ = self.enable.set_low();

                // Figure out direction to move in
                let new_speed = if sp.target <= self.current_position {
                    -(sp.speed).abs()
                } else {
                    (sp.speed).abs()
                };

                warn!(
                    "MOTOR LOW LVL: New position setpoint: {:?} -> new speed {}mm/s",
                    sp,
                    new_speed.get::<millimeter_per_second>()
                );

                // update target velocity and position
                self.vel_state.update_target_velocity(new_speed);
                self.update_target_position(sp.target);
            }
        }

        // Update current state
        self.state = StepperState::from_cmd(cmd);
    }

    /// Are we ready to switch direction?
    /// Determined by checking if we go slow enough to transition
    pub fn ready_for_direction_switch(&mut self) -> bool {
        // Do we have to switch direction?
        self.vel_state.current_dir != self.vel_state.target_dir &&

        // Are we going slow enough?
        self.vel_state.current_speed <= MINIMUM_TRANSITION_SPS
    }

    /// Switch direction
    pub async fn switch_direction(&mut self) {
        // Switch direction
        if let Ok(()) = self.driver.set_direction(self.vel_state.target_dir).await {
            self.vel_state.current_dir = self.vel_state.target_dir;
        }
        // Start acceleration again
        self.vel_state.ramp_state = RampState::Accelerating;
    }

    /// Update velocity based on ramp state
    pub fn update_velocity(&mut self) {
        // Are we ramping?
        if self.vel_state.ramp_state != RampState::Cruising {
            // Are we within a single ramp window of target speed?
            if self.vel_state.current_dir == self.vel_state.target_dir
                && self.vel_state.target_speed_is_within_single_ramp_interval()
            {
                // Set speed to target speed directly
                self.vel_state.current_speed = self.vel_state.target_speed;
                self.vel_state.ramp_state = RampState::Cruising;
            } else {
                // Continue ramping

                match self.vel_state.ramp_state {
                    RampState::Accelerating => {
                        self.vel_state.current_speed.0 = self
                            .vel_state
                            .current_speed
                            .0
                            .saturating_add(ACCEL_SPS_PER_INTERVAL)
                            .clamp(MINIMUM_SPS.0, MAXIMUM_SPS.0);
                    }
                    RampState::Decelerating => {
                        self.vel_state.current_speed.0 = self
                            .vel_state
                            .current_speed
                            .0
                            .saturating_sub(ACCEL_SPS_PER_INTERVAL)
                            .clamp(MINIMUM_SPS.0, MAXIMUM_SPS.0);
                    }
                    _ => {}
                };
            }
        }

        // Update interval cfg to reflect new velocity changes
        self.set_interval_config();

        debug!("{:?} - {:?}", self.vel_state, self.interval_cfg);
    }

    pub fn calculate_steps_in_interval(&self, interval_duration: Duration) -> u32 {
        let interval_ticks = interval_duration.as_ticks();
        let ticks_per_step = Duration::from_hz(self.vel_state.current_speed.0 as u64).as_ticks();

        (interval_ticks / ticks_per_step) as u32
    }

    /// Get number of steps and microseconds per step for this interval based on current speed
    /// Sets the RMT TX driver STEP pulse period in RMT ticks/microseconds
    fn set_interval_config(&mut self) {
        if matches!(self.state, StepperState::Velocity | StepperState::Position) {
            // Are we in position mode?
            let interval_ticks = if let StepperState::Position = self.state
                && self.target_is_close()
            {
                debug!("within APPROACHING distance of target, reducing RMT interval");
                APPROACHING_STEP_INTERVAL.as_ticks()
            } else {
                STEP_INTERVAL.as_ticks()
            };

            let current_speed = self
                .vel_state
                .current_speed
                .0
                .clamp(MINIMUM_SPS.0, MAXIMUM_SPS.0);
            let micros_per_step = Duration::from_hz(current_speed as u64).as_ticks();

            self.interval_cfg = Some(IntervalConfig {
                micros_per_step,
                steps: (interval_ticks / micros_per_step) as u32,
            });

            let step_pulse_period_micros: u16 = if let Some(cfg) = &self.interval_cfg {
                cfg.micros_per_step.try_into().unwrap_or(10_000u16) // Defaults to 100hz
            } else {
                10_000u16
            };

            self.driver
                .set_step_pulse_period_micros(step_pulse_period_micros);

            // info!(
            //     "MOTOR LOW LVL: set_interval_config -> {:?}, {}us/step, {}ticks interval",
            //     self.interval_cfg, micros_per_step, interval_ticks
            // );
        }
    }

    /// Re-arm RMT step pulses, increment position counter
    pub async fn on_step_timer_expire(&mut self) {
        if let Some(cfg) = &self.interval_cfg {
            debug!(
                "on_step_timer_expire entry: {:?} - {:?}",
                cfg, self.vel_state
            );

            // Position or Velocity mode -> Re arm RMT step pulses
            let steps = cfg.steps;

            // info!("SM on_step_timer_expire arming {} steps", steps);
            if let Err(err) = self.driver.arm_n_steps(steps).await {
                error!("ARM N STEPS ERROR {:?}", err);
                return;
            }

            // Assume we stepped N times; Update position
            self.current_position.0 += match self.vel_state.current_dir {
                MotorDirection::Forward => steps as i32,
                MotorDirection::Reverse => -(steps as i32),
            };

            // Position mode specific
            if self.state == StepperState::Position {
                // Are we approaching our target?
                // AKA Is it time to start decelerating?
                let distance_remaining = self.current_position.0.abs_diff(self.target_position.0);

                if self.get_braking_distance() >= distance_remaining {
                    info!(
                        "{:?} -> {:?} = withing braking distance: {} => Velocity::ZERO",
                        self.current_position,
                        self.target_position,
                        self.get_braking_distance()
                    );

                    // Start ramping down velocity to avoid overshoot
                    self.vel_state.update_target_velocity(Velocity::ZERO);
                }
            }

            debug!("on_step_timer_expire EXIT: {:?}", cfg);
        }
    }

    /// Calculate how many steps we would take if we started ramping down right now
    fn get_braking_distance(&self) -> u32 {
        // How many ramp down intervals from current speed -> zero speed?
        // let ramp_down_intervals = self.vel_state.current_speed.0 / ACCEL_SPS_PER_INTERVAL;

        // Approximate braking distance = (dx/dt)^2 / (2 * dx/dt^2)
        let braking_distance = (self.vel_state.current_speed.0 * self.vel_state.current_speed.0)
            / (2 * ACCEL_SPS_PER_SECOND as u32);

        debug!("Braking distance: {}", braking_distance);

        braking_distance
    }

    /// Reset velocity and interval_cfg
    fn reset(&mut self) {
        self.vel_state.reset();
        self.interval_cfg = None;
    }

    pub fn update_target_position(&mut self, target: Steps) {
        self.target_position = target;
    }

    pub fn target_is_reached(&self) -> bool {
        self.target_position
            .target_is_reached(&self.current_position)
    }

    pub fn target_is_close(&self) -> bool {
        self.target_position.is_close_to(&self.current_position)
    }
}

#[derive(PartialEq)]
pub enum StepperState {
    Coast,
    Hold,
    SingleStep,
    Velocity,
    Position,
}

impl StepperState {
    pub fn from_cmd(cmd: StepperAction) -> Self {
        match cmd {
            StepperAction::Coast => Self::Coast,
            StepperAction::Hold => Self::Hold,
            StepperAction::SingleStep => Self::SingleStep,
            StepperAction::MoveVelocity(_) => Self::Velocity,
            StepperAction::MovePosition(_) => Self::Position,
        }
    }
}

/// Steps Per Second (Hertz)
#[derive(PartialEq, PartialOrd, Debug, Clone, Copy)]
pub struct SPS(pub u32);

impl SPS {
    pub const ZERO: SPS = SPS(0);

    pub fn from_velocity(vel: Velocity) -> SPS {
        use messenger_mouse::encoder::*;
        let mm_ps = vel.get::<millimeter_per_second>().abs();

        let sps = (mm_ps / KNIFE_AXIS_LEAD_MM)
            * KNIFE_AXIS_GEAR_RATIO
            * KNIFE_AXIS_MICROSTEPS_PER_STEP
            * KNIFE_AXIS_STEPS_PER_ROTATION;
        let sps = sps as u32;

        info!("velocity_to_interval: target {}mm/s -> {}sps", mm_ps, sps);

        if sps < MINIMUM_SPS.0 {
            SPS::ZERO
        } else if sps > MAXIMUM_SPS.0 {
            MAXIMUM_SPS
        } else {
            SPS(sps)
        }
    }
}

#[derive(Debug)]
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

    pub fn reset(&mut self) {
        *self = Self {
            current_speed: SPS::ZERO,
            target_speed: SPS::ZERO,
            ramp_state: RampState::Cruising,
            current_dir: self.current_dir,
            target_dir: self.target_dir,
        }
    }

    pub fn update_target_velocity(&mut self, new_speed: Velocity) {
        // Update direction
        if new_speed == Velocity::ZERO {
            self.target_dir = self.current_dir
        } else if new_speed > Velocity::ZERO {
            self.target_dir = MotorDirection::Forward
        } else {
            self.target_dir = MotorDirection::Reverse
        }

        // Update speed
        let new_target = SPS::from_velocity(new_speed);

        // Update ramp state
        // Are we switching directions?
        self.ramp_state = if self.current_dir != self.target_dir {
            // Ramp down to reach direction transition speed
            RampState::Decelerating
        } else {
            // Are we traveling at requested speed?
            if self.current_speed == new_target {
                // Cruise along
                RampState::Cruising
            // Are we required to go faster?
            } else if self.current_speed < new_target {
                RampState::Accelerating
            } else {
                // We should go slower
                RampState::Decelerating
            }
        };

        debug!(
            "SM update_target_velocity - current {:?}-{}sps - target {:?}-{}sps -> ramping {:?}",
            self.current_dir, self.current_speed.0, self.target_dir, new_target.0, self.ramp_state
        );

        self.target_speed = new_target;
    }

    fn target_speed_is_within_single_ramp_interval(&self) -> bool {
        match self.ramp_state {
            RampState::Cruising => true,
            RampState::Accelerating => {
                (self.current_speed.0.saturating_add(ACCEL_SPS_PER_INTERVAL)) >= self.target_speed.0
            }
            RampState::Decelerating => {
                (self.current_speed.0.saturating_sub(ACCEL_SPS_PER_INTERVAL)) <= self.target_speed.0
            }
        }
    }
}

#[derive(PartialEq, Clone, Copy, Debug)]
pub enum RampState {
    Cruising,
    Accelerating,
    Decelerating,
}
