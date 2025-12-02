#![no_std]

use defmt::trace;
use embassy_stm32::{
    time::Hertz,
    timer::{GeneralInstance4Channel, simple_pwm::SimplePwm},
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use thiserror::Error;

const BASE_DUTY_CYCLE_PERCENT: u8 = 50;
pub const BASE_STEP_FREQUENCY_HZ: Hertz = Hertz(10_000);
pub const MIN_STEP_FREQUENCY_HZ: Hertz = Hertz(1);
pub const MAX_STEP_FREQUENCY_HZ: Hertz = Hertz(25_000); // 100khz max theoretical (~5µs pulse width at 50% dc), but this cooks the Tb6600 already

pub struct Tb6600<Timer, DirPin, Delay>
where
    Timer: GeneralInstance4Channel, // General-purpose 16-bit timer with 4 channels instance
{
    name: &'static str,
    step_pwm: SimplePwm<'static, Timer>,
    step_frequency: Hertz,
    dir_pin: DirPin,
    enabled: bool,
    delay: Delay,
    /// Fierce googling: min pulse width ~5µs, maybe?
    pub direction: Direction,
    pub pulse_period_us: u32,
}

#[derive(Error, Debug, defmt::Format)]
pub enum TB6600Error {
    #[error("Direction pin ging op zn gat")]
    DirectionPin,
    #[error("Step pin ging op zn gat")]
    StepPin,
    #[error("Enable pin ging op zn gat")]
    EnablePin,
}

#[derive(Clone, Debug, defmt::Format)]
pub enum Direction {
    Forward,
    Reverse,
}

impl<Timer, DirPin, Delay> Tb6600<Timer, DirPin, Delay>
where
    DirPin: OutputPin,              // Any output pin
    Delay: DelayNs,                 // Nanosecond capable delay
    Timer: GeneralInstance4Channel, // General-purpose 16-bit timer with 4 channels instance to back the pwm impl
{
    pub fn new(
        name: &'static str,
        mut step_pwm: SimplePwm<'static, Timer>,
        dir_pin: DirPin,
        delay: Delay,
        pulse_period_us: u32,
    ) -> Self {
        step_pwm.ch1().disable(); // start disabled

        let mut out = Self {
            name,
            step_pwm,
            step_frequency: BASE_STEP_FREQUENCY_HZ,
            dir_pin,
            delay,
            direction: Direction::Forward,
            pulse_period_us,
            enabled: false,
        };

        out.set_speed(BASE_STEP_FREQUENCY_HZ.0);

        out
    }

    /// ENABLE LOW = ON + EN tied low -> Driver is always on
    /// Software enables the step pwm output
    pub fn start_stepping(&mut self) {
        self.control_stepping(true);
    }

    /// ENABLE LOW = ON + EN tied low -> Driver is always on
    /// Software disables the step pwm output
    pub fn stop_stepping(&mut self) {
        self.control_stepping(false);
    }

    /// ENABLE LOW = ON + EN tied low -> Driver is always on
    /// Software controls the step pwm output
    pub fn control_stepping(&mut self, should_step: bool) {
        if should_step {
            trace!(
                "{} started stepping at {}hz",
                self.name, self.step_frequency
            );
            self.step_pwm.ch1().enable();
        } else {
            trace!("{} stopped stepping", self.name);
            self.step_pwm.ch1().disable();
        }
        self.enabled = should_step;
    }

    /// Set stepper speed
    pub fn set_speed(&mut self, frequency: u32) {
        let frequency = frequency.clamp(MIN_STEP_FREQUENCY_HZ.0, MAX_STEP_FREQUENCY_HZ.0);
        trace!("{} set speed to {}Hz", self.name, frequency);
        self.step_pwm.set_frequency(Hertz(frequency));
        self.step_pwm
            .ch1()
            .set_duty_cycle_percent(BASE_DUTY_CYCLE_PERCENT); // set_frequency docs suggests I have to call this again
        self.step_frequency = Hertz(frequency);
    }

    /// Set step direction
    pub async fn set_direction(&mut self, direction: Direction) -> Result<(), TB6600Error> {
        self.direction = direction;

        match self.direction {
            Direction::Forward => self
                .dir_pin
                .set_high()
                .map_err(|_| TB6600Error::DirectionPin)?,
            Direction::Reverse => self
                .dir_pin
                .set_low()
                .map_err(|_| TB6600Error::DirectionPin)?,
        };

        // Chill out, catch some waves
        self.delay.delay_us(5).await;

        Ok(())
    }

    /// Flip step direction
    pub async fn flip_direction(&mut self) -> Result<(), TB6600Error> {
        let dir = match self.direction {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        };

        self.set_direction(dir).await?;

        Ok(())
    }

    /// Set step pwm dc
    /// Percentage [0-100]
    pub fn set_duty_cycle_percent(&mut self, percentage: u8) {
        trace!("{} set dc to {}%", self.name, percentage);
        self.step_pwm.ch1().set_duty_cycle_percent(percentage);
    }
}
