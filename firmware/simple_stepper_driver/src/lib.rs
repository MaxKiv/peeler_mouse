#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use esp_idf_hal::{ledc::LedcDriver, units::Hertz};
use thiserror::Error;

pub const BASE_STEP_FREQUENCY_HZ: Hertz = Hertz(1_000);

use log::*;

/// Simple single speed DIR/STEP driver
pub struct SimpleStepperDriver<DirPin, Delay> {
    name: &'static str,
    step_pwm: LedcDriver<'static>,
    dir_pin: DirPin,
    enabled: bool,
    delay: Delay,
    pub direction: Direction,
}

#[derive(Error, Debug)]
pub enum SimpleStepperError {
    #[error("Unable to drive direction pin")]
    DirectionPin,
    #[error("Unable to drive enable pin")]
    EnablePin,
    #[error("Issue with PWM driver")]
    Pwm,
}

#[derive(Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
}

impl<DirPin, Delay> SimpleStepperDriver<DirPin, Delay>
where
    DirPin: OutputPin, // Any output pin
    Delay: DelayNs,    // Nanosecond capable delay
{
    pub fn try_new(
        name: &'static str,
        mut timer: esp_idf_hal::ledc::LedcTimerDriver<'static, esp_idf_hal::ledc::TIMER1>,
        channel: esp_idf_hal::ledc::CHANNEL1,
        pwm_pin: esp_idf_hal::gpio::Gpio41,
        dir_pin: DirPin,
        delay: Delay,
    ) -> anyhow::Result<Self> {
        // Set stepping frequency
        timer.set_frequency(BASE_STEP_FREQUENCY_HZ)?;

        // Construct pwm driver
        let mut step_pwm = LedcDriver::new(channel, timer, pwm_pin)?;

        // Start disabled
        step_pwm.disable()?;

        // Set duty cycle
        step_pwm.set_duty(step_pwm.get_max_duty() / 2)?;

        let out = Self {
            name,
            step_pwm,
            dir_pin,
            delay,
            direction: Direction::Forward,
            enabled: false,
        };

        Ok(out)
    }

    pub async fn stop(&mut self) -> Result<(), SimpleStepperError> {
        self.step_pwm
            .disable()
            .map_err(|_| SimpleStepperError::Pwm)?;

        self.enabled = false;

        info!("{} motor STOPPED", self.name);

        Ok(())
    }

    pub async fn forward(&mut self) -> Result<(), SimpleStepperError> {
        Ok(self.run(Direction::Forward).await?)
    }

    pub async fn reverse(&mut self) -> Result<(), SimpleStepperError> {
        Ok(self.run(Direction::Reverse).await?)
    }

    /// Run the stepper at given speed
    pub async fn run(&mut self, direction: Direction) -> Result<(), SimpleStepperError> {
        info!(
            "{} motor running {}",
            self.name,
            match direction {
                Direction::Forward => "FORWARD",
                Direction::Reverse => "REVERSE",
            }
        );

        match direction {
            Direction::Forward => self
                .dir_pin
                .set_low()
                .map_err(|_| SimpleStepperError::DirectionPin)?,
            Direction::Reverse => self
                .dir_pin
                .set_high()
                .map_err(|_| SimpleStepperError::DirectionPin)?,
        };

        self.step_pwm
            .enable()
            .map_err(|_| SimpleStepperError::Pwm)?;

        self.enabled = true;

        Ok(())
    }
}
