#![no_std]

use defmt::trace;
use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use thiserror::Error;

pub struct Tb6600<StepPin, DirPin, Delay> {
    step_pin: StepPin,
    dir_pin: DirPin,
    direction: Direction,
    // enable: Option<EnablePin>,
    // enabled: bool,
    delay: Delay,
    /// Fierce googling: min pulse width ~5µs, maybe?
    pulse_period_us: u32,
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

#[derive(Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
}

impl<StepPin, DirPin, Delay> Tb6600<StepPin, DirPin, Delay>
where
    StepPin: OutputPin,
    DirPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(step_pin: StepPin, dir_pin: DirPin, delay: Delay, pulse_period_us: u32) -> Self {
        Self {
            step_pin,
            dir_pin,
            delay,
            direction: Direction::Forward,
            pulse_period_us,
        }
    }

    // ENABLE LOW = ON + EN tied low -> Driver is always on
    pub fn enable(&mut self) {}

    // ENABLE LOW = ON + EN tied low -> Driver is always on
    pub fn disable(&mut self) {}

    // Set step direction
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

    pub async fn flip_direction(&mut self) -> Result<(), TB6600Error> {
        let dir = match self.direction {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        };

        self.set_direction(dir).await?;

        Ok(())
    }

    // Perform a single step
    pub async fn step_once(&mut self) -> Result<(), TB6600Error> {
        trace!("Stepper stepping once");

        self.step_pin.set_high().map_err(|_| TB6600Error::StepPin)?;

        self.delay.delay_us(self.pulse_period_us).await;

        self.step_pin.set_low().map_err(|_| TB6600Error::StepPin)?;

        self.delay.delay_us(self.pulse_period_us).await;

        Ok(())
    }

    pub async fn step_n(&mut self, steps: u32) -> Result<(), TB6600Error> {
        for _ in 0..steps {
            self.step_once().await.map_err(|_| TB6600Error::StepPin)?;
        }

        Ok(())
    }
}
