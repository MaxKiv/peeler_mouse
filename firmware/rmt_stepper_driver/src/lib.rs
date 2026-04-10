//! Simple RMT based stepper driver

#![no_std]

use embedded_hal::digital::OutputPin;
use embedded_hal_async::delay::DelayNs;
use esp_idf_hal::rmt::{FixedLengthSignal, PinState, Pulse, PulseTicks, TxRmtDriver};
use thiserror::Error;

use log::*;

use messenger_mouse::motor::MotorDirection as Direction;

#[derive(Error, Debug)]
pub enum StepperError {
    #[error("RMT error")]
    Rmt,
    #[error("Direction pin error")]
    DirPin,
}

/// KISS RMT stepper driver
pub struct RmtStepper<DirPin, Delay> {
    name: &'static str,
    rmt: TxRmtDriver<'static>,
    dir_pin: DirPin,
    delay: Delay,

    step_high_ticks: u16,
    step_period_ticks: u16,

    direction: Direction,
    running: bool,
}

impl<DirPin, Delay> RmtStepper<DirPin, Delay>
where
    DirPin: OutputPin,
    Delay: DelayNs,
{
    pub fn new(
        name: &'static str,
        rmt: TxRmtDriver<'static>,
        dir_pin: DirPin,
        delay: Delay,
    ) -> Self {
        Self {
            name,
            rmt,
            dir_pin,
            delay,
            step_high_ticks: 1, // 20µs using default clock divider of 80 (1 µs tick)
            step_period_ticks: 100, // 1ms period (1khz)
            direction: Direction::Forward,
            running: false,
        }
    }

    pub fn set_step_period(&mut self, step_period_ticks: u16) {
        self.step_period_ticks = step_period_ticks;
    }

    pub async fn set_direction(&mut self, dir: Direction) -> Result<(), StepperError> {
        match dir {
            Direction::Forward => self.dir_pin.set_low().map_err(|_| StepperError::DirPin)?,
            Direction::Reverse => self.dir_pin.set_high().map_err(|_| StepperError::DirPin)?,
        }

        // Make sure Dir is stable before start of movement
        self.delay.delay_ns(10_000).await;

        self.direction = dir;
        Ok(())
    }

    /// Start Indefinite RMT pulse stepping
    pub async fn start_stepping(&mut self) -> Result<(), StepperError> {
        let signal = self.create_signal()?;

        self.rmt
            .set_looping(esp_idf_hal::rmt::config::Loop::Endless)
            .map_err(|_| StepperError::Rmt)?;

        self.rmt.start(signal).map_err(|_| StepperError::Rmt)?;

        Ok(())
    }

    /// Stop Indefinite RMT pulse stepping
    pub async fn stop_stepping(&mut self) -> Result<(), StepperError> {
        self.rmt.stop().map_err(|_| StepperError::Rmt)?;
        Ok(())
    }

    /// Single step using RMT pulse
    pub async fn step_once(&mut self) -> Result<(), StepperError> {
        let signal = self.create_signal()?;

        self.rmt.start(signal).map_err(|_| StepperError::Rmt)?;

        // Wait for the total RMT signal duration, hacky but esp-idf-hal doesn't support RMT
        // interrupt atm
        self.delay.delay_ns(self.step_period_ticks as u32).await;

        Ok(())
    }

    /// Single step using RMT pulse
    pub async fn do_n_steps(&mut self, n: u32) -> Result<(), StepperError> {
        let signal = self.create_signal()?;

        self.rmt
            .set_looping(esp_idf_hal::rmt::config::Loop::Count(n))
            .map_err(|_| StepperError::Rmt)?;

        self.rmt.start(signal).map_err(|_| StepperError::Rmt)?;

        Ok(())
    }

    fn create_signal(&self) -> Result<FixedLengthSignal<1>, StepperError> {
        let mut signal = FixedLengthSignal::<1>::new();

        let high: Pulse = Pulse::new(
            PinState::High,
            PulseTicks::new((self.step_high_ticks) as u16)
                .expect("unable to construct pulseticks HIGH"),
        );
        let low: Pulse = Pulse::new(
            PinState::Low,
            PulseTicks::new((self.step_period_ticks - self.step_high_ticks) as u16)
                .expect("unable to construct pulseticks LOW"),
        );

        signal.set(0, &(high, low)).map_err(|_| StepperError::Rmt)?;

        Ok(signal)
    }

    /// Run continuously until stopped
    pub async fn run(&mut self) -> Result<(), StepperError> {
        self.running = true;

        info!("{} running", self.name);

        while self.running {
            self.step_once().await?;
        }

        Ok(())
    }

    pub fn stop(&mut self) {
        self.running = false;
        info!("{} stopped", self.name);
    }

    /// Move fixed number of steps (blocking async)
    pub async fn move_steps(&mut self, steps: u32) -> Result<(), StepperError> {
        info!("{} moving {} steps", self.name, steps);

        for _ in 0..steps {
            self.step_once().await?;
        }

        Ok(())
    }
}
