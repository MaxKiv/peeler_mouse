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

    step_high_ticks: u32,
    step_period_ticks: u32,

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
            step_high_ticks: 5, // 5µs using default clock divider of 80 (1 µs tick)
            step_period_ticks: 1_000, // 1ms period (1khz)
            direction: Direction::Forward,
            running: false,
        }
    }

    /// Set speed via step frequency (Hz)
    pub fn set_speed_hz(&mut self, hz: u32) {
        self.step_period_ticks = 1_000_000 / hz;
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

    /// Single step using RMT pulse
    pub async fn step_once(&mut self) -> Result<(), StepperError> {
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

        self.rmt.start(signal).map_err(|_| StepperError::Rmt)?;

        // Wait for the total RMT signal duration, hacky but esp-idf-hal doesn't support RMT
        // interrupt atm
        self.delay.delay_ns(self.step_period_ticks).await;

        Ok(())
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
