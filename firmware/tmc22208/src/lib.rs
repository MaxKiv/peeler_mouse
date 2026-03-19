#![no_std]

use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};
use embedded_hal_async::delay::DelayNs;
use esp_idf_hal::{ledc::LedcDriver, units::Hertz};
use thiserror::Error;
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

const BASE_DUTY_CYCLE_PERCENT: u8 = 50;
pub const BASE_STEP_FREQUENCY_HZ: Hertz = Hertz(25_000);

use log::*;

pub struct Tb6600<DirPin, Delay> {
    name: &'static str,
    step_pwm: LedcDriver<'static>,
    step_frequency: Hertz,
    dir_pin: DirPin,
    enabled: bool,
    delay: Delay,
    /// Fierce googling: min pulse width ~5µs, maybe?
    pub direction: Direction,
}

#[derive(Error, Debug)]
pub enum TB6600Error {
    #[error("Direction pin ging op zn gat")]
    DirectionPin,
    #[error("Step pin ging op zn gat")]
    StepPin,
    #[error("Enable pin ging op zn gat")]
    EnablePin,
    #[error("Invalid input")]
    Input,
}

#[derive(Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
}

impl<DirPin, Delay> Tb6600<DirPin, Delay>
where
    DirPin: OutputPin, // Any output pin
    Delay: DelayNs,    // Nanosecond capable delay
{
    pub fn new(
        name: &'static str,
        timer: &'static esp_idf_hal::ledc::LedcTimerDriver<'static, esp_idf_hal::ledc::TIMER0>,
        channel_b: esp_idf_hal::ledc::CHANNEL2,
        pwm_pin_b: esp_idf_hal::gpio::Gpio13,
        dir_pin: DirPin,
        delay: Delay,
    ) -> Self {
        step_pwm.disable(); // start disabled

        let mut out = Self {
            name,
            step_pwm,
            step_frequency: BASE_STEP_FREQUENCY_HZ,
            dir_pin,
            delay,
            direction: Direction::Forward,
            enabled: false,
        };

        // start disabled
        out.stop();

        out
    }

    pub async fn halt() {
        self.run(
            Direction::Forward,
            Velocity::new::<millimeter_per_second>(0.0),
        )
        .await;
    }

    pub async fn forward(speed: Velocity) {
        self.run(Direction::Forward, speed).await;
    }
    pub async fn reverse(speed: Velocity) {
        self.run(Direction::Reverse, speed).await;
    }

    /// Run the stepper at given speed
    pub async fn run_with_dir(
        &mut self,
        speed: Velocity,
        dir: Direction,
    ) -> Result<(), TB6600Error> {
        match dir {
            Direction::Forward => self.run(speed).await,
            Direction::Reverse => self.run(-speed).await,
        }
    }

    /// Run the stepper at given speed
    pub async fn run(&mut self, speed: Velocity) -> Result<(), TB6600Error> {
        let mut speed = speed.get::<millimeter_per_second>();
        info!("{} motor RUNNING at {}mm/s", self.name, speed);

        // Check in which direction we should be running
        if speed > 0.0 {
            self.set_direction(Direction::Forward).await?;
        } else {
            self.set_direction(Direction::Reverse).await?;
            // make sure speed is positive from here
            speed = -speed;
        }

        // Set stepping frequency appropriate to the velocity setpoint
        // Validate setpoint
        if let Some(freq) = self.speed_to_frequency(speed) {
            self.step_pwm.set_frequency(freq);

            self.step_pwm
                .ch1()
                .set_duty_cycle_percent(BASE_DUTY_CYCLE_PERCENT); // set_frequency docs suggests I have to call this again

            self.start_stepping();

            self.step_frequency = freq;
        } else {
            // Setpoint invalid, stop motor
            self.stop();
        }

        Ok(())
    }

    /// Stop the stepper
    pub fn stop(&mut self) {
        info!("{} motor stopped stepping", self.name);
        self.step_pwm.disable();
        self.enabled = false;
    }

    /// Set step N times
    pub async fn do_steps(&mut self, num_steps: u32, speed: Velocity) {
        let us = ((num_steps as f32 / self.step_frequency.0 as f32) * 1_000_000.0) as u32;

        info!(
            "{} motor START stepping {} times -> {}us",
            self.name, num_steps, us
        );

        self.run(speed).await;
        self.delay.delay_us(us).await;

        info!(
            "{} motor DONE stepping {} times -> {}us",
            self.name, num_steps, us
        );
        self.stop();
    }

    /// Start stepping
    /// ENABLE LOW = ON + EN tied low -> Driver is always on
    /// Software enables the step pwm output
    fn start_stepping(&mut self) {
        info!(
            "{} motor started stepping at {}",
            self.name, self.step_frequency
        );
        self.step_pwm.enable();
        self.enabled = true;
    }

    /// Set step direction
    async fn set_direction(&mut self, direction: Direction) -> Result<(), TB6600Error> {
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
    async fn flip_direction(&mut self) -> Result<(), TB6600Error> {
        let dir = match self.direction {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        };

        self.set_direction(dir).await?;

        Ok(())
    }

    /// Set step pwm dc
    /// Percentage [0-100]
    fn set_duty_cycle_percent(&mut self, percentage: u8) {
        trace!("{} motor set dc to {}%", self.name, percentage);
        self.step_pwm.set_duty_cycle_percent(percentage);
    }

    fn speed_to_frequency(&self, speed: f32) -> Option<Hertz> {
        /// Speed [mm/s] at maximum stepping frequency
        const MAX_SPEED_MS_PS: f32 = 10.0;

        let speed_percentage = (speed / MAX_SPEED_MS_PS).clamp(0.0, 1.0);
        let freq = (speed_percentage * MAX_STEP_FREQUENCY_HZ.0 as f32) as u32;
        if freq > 0 {
            info!(
                "converting {} motor speed setpoint of {}mm/s to {}% speed ({})",
                self.name,
                speed,
                speed_percentage * 100.0,
                freq
            );

            Some(Hertz(freq))
        } else {
            warn!(
                "INVALID SPEED: attempting to convert {} motor speed setpoint of {}mm/s to {}% speed ({})",
                self.name,
                speed,
                speed_percentage * 100.0,
                freq
            );

            None
        }
    }
}
