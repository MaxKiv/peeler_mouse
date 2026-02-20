#![no_std]

use embedded_hal::{digital::PinState, pwm::SetDutyCycle as _};
use embedded_hal_async::delay::DelayNs;
use esp_idf_hal::{ledc::LedcDriver, units::Hertz};
use log::*;
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

pub const PWM_FREQUENCY: Hertz = Hertz(20_000); // 1-20kHz, low means audible noise, high = increased switching loss
pub const DEFAULT_DIR_STATE: PinState = PinState::Low;
pub const DEFAULT_DIRECTION: Direction = Direction::Forward;
pub const DEAD_TIME_US: u8 = 1; // TODO: validate
pub const DEFAULT_SPEED_MS_PS: f32 = 1.0;
pub const CUT_MAX_SPEED_MS_PS: f32 = 2.0;
/// Duration to break by shorting.
/// Increasing this increase heat release due to large ampererage in L9110 H-bridge short, potentially killing the device
pub const BREAK_DURATION_MS: u32 = 10;

pub struct L9110<Delay> {
    name: &'static str,
    pwm_a: LedcDriver<'static>,
    pwm_b: LedcDriver<'static>,
    delay: Delay,
    speed: Velocity,
    direction: Direction,
}

#[derive(Clone, Debug)]
pub enum Direction {
    Forward,
    Reverse,
}

impl Direction {
    pub fn flip(&mut self) {
        *self = match self {
            Direction::Forward => Direction::Reverse,
            Direction::Reverse => Direction::Forward,
        }
    }
}

impl<Delay> L9110<Delay>
where
    Delay: DelayNs,
{
    pub fn try_new(
        name: &'static str,
        mut timer: esp_idf_hal::ledc::LedcTimerDriver<'static, esp_idf_hal::ledc::TIMER1>,
        ch_a: esp_idf_hal::ledc::CHANNEL1,
        pin_a: esp_idf_hal::gpio::Gpio47,
        ch_b: esp_idf_hal::ledc::CHANNEL2,
        pin_b: esp_idf_hal::gpio::Gpio21,
        delay: Delay,
    ) -> anyhow::Result<Self> {
        // Set desired pwm frequency
        timer.set_frequency(PWM_FREQUENCY)?;

        // Construct LedcDriver
        let mut pwm_a = LedcDriver::new(ch_a, &timer, pin_a)?;
        let mut pwm_b = LedcDriver::new(ch_b, &timer, pin_b)?;

        // Disable PWM output at start
        pwm_a.set_duty_cycle_fully_off()?;
        pwm_b.set_duty_cycle_fully_off()?;

        // The L9110 driver requires the PWM channels to be enabled after construction, do that now
        pwm_a.enable()?;
        pwm_b.enable()?;

        // Construct Driver
        let mut out = Self {
            name,
            delay,
            speed: Velocity::new::<millimeter_per_second>(DEFAULT_SPEED_MS_PS),
            direction: DEFAULT_DIRECTION,
            pwm_a,
            pwm_b,
        };

        // Default driver state
        out.coast();

        Ok(out)
    }

    pub fn forward(&mut self, speed: Velocity) {
        let dc = Self::speed_to_duty_cycle_percent(speed);

        info!(
            "{} moving forward with {}mm/s ({}%dc)",
            self.name,
            speed.get::<millimeter_per_second>(),
            dc
        );

        if self.pwm_a.set_duty_cycle_percent(dc).is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
        if self.pwm_b.set_duty_cycle_fully_off().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
    }

    pub fn reverse(&mut self, speed: Velocity) {
        let dc = Self::speed_to_duty_cycle_percent(speed);
        info!(
            "{} moving reverse with {}mm/s ({}%dc)",
            self.name,
            speed.get::<millimeter_per_second>(),
            dc
        );

        if self.pwm_a.set_duty_cycle_fully_off().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
        if self.pwm_b.set_duty_cycle_percent(dc).is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
    }

    pub fn run(&mut self, mut speed: Velocity) {
        let mm_ps = speed.get::<millimeter_per_second>();
        if mm_ps.abs() > CUT_MAX_SPEED_MS_PS {
            warn!(
                "{} attempting to set speed to {}mm/s, exceeding max speed ({}mm/s) - clipping to max",
                self.name, mm_ps, CUT_MAX_SPEED_MS_PS
            );
            speed = Velocity::new::<millimeter_per_second>(CUT_MAX_SPEED_MS_PS);
        }

        if mm_ps > 0.0 {
            self.forward(speed);
        } else {
            self.reverse(speed);
        }
    }

    pub fn coast(&mut self) {
        info!("{} coasting", self.name);
        if self.pwm_a.set_duty_cycle_fully_off().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
        if self.pwm_b.set_duty_cycle_fully_off().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
    }

    /// Break by shorting, coast after
    pub async fn short_break(&mut self) {
        info!("{} breaking!", self.name);
        // Break by shorting motor
        if self.pwm_a.set_duty_cycle_fully_on().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }
        if self.pwm_b.set_duty_cycle_fully_on().is_err() {
            log::error!("L9110: Unable to set duty cycle");
        }

        // avoid shorting h bridge high side transistors for too long
        self.delay.delay_ms(BREAK_DURATION_MS).await;
        self.coast();
    }

    fn speed_to_duty_cycle_percent(speed: Velocity) -> u8 {
        let speed_abs = speed.get::<millimeter_per_second>().abs();

        let out = ((100.0 * speed_abs / CUT_MAX_SPEED_MS_PS) as u8).clamp(0, 100);

        trace!(
            "Converted speed {}mm/s to {}% dc",
            speed.get::<millimeter_per_second>(),
            out,
        );

        out
    }
}
