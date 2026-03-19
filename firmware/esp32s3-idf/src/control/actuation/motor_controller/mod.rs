pub mod motor_command;

use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::gpio::{Gpio40, Input, PinDriver};
use l9110::L9110;
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::control::actuation::motor_controller::motor_command::MotorDirection;

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 1.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;
pub const DEBOUNCE_DURATION: Duration = Duration::from_millis(5);

pub struct MotorController {
    motor: L9110<Delay>,
    limit_switch: PinDriver<'static, Gpio40, Input>,
    homed: bool,
}

impl MotorController {
    pub fn new(motor: L9110<Delay>, limit_switch: PinDriver<'static, Gpio40, Input>) -> Self {
        Self {
            motor,
            limit_switch,
            homed: false,
        }
    }

    pub async fn halt(&mut self) {
        self.motor.short_break().await;
    }

    pub async fn home(&mut self) {
        // Move in homing direction
        self.motor.move_in_direction(
            Velocity::new::<millimeter_per_second>(
                SPEED_REV_PS
                    * HOMING_SPEED_MM_PS
                    * (1.0 / messenger_mouse::encoder::KNIFE_AXIS_LEAD),
            ),
            HOMING_DIRECTION.into(),
        );

        // Wait for limit_switch to indicate home reached
        loop {
            self.limit_switch.wait_for_falling_edge().await;

            // debounce
            Timer::after(DEBOUNCE_DURATION).await;
            if self.limit_switch.is_low() {
                // valid home position, stop motor
                self.motor.coast();

                self.homed = true;

                return;
            }
        }
    }

    pub async fn run(&mut self, direction: MotorDirection, speed: Velocity) {
        self.motor.short_break().await;
    }
}
