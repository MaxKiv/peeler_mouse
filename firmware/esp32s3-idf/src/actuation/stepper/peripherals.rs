use esp_idf_hal::{gpio::*, ledc::TIMER1, spi::*};

pub struct MotorPeripherals {
    pub timer: TIMER1,
    pub channel: esp_idf_hal::ledc::CHANNEL1,
    pub pwm_pin: esp_idf_hal::gpio::Gpio41,
    pub dir_pin: esp_idf_hal::gpio::Gpio42,
    pub limit_switch: esp_idf_hal::gpio::Gpio45,
}
