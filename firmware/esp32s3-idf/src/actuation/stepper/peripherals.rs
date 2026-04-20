use esp_idf_hal::{gpio::*, ledc::TIMER1, spi::*};

#[cfg(feature = "devkit")]
pub struct StepperPeripherals {
    pub timer: TIMER1,
    pub rmt_channel: esp_idf_hal::rmt::CHANNEL0,
    pub step_rmt_pin: esp_idf_hal::gpio::Gpio41,
    pub dir_pin: esp_idf_hal::gpio::Gpio42,
    pub enable_pin: esp_idf_hal::gpio::Gpio40,
}
#[cfg(feature = "devkit")]
pub struct MotorPeripherals {
    pub stepper: StepperPeripherals,
    pub limit_switch: esp_idf_hal::gpio::Gpio45,
}

#[cfg(feature = "pcb")]
pub struct StepperPeripherals {
    pub timer: TIMER1,
    pub rmt_channel: esp_idf_hal::rmt::CHANNEL0,
    pub step_rmt_pin: esp_idf_hal::gpio::Gpio3,
    pub dir_pin: esp_idf_hal::gpio::Gpio2,
    pub enable_pin: esp_idf_hal::gpio::Gpio4,
}
#[cfg(feature = "pcb")]
pub struct MotorPeripherals {
    pub stepper: StepperPeripherals,
    pub limit_switch: esp_idf_hal::gpio::Gpio1,
}
