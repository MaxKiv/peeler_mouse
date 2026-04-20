use esp_idf_hal::{
    gpio::*,
    ledc::{CHANNEL0, TIMER0},
};

#[cfg(feature = "devkit")]
pub struct ControlPeripherals {
    pub led_timer: TIMER0,
    pub led_ch: CHANNEL0,
    pub led_pin: Gpio48,
}

#[cfg(feature = "pcb")]
pub struct ControlPeripherals {
    pub led_timer: TIMER0,
    pub led_ch: CHANNEL0,
    pub led_pin: Gpio5,
}
