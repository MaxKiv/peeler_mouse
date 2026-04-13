use esp_idf_hal::{gpio::*, uart::*};

pub struct CommsPeripherals {
    pub uart: UART2,
    pub tx: Gpio19,
    pub rx: Gpio20,
}
