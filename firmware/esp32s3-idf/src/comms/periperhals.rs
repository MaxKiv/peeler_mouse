use esp_idf_hal::{gpio::*, uart::UART0};

pub struct CommsPeripherals {
    pub uart: UART0,
    pub tx: Gpio43,
    pub rx: Gpio44,
}
