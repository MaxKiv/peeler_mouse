use esp_idf_hal::{gpio::*, uart::*};

pub struct CommsPeripherals {
    pub uart: UART2,
    pub tx: Gpio43,
    pub rx: Gpio44,
}
