use esp_idf_hal::{gpio::*, uart::*};

#[cfg(feature = "devkit")]
pub struct CommsPeripherals {
    pub uart: UART2,
    pub tx: Gpio19,
    pub rx: Gpio20,
}

#[cfg(feature = "pcb")]
pub struct CommsPeripherals {
    pub uart: UART2,
    pub tx: Gpio43,
    pub rx: Gpio44,
}
