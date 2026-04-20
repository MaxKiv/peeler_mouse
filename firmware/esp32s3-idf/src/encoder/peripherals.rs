use esp_idf_hal::{gpio::*, spi::*};

#[cfg(feature = "devkit")]
pub struct EncoderPeripherals {
    pub spi: SPI3, // SPI0 & SPI1 are used by the cpu/dma controller to access PSRAM, SPI2 Prio 1 pins are used by SDMMC driver
    pub sclk: Gpio1,
    pub serial_out: Gpio2,
    pub serial_in: Gpio21,
    pub cs: Gpio47,
}

#[cfg(feature = "pcb")]
pub struct EncoderPeripherals {
    pub spi: SPI3, // SPI0 & SPI1 are used by the cpu/dma controller to access PSRAM, SPI2 Prio 1 pins are used by SDMMC driver
    pub sclk: Gpio7,
    pub serial_out: Gpio9,
    pub serial_in: Gpio8,
    pub cs: Gpio6,
}
