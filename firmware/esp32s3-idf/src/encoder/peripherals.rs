use esp_idf_hal::{gpio::*, spi::*};

pub struct EncoderPeripherals {
    pub spi: SPI3, // SPI0 & SPI1 are used by the cpu/dma controller to access PSRAM, SPI2 Prio 1 pins are used by SDMMC driver
    pub sclk: Gpio1,
    pub serial_out: Gpio2,
    pub serial_in: Gpio21,
    pub cs: Gpio47,
}
