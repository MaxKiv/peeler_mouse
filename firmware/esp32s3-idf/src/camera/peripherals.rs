use esp_idf_hal::{gpio::*, sd::mmc::SDMMC1};

// Define the CameraPeripherals struct with concrete GPIO pin types
pub struct CameraPeripherals {
    pub pin_xclk: Gpio15,
    pub pin_d0: Gpio11,
    pub pin_d1: Gpio9,
    pub pin_d2: Gpio8,
    pub pin_d3: Gpio10,
    pub pin_d4: Gpio12,
    pub pin_d5: Gpio18,
    pub pin_d6: Gpio17,
    pub pin_d7: Gpio16,
    pub pin_vsync: Gpio6,
    pub pin_href: Gpio7,
    pub pin_pclk: Gpio13,
    pub pin_sda: Gpio4,
    pub pin_scl: Gpio5,
}
