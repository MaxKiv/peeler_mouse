use esp_idf_hal::gpio::*;

/// Defines the hardware resource required to use the camera
#[cfg(feature = "devkit")]
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

/// Defines the hardware resource required to use the camera
/// See: https://github.com/limengdu/SeeedStudio-XIAO-ESP32S3-Sense-camera/blob/main/README.md#camera-slot-circuit-design-for-expansion-boards
#[cfg(feature = "pcb")]
pub struct CameraPeripherals {
    pub pin_xclk: Gpio10,
    pub pin_d0: Gpio15,
    pub pin_d1: Gpio17,
    pub pin_d2: Gpio18,
    pub pin_d3: Gpio16,
    pub pin_d4: Gpio14,
    pub pin_d5: Gpio12,
    pub pin_d6: Gpio11,
    pub pin_d7: Gpio48,
    pub pin_vsync: Gpio38,
    pub pin_href: Gpio47,
    pub pin_pclk: Gpio13,
    pub pin_sda: Gpio40,
    pub pin_scl: Gpio39,
}
