use esp_idf_hal::{gpio::*, sd::mmc::SDMMC1};

pub struct SDPeripherals {
    pub slot: SDMMC1,
    pub cmd: Gpio38,
    pub clk: Gpio39,
    pub d0: Gpio40,
}
