use defmt::*;
use display_interface_i2c::I2CInterface;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri,
    i2c::{Config, I2c},
    peripherals::*,
};
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig};
use oled_async::{builder::Builder, mode::GraphicsMode, prelude::DisplayRotation};
use ratatui::Terminal;

use crate::{
    Irqs,
    comms::task::REPORT_WATCH,
    hmi::lcd::manage_display,
    supervisor::{appstate::APP_STATE_WATCH, task::HMI_STATE_WATCH},
};

extern crate alloc;

/// Period at which this task is ticked
const ADDRESS: u8 = 0x3C;
const DATA_BYTE: u8 = 0x40;
pub const SSD1309_FRAMEBUFFER_SIZE: usize = 128 * 64 / 8;

pub struct LcdPeripherals {
    pub i2c: Peri<'static, I2C2>,
    pub sda: Peri<'static, PF0>,
    pub scl: Peri<'static, PF1>,
    pub tx_dma: Peri<'static, DMA1_CH4>,
    pub rx_dma: Peri<'static, DMA1_CH5>,
}

pub fn setup(p: LcdPeripherals, spawner: &Spawner) {
    info!("Setting up display");
    let i2c_cfg = {
        let mut cfg = Config::default();
        cfg.sda_pullup = true;
        cfg.scl_pullup = true;
        cfg
    };

    // Set up I2C
    let i2c = I2c::new(p.i2c, p.scl, p.sda, Irqs, p.tx_dma, p.rx_dma, i2c_cfg);
    let i2c_interface = I2CInterface::new(i2c, ADDRESS, DATA_BYTE);

    // Set up display
    type Display = oled_async::displays::ssd1309::Ssd1309_128_64;
    let raw_disp = Builder::new(Display {})
        .with_rotation(DisplayRotation::Rotate0)
        .connect(i2c_interface);
    let display: GraphicsMode<_, _, { SSD1309_FRAMEBUFFER_SIZE }> = raw_disp.into();

    // Inputs
    let appstate_rx = APP_STATE_WATCH
        .receiver()
        .expect("increase APPSTATE_WATCH N");
    let report_rx = REPORT_WATCH.receiver().expect("increase REPORT_WATCH N");

    spawner
        .spawn(manage_display(display, appstate_rx, report_rx))
        .unwrap();
}
