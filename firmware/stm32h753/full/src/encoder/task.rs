use as5048a_async::As5048a;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::Output,
    mode::Async,
    spi::{BitOrder, Config, Spi},
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex as Cs, NoopRawMutex},
    mutex::Mutex,
    pipe::{self, Pipe},
    watch::{self, Watch},
};
use static_cell::StaticCell;

use crate::{Irqs, comms::peripherals::CommsPeripherals};

// pub static SETPOINT_WATCH: Watch<Cs, messenger_mouse::Setpoint, 1> = Watch::new();

pub static SPI_BUS: StaticCell<Mutex<NoopRawMutex, Spi<'static, Async>>> = StaticCell::new();

pub fn setup(spawner: &Spawner, p: CommsPeripherals) {
    info!("Setting up Supervisor");

    spawner.spawn(encoder_task(sensor)).unwrap();
}

/// Receives bytes over uart
#[embassy_executor::task]
async fn encoder_task(p: CommsPeripherals) {
    // Configure SPI bus
    let mut spi_config = Config::default();
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.mode = MODE_0;
    // IS31FL3743B can handle up to 12 MHz
    spi_config.frequency = Hertz(10_000_000);

    // Instantiate your SPI bus, this example uses embassy_stm32::spi::Spi
    let spi = Spi::new(
        p.SPI4, p.PB13, p.PA1, p.PA11, p.DMA2_CH1, p.DMA2_CH0, spi_config,
    );

    // Create mutex guarding the SPI bus and initialize the StaticCell with it
    let mutex: Mutex<NoopRawMutex, Spi<'_, Async>> = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(mutex);

    // Define your chip select pins for your peripheral devices
    let cs = Output::new(p.PB1, Level::High, Speed::VeryHigh);

    // Now we can define multiple SPI devices with each chip select pin
    let spi_dev = SpiDevice::new(spi_bus, cs);

    // Create sensor with an SpiDevice (bus + CS pin)
    let mut sensor = As5048a::new(spi_dev);

    loop {
        // Read angle (0-16383, representing 0-360°)
        let angle = sensor.angle().await?;
        let degrees = (angle as f32) * 360.0 / 16384.0;
    }
}
