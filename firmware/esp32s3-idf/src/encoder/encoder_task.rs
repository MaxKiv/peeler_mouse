use as5048a_async::{As5048a, ANGLE_MAX};
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker};
use embedded_hal_async::spi::{self, SpiDevice};
use esp_idf_hal::gpio::Output;
use esp_idf_hal::spi::{config, Spi, SpiDeviceDriver};
use esp_idf_hal::units::*;
use esp_idf_hal::{
    gpio,
    io::asynch::Write,
    spi::{SpiDriver, SpiDriverConfig, SPI3},
};
use log::*;
use static_cell::StaticCell;

use crate::encoder::peripherals::EncoderPeripherals;

const DURATION: Duration = Duration::from_hz(10);

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: EncoderPeripherals) -> anyhow::Result<()> {
    log::info!("initialising Encoder task");

    spawner.spawn(encoder_task(p))?;

    Ok(())
}

// UART RX COMMS task, pushes serialised setpoints over the wire
#[embassy_executor::task]
pub async fn encoder_task(p: EncoderPeripherals) {
    info!("ENCODER: Starting task");

    let mut ticker = Ticker::every(DURATION);

    log::info!("Encoder: Initialising SPI");

    let spi = SpiDriver::new::<SPI3>(
        p.spi,
        p.sclk,
        p.serial_out,
        Some(p.serial_in),
        &SpiDriverConfig::new(),
    )
    .expect("ENCODER: unable to set up SpiDriver");

    let spi_cfg = config::Config::new()
        .bit_order(config::BitOrder::MsbFirst)
        .data_mode(spi::MODE_1)
        .baudrate(1.MHz().into())
        .cs_pre_delay_us(1);
    let mut spi_device = SpiDeviceDriver::new(&spi, Some(p.cs), &spi_cfg)
        .expect("ENCODER: unable to set up SpiDeviceDriver");

    let mut sensor = As5048a::new(&mut spi_device);

    loop {
        ticker.next().await;

        let diag = sensor.diagnostics().await;
        match diag {
            Ok(diag) => {
                if diag.is_valid() {
                    info!("ENCODER: Diagnostics are valid!");

                    let angle = sensor.angle().await;
                    match angle {
                        Ok(angle) => {
                            info!("ENCODER: ANGLE => {}", angle);
                            // angle as f32 / ANGLE_MAX as f32
                        }
                        Err(err) => {
                            error!("ENCODER: sensor angle err: {:?}", err);
                        }
                    };

                    let magnitude = sensor.magnitude().await;
                    match magnitude {
                        Ok(magnitude) => {
                            info!("ENCODER: MAGNITUDE => {}", magnitude);
                        }
                        Err(err) => {
                            error!("ENCODER: sensor magnitude err: {:?}", err);
                        }
                    };
                } else if diag.cordic_overflow() {
                    error!("ENCODER: CORDIC overflow - data invalid!");
                } else if diag.comp_high() {
                    error!("ENCODER: Magnet too close");
                } else if diag.comp_low() {
                    error!("ENCODER: Magnet too far");
                }
            }
            Err(err) => {
                error!("ENCODER: SPI error: {:?}", err);
                let _ = sensor.clear_error_flag().await;
            }
        };
    }
}
