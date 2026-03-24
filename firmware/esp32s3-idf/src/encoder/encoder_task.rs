use as5048a_async::{As5048a, Error};
use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    signal::Signal,
    watch::{self, Watch},
};
use embassy_time::{Duration, Ticker};
use embedded_hal_async::spi::{self, SpiDevice};
use esp_idf_hal::spi::{config, Spi, SpiDeviceDriver};
use esp_idf_hal::spi::{SpiDriver, SpiDriverConfig, SPI3};
use esp_idf_hal::units::*;
use log::*;
use messenger_mouse::encoder::{EncoderError, EncoderValidity};

use crate::encoder::peripherals::EncoderPeripherals;
use messenger_mouse::encoder::KnifeState;

const DURATION: Duration = Duration::from_hz(10);

pub static KNIFE_STATE: Watch<Cs, KnifeState, 2> = Watch::new();
pub static ENCODER_RESET: Signal<Cs, bool> = Signal::new();

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: EncoderPeripherals) -> anyhow::Result<()> {
    log::info!("initialising Encoder task");

    spawner.spawn(encoder_task(p))?;

    Ok(())
}

// Track encoder position for consumption in other tasks
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

    // Initialize encoder state
    let mut knife_state = KnifeState::new();
    let tx = KNIFE_STATE.sender();

    log::info!("Encoder: Initialisation done, starting loop");
    loop {
        // Continously wait for either the sampling tick or a reset signal
        let event = embassy_futures::select::select(ticker.next(), ENCODER_RESET.wait()).await;

        match event {
            // Sampling tick
            Either::First(_) => {
                match sensor.angle().await {
                    // Happy path
                    Ok(angle) => {
                        info!("ENCODER: got angle {}", angle);

                        knife_state.encoder_state.update(angle);
                    }

                    // Shit hit the fan
                    Err(err) => {
                        error!("ENCODER: Error {:?}", err);

                        // Investigate problem, not much we can do but continue however
                        match err {
                            Error::Communication(_) => {
                                // SPI error, likely unrecoverable
                                knife_state.validity =
                                    EncoderValidity::EncoderError(EncoderError::Communication)
                            }
                            Error::ParityError => {
                                // Parity or Sensor error, likely something wrong with SPI Bus, might recover
                                if let Err(err) = sensor.clear_error_flag().await {
                                    knife_state.validity =
                                        EncoderValidity::EncoderError(EncoderError::ParityError)
                                }
                            }
                            Error::SensorError => {
                                // Parity or Sensor error, likely something wrong with SPI Bus, might recover
                                if let Err(err) = sensor.clear_error_flag().await {
                                    knife_state.validity =
                                        EncoderValidity::EncoderError(EncoderError::SensorError)
                                }
                            }
                        }
                    }
                }

                // Make latest state available for consumers
                tx.send(knife_state.clone());
            }

            // Reset signal received, reset the encoder state
            Either::Second(_) => {
                knife_state.encoder_state.reset();
                info!("ENCODER: RESET");
            }
        }
    }
}
