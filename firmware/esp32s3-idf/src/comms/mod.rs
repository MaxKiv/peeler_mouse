use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use esp_idf_hal::{
    gpio,
    uart::{config, AsyncUartDriver},
    units::Hertz,
};
use log::*;

pub fn run(spawner: &Spawner, comms_peri: CommsPeripherals) -> anyhow::Result<()> {
    log::info!("initialising Comms task");

    log::info!("Initialising Comms Async UART");
    let config = config::Config::new().baudrate(Hertz(115_200));
    let uart = AsyncUartDriver::new(
        comms_peri.uart,
        comms_peri.tx,
        comms_peri.rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    )?;

    let mut cfg = SdMmcHostConfiguration::new();
    // It seems the esp32s3-cam has external pullups, but who really knows without a datasheet?
    cfg.enable_internal_pullups = false;
    let sd_card_driver = SdCardDriver::new_mmc(
        // => Data width = 1 bit
        SdMmcHostDriver::new_1bit(
            sd_peri.slot,
            sd_peri.cmd,
            sd_peri.clk,
            sd_peri.d0,
            None::<gpio::AnyIOPin>,
            None::<gpio::AnyIOPin>,
            &cfg,
        )
        .expect("unable to construct SdMmcHostDriver"),
        &SdCardConfiguration::new(),
    )
    .expect("Unable to construct SdCardDriver");

    log::info!("SD card driver initialised");

    spawner.spawn(log_to_sd_task(sd_card_driver))?;

    Ok(())
}

#[embassy_executor::task]
pub async fn tx_task() {
    info!("COMMS: Starting TX task");
    loop {}
}

// #[embassy_executor::task]
// /// Forward firmware state reports to the HHH host
// pub async fn forward_reports(
//     mut uart_tx: BufferedUartTx<'static>,
//     report_pipe_rx: pipe::Reader<'static, Cs, { love_letter::REPORT_BYTES * 4 }>,
// ) {
//     let mut buf = [0u8; 64];
//
//     loop {
//         // Get latest serialised report from the framing task
//         let n = report_pipe_rx.read(&mut buf).await;
//         info!("COMMS - forward_reports: writing {} bytes to UART", n);
//         if let Err(err) = uart_tx.write(&buf[..n]).await {
//             error!(
//                 "COMMS - forward_reports: {} unable to write serialised report bytes {:?} to UART",
//                 err,
//                 buf[..n]
//             );
//         }
//     }
// }
//
// #[embassy_executor::task]
// /// Collects UART bytes into a pipe for later processing in framing_task
// pub async fn receive_setpoints(
//     mut uart_rx: BufferedUartRx<'static>,
//     setpoint_pipe_tx: pipe::Writer<'static, Cs, { love_letter::SETPOINT_BYTES * 4 }>,
// ) {
//     const BUF_SIZE: usize = 8;
//     let mut buf = [0u8; BUF_SIZE];
//
//     loop {
//         if uart_rx.read_exact(&mut buf).await.is_ok() {
//             info!(
//                 "COMMS - receive_setpoints: read {} bytes: {}",
//                 BUF_SIZE, buf
//             );
//
//             let mut written = 0;
//             while written < BUF_SIZE {
//                 written += setpoint_pipe_tx.write(&buf[written..]).await;
//             }
//
//             debug!("COMMS - receive_setpoints: write {} bytes to pipe", written);
//         }
//     }
// }
