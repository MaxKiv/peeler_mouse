use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::usart::{self, BufferedUart, BufferedUartRx, BufferedUartTx};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    pipe::{self, Pipe},
    watch::{self, Watch},
};
use embassy_time::Timer;
use embedded_io_async::Read;
use embedded_io_async::Write;
use messenger_mouse::{Report, Setpoint};
use static_cell::StaticCell;

use crate::{Irqs, comms::peripherals::CommsPeripherals};

static RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
static TX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();

static REPORT_PIPE: StaticCell<pipe::Pipe<Cs, { messenger_mouse::REPORT_BYTES * 4 }>> =
    StaticCell::new();
static SETPOINT_PIPE: StaticCell<pipe::Pipe<Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>> =
    StaticCell::new();

pub static REPORT_WATCH: Watch<Cs, messenger_mouse::Report, 1> = Watch::new();
pub static SETPOINT_WATCH: Watch<Cs, messenger_mouse::Setpoint, 1> = Watch::new();

pub fn setup(spawner: &Spawner, p: CommsPeripherals) {
    info!("Setting up Supervisor");

    let report_pipe = REPORT_PIPE.init(Pipe::new());
    let setpoint_pipe = SETPOINT_PIPE.init(Pipe::new());
    let (report_pipe_rx, report_pipe_tx) = report_pipe.split();
    let (setpoint_pipe_rx, setpoint_pipe_tx) = setpoint_pipe.split();

    // Construct the BufferedUart, a structure that allows us to process received uart bytes from a
    // ring buffer that is continously filled by DMA, and send uart bytes using a software FIFO
    let mut uart_cfg = usart::Config::default();
    uart_cfg.baudrate = messenger_mouse::BAUDRATE;
    let rx = p.rx;
    let tx = p.tx;
    let tx_buffer = &mut TX_BUF.init([0u8; 2048])[..];
    let rx_buffer = &mut RX_BUF.init([0u8; 2048])[..];
    let uart = BufferedUart::new(p.uart, rx, tx, tx_buffer, rx_buffer, Irqs, uart_cfg).unwrap();

    // Split UART into RX/TX halves
    let (uart_tx, uart_rx) = uart.split();

    spawner.spawn(rx_task(uart_rx, report_pipe_tx)).unwrap();
    spawner.spawn(tx_task(uart_tx, setpoint_pipe_rx)).unwrap();
    spawner
        .spawn(frame_and_serialise_reports(
            REPORT_WATCH.sender(),
            report_pipe_rx,
        ))
        .unwrap();
    spawner
        .spawn(serialise_setpoints(
            SETPOINT_WATCH
                .receiver()
                .expect("Increase SETPOINT_WATCH N"),
            setpoint_pipe_tx,
        ))
        .unwrap();
}

/// Receives bytes over uart
#[embassy_executor::task]
async fn rx_task(
    mut uart_rx: BufferedUartRx<'static>,
    mut report_pipe_tx: pipe::Writer<'static, Cs, { messenger_mouse::REPORT_BYTES * 4 }>,
) {
    let mut buf = [0u8; messenger_mouse::REPORT_BYTES * 1];

    loop {
        match uart_rx.read(&mut buf).await {
            Ok(n) => {
                info!("read {} bytes: {:?}", n, &buf[..n]);
                // Read N bytes, send along for framing
                if let Err(err) = report_pipe_tx.write_all(&buf[..n]).await {
                    error!("COMMS: RX error: {:?}", err);
                }
            }
            Err(err) => error!("COMMS: RX error: {:?}", err),
        }
    }
}

/// Sends bytes over uart
#[embassy_executor::task]
async fn tx_task(
    mut uart_tx: BufferedUartTx<'static>,
    setpoint_pipe_rx: pipe::Reader<'static, Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>,
) {
    let mut buf = [0u8; 64];

    loop {
        // Get latest serialised report from the framing task
        let n = setpoint_pipe_rx.read(&mut buf).await;
        debug!("COMMS - tx_task: writing {} bytes to UART", n);
        if let Err(err) = uart_tx.write_all(&buf[..n]).await {
            error!(
                "COMMS - tx_task: {} unable to write serialised setpoint bytes {:?} to UART",
                err,
                buf[..n]
            );
        }
    }
}

#[embassy_executor::task]
/// Frame received bytes into
pub async fn frame_and_serialise_reports(
    report_sender: watch::Sender<'static, Cs, Report, 1>,
    report_pipe_rx: pipe::Reader<'static, Cs, { messenger_mouse::REPORT_BYTES * 4 }>,
) {
    let mut framing_buf = heapless::Vec::<u8, { messenger_mouse::REPORT_BYTES * 4 }>::new();
    let mut buf = [0u8; 1];

    loop {
        match report_pipe_rx.read(&mut buf).await {
            // Read single byte
            1 => {
                let byte = buf[0];
                // COBS Delimiter
                if byte == 0 {
                    debug!(
                        "FRAMING - frame_and_serialise_reports: COBS delimiter detected, attempting to frame: {:?}",
                        framing_buf
                    );

                    // COBS delimiter byte: process frame
                    match messenger_mouse::deserialize_report(&mut framing_buf) {
                        Ok(framed_report) => {
                            info!(
                                "FRAMING - frame_and_serialise_reports: COBS delimeter detected & Deserialise succes: {:?}",
                                framed_report
                            );
                            // Happy path - Send deserialised setpoint to control task
                            report_sender.send(framed_report);
                        }
                        Err(err) => {
                            error!(
                                "FRAMING - frame_and_serialise_reports: Unable to deserialise framing buffer into a report. Err: {} - buffer: {:?}",
                                err, framing_buf
                            );
                        }
                    }
                    // Reset current frame
                    framing_buf.clear();
                } else {
                    trace!("FRAMING - frame_setpoints: data byte: {}", byte);
                    // Data byte: add to frame
                    if let Err(byte) = framing_buf.push(byte) {
                        error!(
                            "FRAMING - frame_setpoints: Unable to collect byte {} because framing buffer {:?} is full, should never happen but you are here anyway",
                            byte, framing_buf
                        );
                        // Clear frame, issue is hopefully resolved after next delimiter byte
                        framing_buf.clear();
                    }
                }
            }
            // Unhappy path
            n => {
                error!(
                    "frame_and_serialise_reports: read {} bytes from pipe, ignoring",
                    n
                );
                framing_buf.clear();
            }
        };
    }
}

#[embassy_executor::task]
/// Deserialise the [`Report`]s collected from the control task into a UART byte stream to be
/// picked up by the comms task
pub async fn serialise_setpoints(
    mut setpoint_receiver: watch::Receiver<'static, Cs, Setpoint, 1>,
    setpoint_pipe_tx: pipe::Writer<'static, Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>,
) {
    let mut buf = [0u8; messenger_mouse::SETPOINT_BYTES * 2];
    loop {
        // Get latest setpoint from the control task
        let setpoint = setpoint_receiver.changed().await;

        // Serialize it
        match messenger_mouse::serialize_setpoint(setpoint.clone(), &mut buf) {
            Ok(mut serialised) => {
                // Push serialised report into pipe for consumption in comms task
                debug!(
                    "FRAMING - serialize_setpoint: serialised setpoint: {:?}",
                    serialised
                );
                // Write until full report is pushed into pipe
                while !serialised.is_empty() {
                    let n = setpoint_pipe_tx.write(serialised).await;

                    if n == 0 {
                        // Pipe is full, yield until space is available
                        Timer::after(embassy_time::Duration::from_millis(1)).await;
                        continue;
                    }

                    serialised = &mut serialised[n..];
                }
            }
            Err(err) => {
                error!(
                    "FRAMING - serialize_setpoints: {} - Unable to serialise setpoint {:?}, skipping...",
                    err, setpoint
                );
            }
        }
    }
}
