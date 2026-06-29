use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    pipe::{self, Pipe},
    watch::{self, Watch},
};
use esp_idf_hal::{
    gpio,
    io::asynch::Write,
    uart::{
        config, AsyncUartDriver, AsyncUartRxDriver, AsyncUartTxDriver, UartDriver, UartRxDriver,
        UartTxDriver,
    },
    units::Hertz,
};
use log::*;
use messenger_mouse::VisionAlgorithmOutput;

use crate::comms::periperhals::CommsPeripherals;

use static_cell::StaticCell;

static UART_DRIVER: StaticCell<AsyncUartDriver<'static, UartDriver<'static>>> = StaticCell::new();
static REPORT_PIPE: StaticCell<pipe::Pipe<Cs, { messenger_mouse::REPORT_BYTES * 4 }>> =
    StaticCell::new();
static SETPOINT_PIPE: StaticCell<pipe::Pipe<Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>> =
    StaticCell::new();
pub static REPORT_WATCH: Watch<Cs, messenger_mouse::Esp32Report, 1> = Watch::new();
pub static SETPOINT_WATCH: Watch<Cs, messenger_mouse::Esp32Setpoint, 2> = Watch::new();

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, comms_peri: CommsPeripherals) -> anyhow::Result<()> {
    log::info!("initialising Comms task");

    log::info!("Initialising Comms Async UART");

    let config = config::Config::new().baudrate(Hertz(115_200));

    log::info!("Config done");

    let uart = match AsyncUartDriver::new(
        comms_peri.uart,
        comms_peri.tx,
        comms_peri.rx,
        Option::<gpio::Gpio0>::None,
        Option::<gpio::Gpio1>::None,
        &config,
    ) {
        Ok(uart) => {
            log::info!("UART: OK");
            uart
        }
        Err(err) => {
            log::error!("UART: unable to construct uart driver: {}", err);
            panic!("UART: unable to construct uart driver: {}", err);
        }
    };
    let uart = UART_DRIVER.init(uart);

    let (tx, rx) = uart.split();

    let report_pipe = REPORT_PIPE.init(Pipe::new());
    let (report_pipe_rx, report_pipe_tx) = report_pipe.split();
    let setpoint_pipe = SETPOINT_PIPE.init(Pipe::new());
    let (setpoint_pipe_rx, setpoint_pipe_tx) = setpoint_pipe.split();

    spawner.spawn(rx_task(rx, setpoint_pipe_tx))?;
    spawner.spawn(deserialise_task(setpoint_pipe_rx, SETPOINT_WATCH.sender()))?;

    spawner.spawn(tx_task(tx, report_pipe_rx))?;
    spawner.spawn(serialise_task(
        report_pipe_tx,
        REPORT_WATCH.receiver().expect("increase REPORT_WATCH N"),
    ))?;

    Ok(())
}

// UART RX COMMS task, receives serialised setpoints over the wire
#[embassy_executor::task]
pub async fn rx_task(
    rx: AsyncUartRxDriver<'static, UartRxDriver<'static>>,
    mut setpoint_pipe_tx: pipe::Writer<'static, Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>,
) {
    info!("COMMS: Starting RX task");
    let mut buf = [0u8; messenger_mouse::SETPOINT_BYTES * 4];

    info!("COMMS: Starting RX comms loop");
    loop {
        match rx.read(&mut buf).await {
            Ok(n) => {
                // info!("read {} bytes: {:?}", n, &buf[..n]);
                // Read N bytes, send along for framing
                if let Err(err) = setpoint_pipe_tx.write_all(&buf[..n]).await {
                    error!("COMMS: RX error: {err}");
                }
            }
            Err(err) => error!("COMMS: RX error: {err}"),
        }
    }
}

// UART TX COMMS task, sends latest serialised reports over wire
#[embassy_executor::task]
pub async fn tx_task(
    mut tx: AsyncUartTxDriver<'static, UartTxDriver<'static>>,
    report_pipe_rx: pipe::Reader<'static, Cs, { messenger_mouse::REPORT_BYTES * 4 }>,
) {
    info!("COMMS: Starting TX task");
    let mut buf = [0u8; 64];

    loop {
        // Receive serialised report bytes
        let x = report_pipe_rx.read(&mut buf).await;
        // Send them across the wire
        if let Err(err) = tx.write_all(&buf[..x]).await {
            error!("COMMS: TX error: {err}");
        }
    }
}

// UART TX FRAMING task, serialises latest report to send to stm32
#[embassy_executor::task]
pub async fn serialise_task(
    mut report_pipe_tx: pipe::Writer<'static, Cs, { messenger_mouse::REPORT_BYTES * 4 }>,
    mut report_receiver: watch::Receiver<'static, Cs, messenger_mouse::Esp32Report, 1>,
) {
    info!("FRAMING: Starting serialize task");
    let mut buf = [0u8; messenger_mouse::REPORT_BYTES];

    loop {
        let report = report_receiver.changed().await;

        match messenger_mouse::serialize_report(report, &mut buf) {
            Ok(serialised) => {
                // Only send the bytes that were actually written
                if let Err(err) = report_pipe_tx.write_all(serialised).await {
                    error!("FRAMING: pipe write error: {err}");
                }
            }
            Err(err) => {
                error!("FRAMING: serialisation error: {err}");
            }
        }
    }
}

// UART RX FRAMING task, deserialises received setpoints for consumption in this firmware
#[embassy_executor::task]
pub async fn deserialise_task(
    setpoint_pipe_rx: pipe::Reader<'static, Cs, { messenger_mouse::SETPOINT_BYTES * 4 }>,
    setpoint_sender: watch::Sender<'static, Cs, messenger_mouse::Esp32Setpoint, 2>,
) {
    info!("COMMS: Starting deserialise task");
    let mut framing_buf = heapless::Vec::<u8, { messenger_mouse::SETPOINT_BYTES * 2 }>::new();
    let mut buf = [0u8; messenger_mouse::SETPOINT_BYTES / 2];

    loop {
        let n = setpoint_pipe_rx.read(&mut buf).await;

        for &byte in &buf[..n] {
            if byte == 0 {
                if let Ok(setpoint) = messenger_mouse::deserialize_setpoint(&mut framing_buf) {
                    setpoint_sender.send(setpoint);
                }

                framing_buf.clear();
            } else if framing_buf.push(byte).is_err() {
                framing_buf.clear();
            }
        }
    }
}
