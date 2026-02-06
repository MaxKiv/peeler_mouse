pub mod blinky;
pub mod control;
pub mod espcam;
pub mod request;
mod server;
pub mod wifi;

use std::{ffi::CStr, sync::Arc};

use anyhow::Result;
use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_idf_hal::{
    ledc::{config::TimerConfig, LedcTimerDriver, TIMER0},
    prelude::Peripherals,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};
use log::info;

use crate::{control::setpoint::Setpoint, espcam::Camera, wifi::WifiState};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};

static WIFI_STATE: Watch<Cs, WifiState, 1> = Watch::new();
static SETPOINT: Watch<Cs, Setpoint, 1> = Watch::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    if let Err(err) = main_fallible(&spawner).await {
        log::error!("FATAL ERROR IN MAIN: {err}");
    }
}

async fn main_fallible(spawner: &Spawner) -> Result<()> {
    let _ = spawner;
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let version = unsafe { esp_idf_sys::esp_get_idf_version() };
    let version = unsafe { CStr::from_ptr(version) };
    let version = version.to_str()?;
    log::info!("ESP-IDF version: {version}");

    log::info!("Setting up peripherals, esp event loop, nvs partition and timer service");
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = EspTaskTimerService::new()?;

    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default())
        .expect("Unable to construct LedcTimerDriver");

    log::info!("Initialize LED task");
    spawner.spawn(blinky::blink_led(
        timer,
        peripherals.ledc.channel0,
        peripherals.pins.gpio2,
    ))?;

    log::info!("Setting up camera");
    let camera = Camera::new(
        peripherals.pins.gpio15,
        peripherals.pins.gpio11,
        peripherals.pins.gpio9,
        peripherals.pins.gpio8,
        peripherals.pins.gpio10,
        peripherals.pins.gpio12,
        peripherals.pins.gpio18,
        peripherals.pins.gpio17,
        peripherals.pins.gpio16,
        peripherals.pins.gpio6,
        peripherals.pins.gpio7,
        peripherals.pins.gpio13,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
        esp_idf_sys::camera::pixformat_t_PIXFORMAT_GRAYSCALE,
        // Set quality here
        // esp_idf_sys::camera::framesize_t_FRAMESIZE_QVGA,
        esp_idf_sys::camera::framesize_t_FRAMESIZE_QQVGA,
        20_000_000,
        32,
        1,
    )?;

    let cam_arc = Arc::new(camera);

    log::info!("Initialize Wifi task");
    spawner.spawn(wifi::wifi_task(
        peripherals.modem,
        sys_loop,
        nvs,
        timer_service,
        WIFI_STATE.sender(),
    ))?;

    log::info!("Initialize Webserver task");
    spawner.spawn(server::server_task(
        cam_arc.clone(),
        WIFI_STATE
            .receiver()
            .expect("Max wifi_state receivers reached"),
        SETPOINT.sender(),
    ))?;

    // log::info!("Initialize Controller task");
    // spawner.spawn(control::controller::control_loop(
    //     SETPOINT.receiver().expect("Max setpoint receivers reached"),
    //     timer,
    //     peripherals.ledc.channel1,
    //     peripherals.pins.gpio12,
    //     peripherals.ledc.channel2,
    //     peripherals.pins.gpio13,
    // ))?;

    // let mut ticker = Ticker::every(Duration::from_hz(1));
    // let camera = cam_arc.clone();
    // loop {
    //     if let Some(fb) = camera.get_framebuffer() {
    //         info!(
    //             "{:?} => Got {} bytes [{} x {}] {} frame buffer",
    //             fb.timestamp(),
    //             fb.len(),
    //             fb.width(),
    //             fb.height(),
    //             fb.format()
    //         );
    //         let data = &fb.data()[..fb.width() / 10];
    //         info!("data: {:?}", data);
    //
    //         ticker.next().await;
    //     }
    // }

    core::future::pending::<()>().await;

    Ok(())
}
