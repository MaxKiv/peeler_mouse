pub mod blinky;
pub mod control;
pub mod espcam;
pub mod request;
mod server;
pub mod wifi;

use std::{ffi::CStr, sync::Arc};

use anyhow::Result;
use embassy_executor::Spawner;
use esp_idf_hal::{
    gpio::{AnyOutputPin, PinDriver},
    prelude::Peripherals,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};
use serde::Deserialize;

use crate::{espcam::Camera, wifi::WifiState};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};

static WIFI_STATE: Watch<Cs, WifiState, 1> = Watch::new();
static SETPOINT: Watch<Cs, Setpoint, 1> = Watch::new();

#[derive(Deserialize, Copy, Clone, Debug)]
struct Setpoint {
    depth: f32,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    if let Err(err) = main_fallible(&spawner).await {
        log::error!("MAIN: {err}");
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

    log::info!("Initialize LED task");
    let led = PinDriver::output(AnyOutputPin::from(peripherals.pins.gpio33))?;
    spawner.spawn(blinky::blink_led(led))?;

    log::info!("Initialize Wifi task");
    spawner.spawn(wifi::wifi_task(
        peripherals.modem,
        sys_loop,
        nvs,
        timer_service,
        WIFI_STATE.sender(),
    ))?;

    log::info!("Setting up camera");
    let camera = Camera::new(
        peripherals.pins.gpio32,
        peripherals.pins.gpio0,
        peripherals.pins.gpio5,
        peripherals.pins.gpio18,
        peripherals.pins.gpio19,
        peripherals.pins.gpio21,
        peripherals.pins.gpio36,
        peripherals.pins.gpio39,
        peripherals.pins.gpio34,
        peripherals.pins.gpio35,
        peripherals.pins.gpio25,
        peripherals.pins.gpio23,
        peripherals.pins.gpio22,
        peripherals.pins.gpio26,
        peripherals.pins.gpio27,
        esp_idf_sys::camera::pixformat_t_PIXFORMAT_JPEG,
        // Set quality here
        esp_idf_sys::camera::framesize_t_FRAMESIZE_SVGA,
    )?;
    let cam_arc = Arc::new(camera);

    log::info!("Initialize Webserver task");
    spawner.spawn(server::server_task(
        cam_arc,
        WIFI_STATE
            .receiver()
            .expect("Max wifi_state receivers reached"),
        SETPOINT.sender(),
    ))?;

    log::info!("Initialize Controller task");
    spawner.spawn(control::control_loop(
        SETPOINT.receiver().expect("Max setpoint receivers reached"),
    ))?;

    core::future::pending::<()>().await;

    Ok(())
}
