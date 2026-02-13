pub mod blinky;
pub mod camera;
pub mod control;
pub mod request;
pub mod sd;
pub mod server;
pub mod wifi;

use esp_idf_hal::ledc::config::TimerConfig;
use std::ffi::CStr;

use anyhow::Result;
use embassy_executor::Spawner;
use esp_idf_hal::cpu::Core;
use esp_idf_hal::ledc::LedcTimerDriver;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::task::watchdog::{TWDTConfig, TWDTDriver};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};

use crate::sd::periperhals::SDPeripherals;
use crate::{camera::peripherals::CameraPeripherals, control::setpoint::Setpoint, wifi::WifiState};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};

static WIFI_STATE: Watch<Cs, WifiState, 1> = Watch::new();
static SETPOINT: Watch<Cs, Setpoint, 1> = Watch::new();
static mut WATCHDOG: Option<TWDTDriver> = None;

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

    log::info!("Disabling watchdog");
    disable_watchdog(peripherals.twdt);
    log::info!("Watchdog disabled");

    let timer = LedcTimerDriver::new(peripherals.ledc.timer0, &TimerConfig::default())
        .expect("Unable to construct LedcTimerDriver");

    log::info!("Initialize LED task");
    spawner.spawn(blinky::blink_led(
        timer,
        peripherals.ledc.channel0,
        peripherals.pins.gpio2,
    ))?;

    log::info!("Initialize Camera freertos task");

    let camera_peripherals = CameraPeripherals {
        pin_xclk: peripherals.pins.gpio15,
        pin_d0: peripherals.pins.gpio11,
        pin_d1: peripherals.pins.gpio9,
        pin_d2: peripherals.pins.gpio8,
        pin_d3: peripherals.pins.gpio10,
        pin_d4: peripherals.pins.gpio12,
        pin_d5: peripherals.pins.gpio18,
        pin_d6: peripherals.pins.gpio17,
        pin_d7: peripherals.pins.gpio16,
        pin_vsync: peripherals.pins.gpio6,
        pin_href: peripherals.pins.gpio7,
        pin_pclk: peripherals.pins.gpio13,
        pin_sda: peripherals.pins.gpio4,
        pin_scl: peripherals.pins.gpio5,
    };

    let sd_peri = SDPeripherals {
        slot: peripherals.sdmmc1,
        cmd: peripherals.pins.gpio38,
        clk: peripherals.pins.gpio39,
        d0: peripherals.pins.gpio40,
    };

    camera::camera_freertos_task::setup_freertos(camera_peripherals);

    sd::save_image_task::run(spawner, sd_peri)?;

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

    Ok(())
}

// Quick! Nobody is watching
fn disable_watchdog(twdt: esp_idf_hal::task::watchdog::TWDT) -> Result<(), esp_idf_sys::EspError> {
    let config = TWDTConfig {
        duration: std::time::Duration::MAX,
        panic_on_trigger: true,
        subscribed_idle_tasks: Core::Core0.into(),
    };
    let driver = esp_idf_hal::task::watchdog::TWDTDriver::new(twdt, &config)?;
    // Save the WD
    unsafe {
        WATCHDOG = Some(driver);
    }

    Ok(())
}
