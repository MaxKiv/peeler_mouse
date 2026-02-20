pub mod blinky;
pub mod camera;
pub mod comms;
pub mod control;
pub mod request;
pub mod sd;
pub mod server;
pub mod wifi;

use crate::camera::peripherals::CameraPeripherals;
use crate::comms::periperhals::CommsPeripherals;
use crate::control::control_loop::setup::ControlPeripherals;
use anyhow::Result;
use embassy_executor::Spawner;
use esp_idf_hal::cpu::Core;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::task::watchdog::{TWDTConfig, TWDTDriver};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};
use std::ffi::CStr;

#[cfg(feature = "sd")]
use crate::sd::periperhals::SDPeripherals;

#[cfg(feature = "webserver")]
use crate::wifi::WIFI_STATE;

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

    let comms_peri = CommsPeripherals {
        uart: peripherals.uart0,
        tx: peripherals.pins.gpio43,
        rx: peripherals.pins.gpio44,
    };

    camera::camera_freertos_task::setup_freertos(camera_peripherals);
    comms::comms_task::run(spawner, comms_peri)?;

    log::info!("Initialize Controller task");

    let control_peri = ControlPeripherals {
        led_timer: peripherals.ledc.timer0,
        led_ch: peripherals.ledc.channel0,
        led_pin: peripherals.pins.gpio2,
        motor_timer: peripherals.ledc.timer1,
        motor_ch_a: peripherals.ledc.channel1,
        motor_pin_a: peripherals.pins.gpio47,
        motor_ch_b: peripherals.ledc.channel2,
        motor_pin_b: peripherals.pins.gpio21,
    };

    // Attempt to start up the control loop + dependencies
    control::control_loop::setup::run(spawner, control_peri)?;

    // Spawn auxilary SD writing task, when enabled
    #[cfg(feature = "sd")]
    {
        let sd_peri = SDPeripherals {
            slot: peripherals.sdmmc1,
            cmd: peripherals.pins.gpio38,
            clk: peripherals.pins.gpio39,
            d0: peripherals.pins.gpio40,
        };

        if let Err(err) = sd::save_image_task::run(spawner, sd_peri) {
            log::error!("Unable to Initialize SD write task: {err}");
        }
    }

    // Spawn auxilary Webserver task and wifi stack, when enabled
    // #[cfg(feature = "Webserver")]
    {
        log::info!("Initialize Wifi task");
        if let Err(err) = spawner.spawn(wifi::wifi_task(
            peripherals.modem,
            sys_loop,
            nvs,
            timer_service,
            WIFI_STATE.sender(),
        )) {
            log::error!("Unable to Initialize Wifi task: {err}");
        }

        log::info!("Initialize Webserver task");
        if let Err(err) = spawner.spawn(server::server_task(
            WIFI_STATE
                .receiver()
                .expect("Max wifi_state receivers reached"),
        )) {
            log::error!("Unable to Initialize Webserver task: {err}");
        }
    }

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
