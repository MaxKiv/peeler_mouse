#![feature(let_chains)]

pub mod actuation;
pub mod camera;
pub mod comms;
pub mod control;
pub mod encoder;
pub mod request;
pub mod sd;
pub mod server;
pub mod util;
pub mod wifi;

use crate::actuation::stepper::peripherals::{MotorPeripherals, StepperPeripherals};
use crate::camera::peripherals::CameraPeripherals;
use crate::comms::periperhals::CommsPeripherals;
use crate::control::control_loop::peripherals::ControlPeripherals;
use crate::encoder::peripherals::EncoderPeripherals;
use anyhow::Result;
use embassy_executor::Spawner;
use esp_idf_hal::cpu::Core;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::task::watchdog::{TWDTConfig, TWDTDriver};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};
use std::ffi::CStr;
use std::future;

#[cfg(feature = "sd")]
use crate::sd::periperhals::SDPeripherals;

#[cfg(feature = "webserver")]
use crate::wifi::WIFI_STATE;

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
    log::set_max_level(log::LevelFilter::Trace);
    log::set_max_level(log::LevelFilter::Error);

    let version = unsafe { esp_idf_sys::esp_get_idf_version() };
    let version = unsafe { CStr::from_ptr(version) };
    let version = version.to_str()?;
    log::info!("ESP-IDF version: {version}");

    log::info!("Setting up peripherals, esp event loop, nvs partition, timer service and disabling watchdog");
    // Note: these live on the stack forever since main_fallible diverges, this is intended
    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = EspTaskTimerService::new()?;
    let watchdog = disable_watchdog(peripherals.twdt)?;
    log::info!("Watchdog disabled");

    log::info!("Initialize Camera freertos task");
    #[cfg(feature = "devkit")]
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

    #[cfg(feature = "pcb")]
    let camera_peripherals = CameraPeripherals {
        pin_xclk: peripherals.pins.gpio10,
        pin_d0: peripherals.pins.gpio15,
        pin_d1: peripherals.pins.gpio17,
        pin_d2: peripherals.pins.gpio18,
        pin_d3: peripherals.pins.gpio16,
        pin_d4: peripherals.pins.gpio14,
        pin_d5: peripherals.pins.gpio12,
        pin_d6: peripherals.pins.gpio11,
        pin_d7: peripherals.pins.gpio48,
        pin_vsync: peripherals.pins.gpio38,
        pin_href: peripherals.pins.gpio47,
        pin_pclk: peripherals.pins.gpio13,
        pin_sda: peripherals.pins.gpio40,
        pin_scl: peripherals.pins.gpio39,
    };
    camera::camera_freertos_task::setup_freertos(camera_peripherals);

    #[cfg(feature = "devkit")]
    let comms_peri = CommsPeripherals {
        uart: peripherals.uart2,
        tx: peripherals.pins.gpio19,
        rx: peripherals.pins.gpio20,
    };
    #[cfg(feature = "pcb")]
    let comms_peri = CommsPeripherals {
        uart: peripherals.uart2,
        tx: peripherals.pins.gpio43,
        rx: peripherals.pins.gpio44,
    };

    comms::comms_task::run(spawner, comms_peri)?;

    // Spawn auxilary SD writing task, when enabled
    #[cfg(all(feature = "sd", feature = "devkit"))]
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
    #[cfg(all(feature = "webserver", feature = "devkit"))]
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

    #[cfg(feature = "devkit")]
    let encoder_peri = EncoderPeripherals {
        spi: peripherals.spi3,
        sclk: peripherals.pins.gpio1,
        serial_out: peripherals.pins.gpio2,
        serial_in: peripherals.pins.gpio21,
        cs: peripherals.pins.gpio47,
    };
    #[cfg(feature = "pcb")]
    let encoder_peri = EncoderPeripherals {
        spi: peripherals.spi3,
        sclk: peripherals.pins.gpio7,
        serial_out: peripherals.pins.gpio9,
        serial_in: peripherals.pins.gpio8,
        cs: peripherals.pins.gpio6,
    };

    log::info!("Initialize encoder task");
    // Attempt to start up the control loop + dependencies
    encoder::encoder_task::run(spawner, encoder_peri)?;

    #[cfg(feature = "devkit")]
    let stepper_peri = StepperPeripherals {
        timer: peripherals.ledc.timer1,
        rmt_channel: peripherals.rmt.channel0,
        step_rmt_pin: peripherals.pins.gpio41,
        dir_pin: peripherals.pins.gpio42,
        enable_pin: peripherals.pins.gpio40,
    };
    #[cfg(feature = "devkit")]
    let motor_peri = MotorPeripherals {
        stepper: stepper_peri,
        limit_switch: peripherals.pins.gpio45,
    };
    #[cfg(feature = "pcb")]
    let stepper_peri = StepperPeripherals {
        timer: peripherals.ledc.timer1,
        rmt_channel: peripherals.rmt.channel0,
        step_rmt_pin: peripherals.pins.gpio3,
        dir_pin: peripherals.pins.gpio2,
        enable_pin: peripherals.pins.gpio4,
    };
    #[cfg(feature = "pcb")]
    let motor_peri = MotorPeripherals {
        stepper: stepper_peri,
        limit_switch: peripherals.pins.gpio1,
    };

    actuation::stepper::run(spawner, motor_peri)?;

    #[cfg(feature = "devkit")]
    let control_peri = ControlPeripherals {
        led_timer: peripherals.ledc.timer0,
        led_ch: peripherals.ledc.channel0,
        led_pin: peripherals.pins.gpio48,
    };
    #[cfg(feature = "pcb")]
    let control_peri = ControlPeripherals {
        led_timer: peripherals.ledc.timer0,
        led_ch: peripherals.ledc.channel0,
        led_pin: peripherals.pins.gpio5,
    };

    #[cfg(not(feature = "streaming"))]
    {
        log::info!("Initialize control task");
        // Attempt to start up the control loop + dependencies
        control::control_loop::setup::run(spawner, control_peri)?;
    }

    future::pending::<()>().await;

    Ok(())
}

// Quick! Nobody is watching
fn disable_watchdog(
    twdt: esp_idf_hal::task::watchdog::TWDT,
) -> Result<TWDTDriver<'static>, esp_idf_sys::EspError> {
    let config = TWDTConfig {
        duration: std::time::Duration::MAX,
        panic_on_trigger: true,
        subscribed_idle_tasks: Core::Core0.into(),
    };
    let driver = esp_idf_hal::task::watchdog::TWDTDriver::new(twdt, &config)?;
    // Return the WD
    Ok(driver)
}
