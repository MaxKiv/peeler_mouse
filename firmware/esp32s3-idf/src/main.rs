pub mod blinky;
pub mod camera;
pub mod control;
pub mod request;
mod server;
pub mod wifi;

use std::ffi::CStr;
use std::fs::{read_dir, File};
use std::io::{Read, Seek, Write};

use anyhow::Result;
use embassy_executor::Spawner;
use esp_idf_hal::{
    gpio,
    ledc::{config::TimerConfig, LedcTimerDriver},
    prelude::Peripherals,
};
use esp_idf_svc::io::vfs::MountedFatfs;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, timer::EspTaskTimerService,
};
use esp_idf_svc::{
    fs::fatfs::Fatfs,
    hal::sd::{
        mmc::{SdMmcHostConfiguration, SdMmcHostDriver},
        SdCardConfiguration, SdCardDriver,
    },
};
use log::info;

use crate::{
    camera::{camera_freertos_task::CAMERA_FRAMEBUFFER, peripherals::CameraPeripherals},
    control::setpoint::Setpoint,
    wifi::WifiState,
};
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

    log::info!("Initialize Camera freertos task");

    camera::camera_freertos_task::setup_freertos(camera_peripherals);

    let mut cfg = SdMmcHostConfiguration::new();
    // It seems the esp32s3-cam has external pullups, but who really knows without a datasheet?
    cfg.enable_internal_pullups = false;
    let sd_card_driver = SdCardDriver::new_mmc(
        // => Data width = 1 bit
        SdMmcHostDriver::new_1bit(
            peripherals.sdmmc1,
            peripherals.pins.gpio38,
            peripherals.pins.gpio39,
            peripherals.pins.gpio40,
            None::<gpio::AnyIOPin>,
            None::<gpio::AnyIOPin>,
            &cfg,
        )?,
        &SdCardConfiguration::new(),
    )?;

    let _mounted_fatfs = MountedFatfs::mount(Fatfs::new_sdcard(0, sd_card_driver)?, "/sdcard", 4)?;

    let content = b"Hello, world!";

    {
        let mut file = File::create("/sdcard/test.txt")?;

        info!("File {file:?} created");

        file.write_all(content).expect("Write failed");

        info!("File {file:?} written with {content:?}");

        file.seek(std::io::SeekFrom::Start(0)).expect("Seek failed");

        info!("File {file:?} seeked");
    }

    {
        let mut file = File::open("/sdcard/test.txt")?;

        info!("File {file:?} opened");

        let mut file_content = String::new();

        file.read_to_string(&mut file_content).expect("Read failed");

        info!("File {file:?} read: {file_content}");

        assert_eq!(file_content.as_bytes(), content);
    }

    {
        let directory = read_dir("/sdcard")?;

        for entry in directory {
            log::info!("Entry: {:?}", entry?.file_name());
        }
    }

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

    core::future::pending::<()>().await;

    Ok(())
}
