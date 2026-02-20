use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use esp_idf_hal::{
    gpio,
    sd::{
        mmc::{SdMmcHostConfiguration, SdMmcHostDriver},
        SdCardConfiguration, SdCardDriver,
    },
};
use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::io::vfs::MountedFatfs;

use log::*;
use std::{
    fs::{read_dir, File},
    time::SystemTime,
};
use std::{
    io::Write,
    time::UNIX_EPOCH,
};

use crate::{
    camera::{
        camera_freertos_task::{FRAMEBUFFER_SD_CHANNEL, PIXEL_FORMAT},
        framebuffer::FrameBuffer,
        pixelformat::PixelFormat,
    },
    sd::periperhals::SDPeripherals,
};

const SD_WRITE_FREQUENCY: Duration = Duration::from_hz(1);

pub fn run(spawner: &Spawner, sd_peri: SDPeripherals) -> anyhow::Result<()> {
    log::info!("initialising SD card driver");
    let mut cfg = SdMmcHostConfiguration::new();
    // It seems the esp32s3-cam has external pullups, but who really knows without a datasheet?
    cfg.enable_internal_pullups = false;

    // SDMMC Data width = 1 bit
    let sd_mmc_host_driver = SdMmcHostDriver::new_1bit(
        sd_peri.slot,
        sd_peri.cmd,
        sd_peri.clk,
        sd_peri.d0,
        None::<gpio::AnyIOPin>,
        None::<gpio::AnyIOPin>,
        &cfg,
    )?;

    let sd_card_driver = SdCardDriver::new_mmc(sd_mmc_host_driver, &SdCardConfiguration::new())?;

    log::info!("SD card driver initialised");

    spawner.spawn(log_to_sd_task(sd_card_driver))?;

    Ok(())
}

#[embassy_executor::task]
pub async fn log_to_sd_task(sd_card_driver: SdCardDriver<SdMmcHostDriver<'static>>) {
    log::info!("Starting SD card logging Embassy task");

    let _mounted_fatfs = MountedFatfs::mount(
        Fatfs::new_sdcard(0, sd_card_driver)
            .expect("unable to construct FAT filesystem instance on SD card"),
        "/sdcard",
        4,
    )
    .expect("unable to mount /sdcard as FatFS");

    {
        let directory = read_dir("/sdcard").expect("Unable to read /sdcard");

        for entry in directory {
            match entry {
                Ok(entry) => log::info!("Entry: {:?}", entry.file_name()),
                Err(e) => log::error!("Error reading /sdcard directory entry: {e}"),
            }
        }
    }

    let mut rx = FRAMEBUFFER_SD_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_SD_CHANNEL rx N");

    let mut ticker = Ticker::every(SD_WRITE_FREQUENCY);

    loop {
        // Wait for latest frame to arrive
        let frame = rx.changed().await;
        // Attempt to write it to the SD card

        if let Err(err) = try_save_frame(frame) {
            log::warn!("SD log: unable to save frame: {err}");
        }

        // Throttle writes to desired frequency
        ticker.next().await;
    }
}

fn try_save_frame(frame: FrameBuffer) -> std::io::Result<()> {
    log::info!(
        "SD log: received {}x{} framebuffer -> logging to SD",
        frame.width,
        frame.height
    );

    let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    let gen = frame.generation;
    let width = frame.width;
    let height = frame.height;
    let extension = match PIXEL_FORMAT {
        PixelFormat::GRAYSCALE => "pgm",
        PixelFormat::JPEG => "jpeg",
        _ => "unkown",
    };
    let filename = format!("/sdcard/frame_{gen}.{extension}");

    info!("Attempting to create {filename}");
    let mut file = File::create(filename)?;
    info!("File {file:?} created");

    // Write PGM header for grayscale pixelformat
    if PIXEL_FORMAT == PixelFormat::GRAYSCALE {
        let pgm_header = format!("P5\n{width} {height}\n255\n");
        file.write_all(pgm_header.as_bytes())?
    }

    file.write_all(&frame.data)?;
    info!("File {file:?} written");

    Ok(())
}
