use embassy_executor::Spawner;
use embassy_sync::watch::{Receiver, Sender};
use esp_idf_hal::{
    gpio,
    sd::{
        mmc::{SdMmcHostConfiguration, SdMmcHostDriver},
        SdCardConfiguration, SdCardDriver,
    },
};
use esp_idf_hal::{gpio::*, sd::mmc::SDMMC1};
use log::*;
use std::{
    fs::{read_dir, File},
    time::SystemTime,
};
use std::{
    io::{Read, Seek, Write},
    time::UNIX_EPOCH,
};

use esp_idf_svc::fs::fatfs::Fatfs;
use esp_idf_svc::io::vfs::MountedFatfs;

use crate::{
    camera::{
        camera_freertos_task::{FRAMEBUFFER_SD_CHANNEL, PIXEL_FORMAT},
        pixelformat::PixelFormat,
    },
    sd::periperhals::SDPeripherals,
};

pub struct SdLogTaskPeripherals {
    sd_peripherals: Option<SDPeripherals>,
}

pub fn run(spawner: &Spawner, sd_peri: SDPeripherals) -> Result<(), embassy_executor::SpawnError> {
    log::info!("initialising SD card driver");
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

    loop {
        if let Some(frame) = FRAMEBUFFER_SD_CHANNEL.try_take() {
            log::info!(
                "SD log: received {}x{} framebuffer -> logging to SD",
                frame.width,
                frame.height
            );

            // let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
            let gen = frame.generation;
            let width = frame.width;
            let height = frame.height;
            let extension = match PIXEL_FORMAT {
                PixelFormat::GRAYSCALE => "pgm",
                PixelFormat::JPEG => "jpeg",
                _ => "unkown",
            };
            let filename = format!("/sdcard/{gen:?}.{extension}");

            info!("Attempting to create {filename}");
            let mut file = File::create(filename.clone())
                .expect(format!("Unable to create file {filename}").as_str());
            info!("File {file:?} created");

            // Write PGM header for grayscale pixelformat
            if PIXEL_FORMAT == PixelFormat::GRAYSCALE {
                let pgm_header = format!("P5\n{} {}\n255\n", width, height);
                file.write_all(pgm_header.as_bytes()).expect("Write failed");
            }

            file.write_all(&frame.data).expect("Write failed");
            info!("File {file:?} written");
        }
    }
}
