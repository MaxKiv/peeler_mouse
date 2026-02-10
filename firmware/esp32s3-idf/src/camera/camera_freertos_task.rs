use crate::camera::{
    esp_cam_wrapper::{Camera, FrameBuffer},
    framesize::FrameSize,
    peripherals::{CameraPeripherals, SDPeripherals},
    pixelformat::PixelFormat,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, signal::*};

use esp_idf_hal::gpio;
use esp_idf_svc::io::vfs::MountedFatfs;
use esp_idf_svc::{
    fs::fatfs::Fatfs,
    hal::sd::{
        mmc::{SdMmcHostConfiguration, SdMmcHostDriver},
        SdCardConfiguration, SdCardDriver,
    },
};

use std::io::{Read, Seek, Write};
use std::{
    fs::{read_dir, File},
    time::{SystemTime, UNIX_EPOCH},
};

use esp_idf_sys::*;
use log::*;

pub const PIXEL_FORMAT: PixelFormat = PixelFormat::GRAYSCALE;
pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeVga;
pub const FRAMEBUFFER_LEN: usize = FRAME_SIZE.get_dimensions().0 * FRAME_SIZE.get_dimensions().1;
pub const XCLK_FREQ: i32 = 20_000_000;
pub const JPEG_QUALITY: i32 = 32;

pub static CAMERA_FRAMEBUFFER: Signal<Cs, Option<FrameBuffer>> = Signal::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    sd_peripherals: Option<SDPeripherals>,
    signal: &'static Signal<Cs, Option<FrameBuffer>>,
}

pub fn setup_freertos(camera_peripherals: CameraPeripherals, sd_peripherals: SDPeripherals) {
    info!("Setting up Camera FreeRtos task");

    unsafe {
        CAMERA_TASK_ARGS = Some(CameraTaskArgs {
            camera_peripherals: Some(camera_peripherals),
            sd_peripherals: Some(sd_peripherals),
            signal: &CAMERA_FRAMEBUFFER,
        });

        xTaskCreatePinnedToCore(
            Some(camera_task),
            b"camera\0".as_ptr(),
            4096,
            CAMERA_TASK_ARGS.as_mut().unwrap() as *mut _ as *mut _,
            5,
            core::ptr::null_mut(),
            0,
        );
    }
}

unsafe extern "C" fn camera_task(arg: *mut core::ffi::c_void) {
    log::info!("Starting camera FreeRtos task");

    // Get our camera args
    let args = &mut *(arg as *mut CameraTaskArgs);

    let signal = args.signal;

    let camera_peripherals = args
        .camera_peripherals
        .take()
        .expect("Camera peripherals already taken");

    let sd_peripherals = args
        .sd_peripherals
        .take()
        .expect("Sd peripherals already taken");

    log::info!("Initialising framebuffer sender");

    // Init camera
    // Note: init & usage should be done on the same freertos task!
    log::info!("Initialising camera");
    let mut cam = match Camera::new(
        camera_peripherals.pin_xclk,
        camera_peripherals.pin_d0,
        camera_peripherals.pin_d1,
        camera_peripherals.pin_d2,
        camera_peripherals.pin_d3,
        camera_peripherals.pin_d4,
        camera_peripherals.pin_d5,
        camera_peripherals.pin_d6,
        camera_peripherals.pin_d7,
        camera_peripherals.pin_vsync,
        camera_peripherals.pin_href,
        camera_peripherals.pin_pclk,
        camera_peripherals.pin_sda,
        camera_peripherals.pin_scl,
        PIXEL_FORMAT,
        FRAME_SIZE,
        XCLK_FREQ,
        JPEG_QUALITY,
        3,
    ) {
        Ok(cam) => cam,
        Err(err) => {
            log::error!("Issue initialising camera: {err}");
            return;
        }
    };

    log::info!("camera initialised");

    log::info!("initialising SD card driver");
    let mut cfg = SdMmcHostConfiguration::new();
    // It seems the esp32s3-cam has external pullups, but who really knows without a datasheet?
    cfg.enable_internal_pullups = false;
    let sd_card_driver = SdCardDriver::new_mmc(
        // => Data width = 1 bit
        SdMmcHostDriver::new_1bit(
            sd_peripherals.slot,
            sd_peripherals.cmd,
            sd_peripherals.clk,
            sd_peripherals.d0,
            None::<gpio::AnyIOPin>,
            None::<gpio::AnyIOPin>,
            &cfg,
        )
        .expect("unable to construct SdMmcHostDriver"),
        &SdCardConfiguration::new(),
    )
    .expect("Unable to construct SdCardDriver");

    log::info!("SD card driver initialised");

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
        // Take picture!
        if let Some(frame) = cam.get_framebuffer() {
            log::info!(
                "Camera got {}x{} framebuffer gen {} @ {:p}\n\n",
                frame.width(),
                frame.height(),
                frame.generation,
                &frame.data(),
            );

            {
                let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();

                let gen = frame.generation;
                let width = frame.width();
                let height = frame.height();
                let filename = format!("/sdcard/{gen:?}.pgm");
                info!("Attempting to create {filename}");

                // let mut file = File::create("/sdcard/test.txt")?;
                let mut file = File::create(filename.clone())
                    .expect(format!("Unable to create file {filename}").as_str());

                info!("File {file:?} created");
                let pgm_header = format!("P5\n{} {}\n255\n", width, height);
                file.write_all(pgm_header.as_bytes()).expect("Write failed");
                file.write_all(frame.data()).expect("Write failed");
                info!("File {file:?} written");
            }

            // Send frame buffer pointer to embassy context
            signal.signal(Some(frame));
        };

        // 1 Hz
        vTaskDelay(5 * configTICK_RATE_HZ);

        // Signal the imminent destruction of the framebuffer
        // This actually triggers the camera::esp_camera_fb_return(fb) through FrameBuffer::Drop,
        // which is required before fetching a new frame
        signal.signal(None);

        vTaskDelay(1 * configTICK_RATE_HZ);
    }
}
