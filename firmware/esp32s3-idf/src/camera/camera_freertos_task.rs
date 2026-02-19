use std::time::{SystemTime, UNIX_EPOCH};

use crate::camera::{
    esp_cam_wrapper::Camera, framebuffer::FrameBuffer, framesize::FrameSize,
    peripherals::CameraPeripherals, pixelformat::PixelFormat,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, signal::*, watch::Watch};

use esp_idf_sys::*;
use log::*;

pub const PIXEL_FORMAT: PixelFormat = PixelFormat::GRAYSCALE;
// pub const PIXEL_FORMAT: PixelFormat = PixelFormat::JPEG;
pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeQvga;
pub const FRAMEBUFFER_LEN: usize = FRAME_SIZE.get_dimensions().0 * FRAME_SIZE.get_dimensions().1;
pub const XCLK_FREQ: i32 = 16_000_000;
pub const JPEG_QUALITY: i32 = 20;
pub const CAM_HZ: u64 = 10;

pub static FRAMEBUFFER_WEBSERVER_CHANNEL: Watch<Cs, FrameBuffer, 1> = Watch::new();
pub static FRAMEBUFFER_SD_CHANNEL: Watch<Cs, FrameBuffer, 1> = Watch::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    webserver_signal: &'static Watch<Cs, FrameBuffer, 1>,
    sd_signal: &'static Watch<Cs, FrameBuffer, 1>,
}

pub fn setup_freertos(camera_peripherals: CameraPeripherals) {
    info!("Setting up Camera FreeRtos task");

    unsafe {
        CAMERA_TASK_ARGS = Some(CameraTaskArgs {
            camera_peripherals: Some(camera_peripherals),
            webserver_signal: &FRAMEBUFFER_WEBSERVER_CHANNEL,
            sd_signal: &FRAMEBUFFER_SD_CHANNEL,
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

    let mut x_last_wake_time = xTaskGetTickCount();
    let mut last_time = SystemTime::now();

    // Get our camera args
    let args = &mut *(arg as *mut CameraTaskArgs);

    let webserver_signal = args.webserver_signal;
    let sd_signal = args.sd_signal;

    let camera_peripherals = args
        .camera_peripherals
        .take()
        .expect("Camera peripherals already taken");

    log::info!("Initialising framebuffer sender");

    // Init camera
    // Note: Camera is !Send !Sync, although this is not captured in the C++ driver
    // -> init & usage should be done on the same freertos task!
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
        1, // Large effect on driver behavior: When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    ) {
        Ok(cam) => cam,
        Err(err) => {
            log::error!("Issue initialising camera: {err}");
            return;
        }
    };

    log::info!("camera initialised");

    loop {
        x_last_wake_time = xTaskGetTickCount();

        // Take picture!
        if let Some(frame) = cam.get_framebuffer() {
            // Figure out FPS
            let now = SystemTime::now();
            let time_since_last_fb = now.duration_since(last_time).unwrap_or_default();
            last_time = now;

            // Convert the duration to seconds as an f64
            let secs = time_since_last_fb.as_secs_f64();

            // Guard against a zero‑length interval
            let frequency_hz = if secs > 0.0 {
                1.0 / secs
            } else {
                f64::INFINITY
            };

            log::info!(
                "Camera got {}x{} framebuffer gen {} @ {:p}\nFPS: {:.3}\n\n",
                frame.width(),
                frame.height(),
                frame.generation,
                &frame.data(),
                frequency_hz,
            );

            #[cfg(feature = "webserver")]
            {
                // Send the copied frame buffer to embassy context
                log::info!("Starting FB copy for Webserver usage");
                if let Some(fb_copy) = FrameBuffer::try_from_esp(&frame) {
                    webserver_signal.sender().send(fb_copy);
                } else {
                    log::warn!("unable to make FB copy for webserver usage");
                }
                log::info!("Finished FB copy");
            }

            #[cfg(feature = "sd")]
            {
                // Throttle logging
                if frame.generation % CAM_HZ == 0 {
                    log::info!("Starting FB(gen-{}) copy for SD logging", frame.generation);

                    // Copy framebuffer, continue if this fails
                    let Some(fb_owned) = FrameBuffer::try_from_esp(&frame) else {
                        // Failed to alloc
                        log::error!("unable to make FB copy for SD usage");
                        continue;
                    };
                    log::info!("Finished FB copy");

                    // Send to SD logging task
                    sd_signal.sender().send(fb_owned);
                }
            }

            // Note `frame` is dropped here, which releases the esp FB back to the esp32-camera
        };

        // 5 Hz
        // vTaskDelay(configTICK_RATE_HZ / CAM_HZ as u32);
        let last_wake_ptr: *mut u32 = &mut x_last_wake_time as *mut u32;
        xTaskDelayUntil(last_wake_ptr, configTICK_RATE_HZ / CAM_HZ as u32);
    }
}
