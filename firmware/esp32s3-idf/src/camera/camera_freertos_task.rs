use std::time::{SystemTime, UNIX_EPOCH};

use crate::camera::{
    esp_cam_wrapper::Camera, framebuffer::FrameBuffer, framesize::FrameSize,
    peripherals::CameraPeripherals, pixelformat::PixelFormat, CameraConfig,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};

use esp_idf_sys::*;
use log::*;

pub static FRAMEBUFFER_CONTROL_LOOP_CHANNEL: Watch<Cs, FrameBuffer, 1> = Watch::new();
pub static FRAMEBUFFER_WEBSERVER_CHANNEL: Watch<Cs, FrameBuffer, 1> = Watch::new();
pub static FRAMEBUFFER_SD_CHANNEL: Watch<Cs, FrameBuffer, 1> = Watch::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    control_loop_signal: &'static Watch<Cs, FrameBuffer, 1>,
    webserver_signal: &'static Watch<Cs, FrameBuffer, 1>,
    sd_signal: &'static Watch<Cs, FrameBuffer, 1>,
}

pub fn setup_freertos(camera_peripherals: CameraPeripherals) {
    info!("Setting up Camera FreeRtos task");

    unsafe {
        CAMERA_TASK_ARGS = Some(CameraTaskArgs {
            camera_peripherals: Some(camera_peripherals),
            control_loop_signal: &FRAMEBUFFER_CONTROL_LOOP_CHANNEL,
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

    let control_loop_signal = args.control_loop_signal;
    let webserver_signal = args.webserver_signal;
    let sd_signal = args.sd_signal;

    let camera_peripherals = args
        .camera_peripherals
        .take()
        .expect("Camera peripherals already taken");

    log::info!("Initialising framebuffer sender");

    // Init camera
    let cfg = CameraConfig::new();

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
        cfg.pixel_format,
        cfg.frame_size,
        cfg.xclk_freq,
        cfg.jpeg_quality,
        cfg.fb_count, // Large effect on driver behavior: When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
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

            // Guard against a zero length interval
            let fps = if secs > 0.0 {
                1.0 / secs
            } else {
                f64::INFINITY
            };

            // Get timestamp in micros since boot
            let timestamp_us = unsafe { esp_timer_get_time() };

            log::info!(
                "Camera got {}x{} framebuffer gen {} @ {:p}\nFPS: {:.3}\n\n",
                frame.width(),
                frame.height(),
                frame.generation,
                &frame.data(),
                fps,
            );

            log::info!("Starting FB copy for control loop");
            let start = SystemTime::now();
            // Copy framebuffer, continue if this fails
            if let Some(fb_copy) = FrameBuffer::try_from_esp(&frame, fps, timestamp_us) {
                // Send to ControlLoop task
                control_loop_signal.sender().send(fb_copy);
            } else {
                log::error!("unable to make FB copy for control loop");
            }
            log::info!(
                "Finished FB copy for control loop in {}ms",
                SystemTime::now()
                    .duration_since(start)
                    .unwrap_or_default()
                    .as_millis(),
            );

            // If webserver is enabled, copy the framebuffer for consumption there
            // Note: each FB copy takes ~30ms, this directly impacts control loop perf
            #[cfg(feature = "webserver")]
            {
                log::info!("Starting FB copy for Webserver usage");
                let start = SystemTime::now();
                // Copy framebuffer, continue if this fails
                if let Some(fb_copy) = FrameBuffer::try_from_esp(&frame, fps, timestamp_us) {
                    // Send to Webserver task
                    webserver_signal.sender().send(fb_copy);
                } else {
                    log::warn!("unable to make FB copy for webserver usage");
                }
                log::info!(
                    "Finished FB copy for webserver in {}ms",
                    SystemTime::now()
                        .duration_since(start)
                        .unwrap_or_default()
                        .as_millis(),
                );
            }

            // If SD logging is enabled, copy the framebuffer for consumption there
            // Note: each FB copy takes ~30ms, this directly impacts control loop perf
            #[cfg(feature = "sd")]
            {
                // Throttle logging
                if frame.generation % CAMERA_TARGET_FPS == 0 {
                    let start = SystemTime::now();

                    log::info!("Starting FB copy for SD logging usage");
                    // Copy framebuffer, continue if this fails
                    if let Some(fb_copy) = FrameBuffer::try_from_esp(&frame, fps, timestamp_us) {
                        // Send to Sd logging task
                        sd_signal.sender().send(fb_copy);
                    } else {
                        log::warn!("unable to make FB copy for SD logging usage");
                    }
                    log::info!(
                        "Finished FB copy for SD logging in {}ms",
                        SystemTime::now()
                            .duration_since(start)
                            .unwrap_or_default()
                            .as_millis(),
                    );
                }
            }

            // Note `frame` is dropped here, which releases the esp FB back to the esp32-camera
        };

        // Timekeeping
        let last_wake_ptr: *mut u32 = &mut x_last_wake_time as _;
        xTaskDelayUntil(
            last_wake_ptr,
            configTICK_RATE_HZ / cfg.camera_target_fps as u32,
        );
    }
}
