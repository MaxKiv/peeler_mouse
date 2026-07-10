use std::sync::Arc;

use crate::{
    camera::{
        esp_cam_wrapper::Camera,
        framebuffer_view::{FrameBufferView, FRAME_DONE_SIGNAL},
        peripherals::CameraPeripherals,
        CameraConfig,
    },
    comms::comms_task::SETPOINT_WATCH,
};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    channel::Channel,
    watch::{self, Watch},
};

use embassy_time::Instant;
use esp_idf_sys::*;
use log::*;
use messenger_mouse::{motor::ControlMode, Esp32Setpoint};

pub static FRAMEBUFFER_CONTROL_LOOP_CHANNEL: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();
pub static FRAMEBUFFER_WEBSERVER_CHANNEL: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();
pub static FRAMEBUFFER_SD_CHANNEL: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    control_loop_tx: &'static Watch<Cs, Arc<FrameBufferView>, 1>,
    webserver_watch: &'static Watch<Cs, Arc<FrameBufferView>, 1>,
    sd_signal: &'static Watch<Cs, Arc<FrameBufferView>, 1>,
}

pub fn setup_freertos(camera_peripherals: CameraPeripherals) {
    info!("Setting up Camera FreeRtos task");

    unsafe {
        CAMERA_TASK_ARGS = Some(CameraTaskArgs {
            camera_peripherals: Some(camera_peripherals),
            control_loop_tx: &FRAMEBUFFER_CONTROL_LOOP_CHANNEL,
            webserver_watch: &FRAMEBUFFER_WEBSERVER_CHANNEL,
            sd_signal: &FRAMEBUFFER_SD_CHANNEL,
        });

        xTaskCreatePinnedToCore(
            Some(camera_task),
            b"camera\0".as_ptr(),
            4096,
            CAMERA_TASK_ARGS.as_mut().unwrap() as *mut _ as *mut _,
            24,
            core::ptr::null_mut(),
            1,
        );
    }
}

unsafe extern "C" fn camera_task(arg: *mut core::ffi::c_void) {
    log::info!("Starting camera FreeRtos task");

    let mut x_last_wake_time = xTaskGetTickCount();
    let mut last_time = unsafe { esp_timer_get_time() }; // Has microsecond granularity

    // Get our camera args
    let args = &mut *(arg as *mut CameraTaskArgs);

    let control_loop_tx = args.control_loop_tx.sender();
    let webserver_watch = args.webserver_watch.sender();
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
            let now_us = unsafe { esp_timer_get_time() }; // Has microsecond granularity
            let time_since_last_fb = now_us - last_time;
            last_time = now_us;

            // Guard against a zero length interval
            let fps = if time_since_last_fb > 0 {
                1_000_000.0 / (time_since_last_fb as f64)
            } else {
                f64::INFINITY
            };

            log::warn!(
                "Camera got {} [{}x{}] framebuffer gen {} @ {:p}\n{}us -> FPS: {:.3}\n\n",
                frame.len(),
                frame.width(),
                frame.height(),
                frame.generation,
                &frame.data(),
                time_since_last_fb,
                fps,
            );

            // Move the framebuffer into a higher order structure
            let fb_view = Arc::new(FrameBufferView::from_driver(frame, fps));
            FRAME_DONE_SIGNAL.reset();
            // Send it to the control task for consumption
            control_loop_tx.send(fb_view.clone());

            // If webserver is enabled, copy the framebuffer for consumption there
            // Note: each FB copy takes ~30ms, this directly impacts control loop perf
            #[cfg(feature = "webserver")]
            {
                // Send to Webserver task
                webserver_watch.send(fb_view.clone());
            }
            // If SD logging is enabled, copy the framebuffer for consumption there
            // Note: each FB copy takes ~30ms, this directly impacts control loop perf
            #[cfg(feature = "sd")]
            {
                // Throttle logging
                if frame.generation % CAMERA_TARGET_FPS == 0 {
                    let start = Instant::now();

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
                        SystemTime::now().duration_since(start).as_millis(),
                    );
                }
            }

            debug!(
                "CAMERA: Strong count before signal: {}",
                Arc::strong_count(&fb_view)
            );
            while !FRAME_DONE_SIGNAL.signaled() {
                // warn!("CAMERA: wait for control loop signal");
                // Wait for 1ms
                vTaskDelay(1)
            }

            // Force the watch to release its Arc clone
            // Next frame will overwrite it anyway
            #[cfg(feature = "webserver")]
            webserver_watch.clear();
            control_loop_tx.clear();

            // The Esp-camera driver requires us to release the framebuffer in this task
            // Now is a good time, as the control loop has indicated it's done with the frame
            // SAFETY: This upholds the Esp-camera driver constraint of releasing its framebuffer in
            // the same FreeRtos task that produced it
            debug!(
                "CAMERA: Arc strong_count before return to driver: {}",
                Arc::strong_count(&fb_view)
            );

            if Arc::strong_count(&fb_view) > 1 {
                log::warn!(
                    "CAMERA: Arc count: {} -> still has other owners after control signal &
                    clearing webserver channel -> bug!",
                    Arc::strong_count(&fb_view)
                );
            }
            // Release the esp-camera PSRAM framebuffer, accepting possible tearing for the
            // webserver
            fb_view.return_to_driver();
        };

        // Timekeeping
        let last_wake_ptr: *mut u32 = &mut x_last_wake_time as _;
        if xTaskDelayUntil(
            last_wake_ptr,
            configTICK_RATE_HZ / cfg.camera_target_fps as u32,
        ) != 0
        {
            log::error!("CAMERA: camera task not delayed -> freertos cannot keep up!");
        };
    }
}
