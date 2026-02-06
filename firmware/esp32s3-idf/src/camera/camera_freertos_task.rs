use std::sync::Arc;

use crate::camera::{
    esp_cam_wrapper::{Camera, FrameBuffer},
    framesize::FrameSize,
    peripherals::CameraPeripherals,
    pixelformat::PixelFormat,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::*};

use esp_idf_sys::*;
use log::*;

pub const PIXEL_FORMAT: PixelFormat = PixelFormat::GRAYSCALE;
pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeQqvga;
pub const FRAMEBUFFER_LEN: usize = FRAME_SIZE.get_dimensions().0 * FRAME_SIZE.get_dimensions().1;
pub const XCLK_FREQ: i32 = 20_000_000;
pub const JPEG_QUALITY: i32 = 32;

pub static CAMERA_FRAMEBUFFER: Watch<Cs, Arc<FrameBuffer>, 1> = Watch::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    watch: &'static Watch<Cs, Arc<FrameBuffer>, 1>,
}

pub fn setup_freertos(camera_peripherals: CameraPeripherals) {
    info!("Setting up Camera FreeRtos task");

    unsafe {
        CAMERA_TASK_ARGS = Some(CameraTaskArgs {
            camera_peripherals: Some(camera_peripherals),
            watch: &CAMERA_FRAMEBUFFER,
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

    let watch = args.watch;

    let camera_peripherals = args
        .camera_peripherals
        .take()
        .expect("Camera peripherals already taken");

    log::info!("Initialising framebuffer sender");
    let tx = watch.sender();

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
        1,
    ) {
        Ok(cam) => cam,
        Err(err) => {
            log::error!("Issue initialising camera: {err}");
            return;
        }
    };

    log::info!("camera initialised");

    loop {
        // Take picture!
        if let Some(frame) = cam.get_framebuffer() {
            log::info!(
                "camera got {}x{} framebuffer\n\n",
                frame.width(),
                frame.height(),
            );

            // // Send a reference counted pointer to the framebuffer to the embassy context
            // tx.send(Arc::new(frame));
        };

        // 1 Hz
        vTaskDelay(5 * configTICK_RATE_HZ);
    }
}
