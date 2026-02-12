use crate::camera::{
    esp_cam_wrapper::Camera, framebuffer::FrameBuffer, framesize::FrameSize,
    peripherals::CameraPeripherals, pixelformat::PixelFormat,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, signal::*};

use esp_idf_sys::*;
use log::*;

pub const PIXEL_FORMAT: PixelFormat = PixelFormat::GRAYSCALE;
// pub const PIXEL_FORMAT: PixelFormat = PixelFormat::JPEG;
pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeVga;
pub const FRAMEBUFFER_LEN: usize = FRAME_SIZE.get_dimensions().0 * FRAME_SIZE.get_dimensions().1;
pub const XCLK_FREQ: i32 = 20_000_000;
pub const JPEG_QUALITY: i32 = 20;

pub static FRAMEBUFFER_WEBSERVER_CHANNEL: Signal<Cs, FrameBuffer> = Signal::new();
pub static FRAMEBUFFER_SD_CHANNEL: Signal<Cs, FrameBuffer> = Signal::new();
static mut CAMERA_TASK_ARGS: Option<CameraTaskArgs> = None;

pub struct CameraTaskArgs {
    camera_peripherals: Option<CameraPeripherals>,
    webserver_signal: &'static Signal<Cs, FrameBuffer>,
    sd_signal: &'static Signal<Cs, FrameBuffer>,
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
        // Take picture!
        if let Some(frame) = cam.get_framebuffer() {
            log::info!(
                "Camera got {}x{} framebuffer gen {} @ {:p}\n\n",
                frame.width(),
                frame.height(),
                frame.generation,
                &frame.data(),
            );

            // Copy framebuffer, continue if this fails
            let Some(fb_owned) = FrameBuffer::try_from_esp(frame) else {
                // Failed to alloc, wait a bit and continue
                vTaskDelay(1 * configTICK_RATE_HZ);
                continue;
            };

            // Send the copied frame buffer to embassy context
            if let Some(fb_copy) = fb_owned.try_clone() {
                webserver_signal.signal(fb_copy);
            }
            sd_signal.signal(fb_owned);
        };

        // 1 Hz
        vTaskDelay(1 * configTICK_RATE_HZ);

        // Signal the imminent destruction of the framebuffer
        // This actually triggers the camera::esp_camera_fb_return(fb) through FrameBuffer::Drop,
        // which is required before fetching a new frame
        // signal.signal(None);

        // vTaskDelay(1 * configTICK_RATE_HZ);
    }
}
