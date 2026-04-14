pub mod camera_freertos_task;
pub mod esp_cam_wrapper;
pub mod framebuffer;
pub mod framebuffer_view;
pub mod framesize;
pub mod peripherals;
pub mod pixelformat;

use crate::camera::{framesize::FrameSize, pixelformat::PixelFormat};

#[cfg(not(feature = "streaming"))]
pub const PIXEL_FORMAT: PixelFormat = PixelFormat::GRAYSCALE;
#[cfg(not(feature = "streaming"))]
pub const CAMERA_TARGET_FPS: u64 = 2;
#[cfg(not(feature = "streaming"))]
/// Large effect on driver behavior: When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
pub const FB_COUNT: usize = 1;
#[cfg(not(feature = "streaming"))]
pub const XCLK_FREQ: i32 = 16_000_000;

#[cfg(feature = "streaming")]
pub const PIXEL_FORMAT: PixelFormat = PixelFormat::JPEG;
#[cfg(feature = "streaming")]
pub const CAMERA_TARGET_FPS: u64 = 5;
#[cfg(feature = "streaming")]
/// Large effect on driver behavior: When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
pub const FB_COUNT: usize = 2;
#[cfg(feature = "streaming")]
pub const XCLK_FREQ: i32 = 10_000_000;

pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeQvga;
pub const FRAMEBUFFER_LEN: usize = FRAME_SIZE.get_dimensions().0 * FRAME_SIZE.get_dimensions().1;
pub const JPEG_QUALITY: i32 = 30;

pub struct CameraConfig {
    pub pixel_format: PixelFormat,
    pub frame_size: FrameSize,
    pub framebuffer_len: usize,
    pub xclk_freq: i32,
    pub jpeg_quality: i32,
    pub camera_target_fps: u64,
    pub fb_count: usize,
}

impl CameraConfig {
    pub fn new() -> Self {
        Self {
            pixel_format: PIXEL_FORMAT,
            frame_size: FRAME_SIZE,
            framebuffer_len: FRAMEBUFFER_LEN,
            xclk_freq: XCLK_FREQ,
            jpeg_quality: JPEG_QUALITY,
            camera_target_fps: CAMERA_TARGET_FPS,
            fb_count: FB_COUNT,
        }
    }
}
