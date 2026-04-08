use esp_idf_sys::camera;

use crate::{camera::esp_cam_wrapper::EspCamFrameBuffer, util::hash::fnv1a};

#[derive(Debug)]
pub struct FrameBufferView {
    pub width: usize,
    pub height: usize,
    pub format: camera::pixformat_t,
    pub generation: u32,            // Frame generation counter
    pub fps: f64,                   // Approximate FPS at time of capture
    pub timestamp: EspCamTimeStamp, // microseconds_since_epoch
    pub hash: u32,                  // FNV-1A hash of buf
    fb: EspCamFrameBuffer, // "Raw" framebuffer pointer, [`Drop`] this to release PSRAM memory back to esp-camera
}

/// Safety: buf slice contains PSRAM pointer, memory owned by esp-camera driver
/// Camera should halt DMA until camera::esp_camera_fb_return() is called
/// Meaning anyone slow owning this slows down camara FPS
/// Invariant: No consumer may access framebuffer memory after the camera task has published the next generation.
unsafe impl Send for FrameBufferView {}

impl FrameBufferView {
    pub fn from_driver(fb: EspCamFrameBuffer, fps: f64) -> Self {
        let hash = fnv1a(fb.data());
        let timestamp_us = EspCamTimeStamp::from(fb.timestamp());

        Self {
            width: fb.width(),
            height: fb.height(),
            format: fb.format(),
            generation: fb.generation,
            fps,
            timestamp: timestamp_us,
            hash,
            fb,
        }
    }

    /// Calculate the fnv1a checksum of framebuffer
    /// Used to detect tearing/aliasing from DMA writes by esp camera
    pub fn calculate_checksum(&self) -> u32 {
        fnv1a(self.fb.data())
    }

    /// Safe because buf_ptr comes from fb which we own and has not been freed
    pub fn data(&self) -> &[u8] {
        self.fb.data()
    }

    pub fn rows(&self) -> impl Iterator<Item = &[u8]> {
        self.data().chunks(self.width)
    }
}

#[derive(Debug, Clone)]
pub struct EspCamTimeStamp {
    pub tv_sec: i64,  /* Seconds since boot of DMA completion */
    pub tv_usec: i32, /* Microseconds.  */
}

impl From<camera::timeval> for EspCamTimeStamp {
    fn from(value: camera::timeval) -> Self {
        Self {
            tv_sec: value.tv_sec,
            tv_usec: value.tv_usec,
        }
    }
}
