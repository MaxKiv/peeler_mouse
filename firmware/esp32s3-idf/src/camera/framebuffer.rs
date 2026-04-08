use crate::camera::esp_cam_wrapper::EspCamFrameBuffer;

use esp_idf_sys::*;

#[derive(Debug)]
pub struct FrameBuffer {
    pub data: Vec<u8>,
    pub width: usize,
    pub height: usize,
    pub format: camera::pixformat_t,
    pub generation: u32,   // Frame generation counter
    pub fps: f64,          // Approximate FPS at time of capture
    pub timestamp_us: i64, // microseconds_since_epoch
}

impl Clone for FrameBuffer {
    fn clone(&self) -> Self {
        // Yea
        self.try_clone().unwrap()
    }
}

impl FrameBuffer {
    pub unsafe fn try_from_esp(
        fb: &EspCamFrameBuffer,
        fps: f64,
        timestamp_us: i64,
    ) -> Option<FrameBuffer> {
        let len = fb.len();

        let ptr = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA) as *mut u8;

        let bytes_left = heap_caps_get_free_size(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
        if ptr.is_null() {
            log::warn!("FrameBuffer allocation in PSRAM FAILED, SPIRAM + DMA capable heap left: {bytes_left}");
            // Alloc failed
            return None;
        }
        log::info!("FrameBuffer allocated in PSRAM, SPIRAM + DMA capable heap left: {bytes_left}");

        let src = fb.data().as_ptr();
        core::ptr::copy_nonoverlapping(src, ptr, len);

        let data = Vec::from_raw_parts(ptr, len, len);

        Some(FrameBuffer {
            data,
            width: fb.width(),
            height: fb.height(),
            format: fb.format(),
            generation: fb.generation,
            timestamp_us,
            fps,
        })
    }

    pub fn try_clone(&self) -> Option<Self> {
        let len = self.data.len();

        unsafe {
            let ptr = heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) as *mut u8;

            let bytes_left = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
            if ptr.is_null() {
                log::warn!("FrameBuffer allocation in PSRAM FAILED, SPIRAM + DMA capable heap left: {bytes_left}");
                // Alloc failed
                return None;
            }
            log::info!(
                "FrameBuffer allocated in PSRAM, SPIRAM + DMA capable heap left: {bytes_left}"
            );

            let src = self.data.as_ptr();
            core::ptr::copy_nonoverlapping(src, ptr, len);

            let data = Vec::from_raw_parts(ptr, len, len);

            Some(FrameBuffer {
                data,
                width: self.width,
                height: self.height,
                format: self.format,
                generation: self.generation,
                timestamp_us: self.timestamp_us,
                fps: self.fps,
            })
        }
    }

    // #[inline]
    pub fn rows(&self) -> impl Iterator<Item = &[u8]> {
        self.data.chunks(self.width)
    }
}
