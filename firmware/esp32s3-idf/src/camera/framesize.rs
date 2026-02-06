use crate::camera::framesize::camera::*;
use esp_idf_sys::*;

#[derive(Debug)]
pub enum FrameSize {
    Framesize96x96,   // 96x96
    FramesizeQqvga,   // 160x120
    Framesize128x128, // 128x128
    FramesizeQcif,    // 176x144
    FramesizeHqvga,   // 240x176
    Framesize240x240, // 240x240
    FramesizeQvga,    // 320x240
    Framesize320x320, // 320x320
    FramesizeCif,     // 400x296
    FramesizeHvga,    // 480x320
    FramesizeVga,     // 640x480
    FramesizeSvga,    // 800x600
    FramesizeXga,     // 1024x768
    FramesizeHd,      // 1280x720
    FramesizeSxga,    // 1280x1024
    FramesizeUxga,    // 1600x1200
    // 3MP Sensors
    FramesizeFhd,  // 1920x1080
    FramesizePHd,  // 720x1280
    FramesizeP3mp, // 864x1536
    FramesizeQxga, // 2048x1536
    // 5MP Sensors
    FramesizeQhd,   // 2560x1440
    FramesizeWqxga, // 2560x1600
    FramesizePFhd,  // 1080x1920
    FramesizeQsxga, // 2560x1920
    Framesize5mp,   // 2592x1944
    FramesizeInvalid,
}

impl From<camera::framesize_t> for FrameSize {
    fn from(other: camera::framesize_t) -> Self {
        use esp_idf_sys::camera;
        #[allow(clippy::non_snake_case)]
        match other {
            framesize_t_FRAMESIZE_96X96 => FrameSize::Framesize96x96, // 96x96
            framesize_t_FRAMESIZE_QQVGA => FrameSize::FramesizeQqvga, // 160x120
            framesize_t_FRAMESIZE_128X128 => FrameSize::Framesize128x128, // 128x128
            framesize_t_FRAMESIZE_QCIF => FrameSize::FramesizeQcif,   // 176x144
            framesize_t_FRAMESIZE_HQVGA => FrameSize::FramesizeHqvga, // 240x176
            framesize_t_FRAMESIZE_240X240 => FrameSize::Framesize240x240, // 240x240
            framesize_t_FRAMESIZE_QVGA => FrameSize::FramesizeQvga,   // 320x240
            framesize_t_FRAMESIZE_320X320 => FrameSize::Framesize320x320, // 320x320
            framesize_t_FRAMESIZE_CIF => FrameSize::FramesizeCif,     // 400x296
            framesize_t_FRAMESIZE_HVGA => FrameSize::FramesizeHvga,   // 480x320
            framesize_t_FRAMESIZE_VGA => FrameSize::FramesizeVga,     // 640x480
            framesize_t_FRAMESIZE_SVGA => FrameSize::FramesizeSvga,   // 800x600
            framesize_t_FRAMESIZE_XGA => FrameSize::FramesizeXga,     // 1024x768
            framesize_t_FRAMESIZE_HD => FrameSize::FramesizeHd,       // 1280x720
            framesize_t_FRAMESIZE_SXGA => FrameSize::FramesizeSxga,   // 1280x1024
            framesize_t_FRAMESIZE_UXGA => FrameSize::FramesizeUxga,   // 1600x1200
            // 3MP Sensors
            framesize_t_FRAMESIZE_FHD => FrameSize::FramesizeFhd, // 1920x1080
            framesize_t_FRAMESIZE_P_HD => FrameSize::FramesizePHd, // 720x1280
            framesize_t_FRAMESIZE_P_3MP => FrameSize::FramesizeP3mp, // 864x1536
            framesize_t_FRAMESIZE_QXGA => FrameSize::FramesizeQxga, // 2048x1536
            // 5MP Sensors
            framesize_t_FRAMESIZE_QHD => FrameSize::FramesizeQhd, // 2560x1440
            framesize_t_FRAMESIZE_WQXGA => FrameSize::FramesizeWqxga, // 2560x1600
            framesize_t_FRAMESIZE_P_FHD => FrameSize::FramesizePFhd, // 1080x1920
            framesize_t_FRAMESIZE_QSXGA => FrameSize::FramesizeQsxga, // 2560x1920
            framesize_t_FRAMESIZE_5MP => FrameSize::Framesize5mp, // 2592x1944
            framesize_t_FRAMESIZE_INVALID => FrameSize::FramesizeInvalid,
            _ => FrameSize::FramesizeInvalid, // Fallback for unknown variants
        }
    }
}

impl FrameSize {
    /// Returns the (width, height) dimensions for the given `FrameSize`.
    pub const fn get_dimensions(&self) -> (usize, usize) {
        match self {
            FrameSize::Framesize96x96 => (96, 96),
            FrameSize::FramesizeQqvga => (160, 120),
            FrameSize::Framesize128x128 => (128, 128),
            FrameSize::FramesizeQcif => (176, 144),
            FrameSize::FramesizeHqvga => (240, 176),
            FrameSize::Framesize240x240 => (240, 240),
            FrameSize::FramesizeQvga => (320, 240),
            FrameSize::Framesize320x320 => (320, 320),
            FrameSize::FramesizeCif => (400, 296),
            FrameSize::FramesizeHvga => (480, 320),
            FrameSize::FramesizeVga => (640, 480),
            FrameSize::FramesizeSvga => (800, 600),
            FrameSize::FramesizeXga => (1024, 768),
            FrameSize::FramesizeHd => (1280, 720),
            FrameSize::FramesizeSxga => (1280, 1024),
            FrameSize::FramesizeUxga => (1600, 1200),
            FrameSize::FramesizeFhd => (1920, 1080),
            FrameSize::FramesizePHd => (720, 1280),
            FrameSize::FramesizeP3mp => (864, 1536),
            FrameSize::FramesizeQxga => (2048, 1536),
            FrameSize::FramesizeQhd => (2560, 1440),
            FrameSize::FramesizeWqxga => (2560, 1600),
            FrameSize::FramesizePFhd => (1080, 1920),
            FrameSize::FramesizeQsxga => (2560, 1920),
            FrameSize::Framesize5mp => (2592, 1944),
            FrameSize::FramesizeInvalid => (0, 0),
        }
    }
}

impl From<FrameSize> for camera::framesize_t {
    fn from(framesize: FrameSize) -> Self {
        use esp_idf_sys::camera;
        match framesize {
            FrameSize::Framesize96x96 => framesize_t_FRAMESIZE_96X96, // 96x96
            FrameSize::FramesizeQqvga => framesize_t_FRAMESIZE_QQVGA, // 160x120
            FrameSize::Framesize128x128 => framesize_t_FRAMESIZE_128X128, // 128x128
            FrameSize::FramesizeQcif => framesize_t_FRAMESIZE_QCIF,   // 176x144
            FrameSize::FramesizeHqvga => framesize_t_FRAMESIZE_HQVGA, // 240x176
            FrameSize::Framesize240x240 => framesize_t_FRAMESIZE_240X240, // 240x240
            FrameSize::FramesizeQvga => framesize_t_FRAMESIZE_QVGA,   // 320x240
            FrameSize::Framesize320x320 => framesize_t_FRAMESIZE_320X320, // 320x320
            FrameSize::FramesizeCif => framesize_t_FRAMESIZE_CIF,     // 400x296
            FrameSize::FramesizeHvga => framesize_t_FRAMESIZE_HVGA,   // 480x320
            FrameSize::FramesizeVga => framesize_t_FRAMESIZE_VGA,     // 640x480
            FrameSize::FramesizeSvga => framesize_t_FRAMESIZE_SVGA,   // 800x600
            FrameSize::FramesizeXga => framesize_t_FRAMESIZE_XGA,     // 1024x768
            FrameSize::FramesizeHd => framesize_t_FRAMESIZE_HD,       // 1280x720
            FrameSize::FramesizeSxga => framesize_t_FRAMESIZE_SXGA,   // 1280x1024
            FrameSize::FramesizeUxga => framesize_t_FRAMESIZE_UXGA,   // 1600x1200
            // 3MP Sensors
            FrameSize::FramesizeFhd => framesize_t_FRAMESIZE_FHD, // 1920x1080
            FrameSize::FramesizePHd => framesize_t_FRAMESIZE_P_HD, // 720x1280
            FrameSize::FramesizeP3mp => framesize_t_FRAMESIZE_P_3MP, // 864x1536
            FrameSize::FramesizeQxga => framesize_t_FRAMESIZE_QXGA, // 2048x1536
            // 5MP Sensors
            FrameSize::FramesizeQhd => framesize_t_FRAMESIZE_QHD, // 2560x1440
            FrameSize::FramesizeWqxga => framesize_t_FRAMESIZE_WQXGA, // 2560x1600
            FrameSize::FramesizePFhd => framesize_t_FRAMESIZE_P_FHD, // 1080x1920
            FrameSize::FramesizeQsxga => framesize_t_FRAMESIZE_QSXGA, // 2560x1920
            FrameSize::Framesize5mp => framesize_t_FRAMESIZE_5MP, // 2592x1944
            FrameSize::FramesizeInvalid => framesize_t_FRAMESIZE_INVALID,
        }
    }
}
