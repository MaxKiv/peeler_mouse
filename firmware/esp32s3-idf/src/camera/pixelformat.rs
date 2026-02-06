use crate::camera::pixelformat::camera::*;
use esp_idf_sys::*;

#[derive(Debug, Clone, Copy)]
pub enum PixelFormat {
    RGB565,
    YUV422,
    YUV420,
    GRAYSCALE,
    JPEG,
    RGB888,
    RAW,
    RGB444,
    RGB555,
    RAW8,
    Undefined,
}

impl From<camera::pixformat_t> for PixelFormat {
    fn from(other: camera::pixformat_t) -> Self {
        use esp_idf_sys::camera;
        #[allow(clippy::non_snake_case)]
        match other {
            pixformat_t_PIXFORMAT_RGB565 => PixelFormat::RGB565, // 2BPP/RGB565
            pixformat_t_PIXFORMAT_YUV422 => PixelFormat::YUV422, // 2BPP/YUV422
            pixformat_t_PIXFORMAT_YUV420 => PixelFormat::YUV420, // 1.5BPP/YUV420
            pixformat_t_PIXFORMAT_GRAYSCALE => PixelFormat::GRAYSCALE, // 1BPP/GRAYSCALE
            pixformat_t_PIXFORMAT_JPEG => PixelFormat::JPEG,     // JPEG/COMPRESSED
            pixformat_t_PIXFORMAT_RGB888 => PixelFormat::RGB888, // 3BPP/RGB888
            pixformat_t_PIXFORMAT_RAW => PixelFormat::RAW,       // RAW
            pixformat_t_PIXFORMAT_RGB444 => PixelFormat::RGB444, // 3BP2P/RGB444
            pixformat_t_PIXFORMAT_RGB555 => PixelFormat::RGB555, // 3BP2P/RGB555
            pixformat_t_PIXFORMAT_RAW8 => PixelFormat::RAW8,     // RAW 8-bit
            _ => PixelFormat::Undefined,                         // Fallthrough
        }
    }
}

impl From<PixelFormat> for camera::pixformat_t {
    fn from(pf: PixelFormat) -> Self {
        use esp_idf_sys::camera;
        match pf {
            PixelFormat::RGB565 => pixformat_t_PIXFORMAT_RGB565,
            PixelFormat::YUV422 => pixformat_t_PIXFORMAT_YUV422,
            PixelFormat::YUV420 => pixformat_t_PIXFORMAT_YUV420,
            PixelFormat::GRAYSCALE => pixformat_t_PIXFORMAT_GRAYSCALE,
            PixelFormat::JPEG => pixformat_t_PIXFORMAT_JPEG,
            PixelFormat::RGB888 => pixformat_t_PIXFORMAT_RGB888,
            PixelFormat::RAW => pixformat_t_PIXFORMAT_RAW,
            PixelFormat::RGB444 => pixformat_t_PIXFORMAT_RGB444,
            PixelFormat::RGB555 => pixformat_t_PIXFORMAT_RGB555,
            PixelFormat::RAW8 => pixformat_t_PIXFORMAT_RAW8,
            PixelFormat::Undefined => todo!(),
        }
    }
}
