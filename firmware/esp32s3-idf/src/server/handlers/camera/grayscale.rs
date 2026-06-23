use crate::camera::camera_freertos_task::FRAMEBUFFER_WEBSERVER_CHANNEL;
use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use esp_idf_sys::vTaskDelay;
use log::*;

// GET /camera handler when camera pixel_format_t = grayscale
pub fn handle_camera_grayscale(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // Get latest frame
    let mut rx = FRAMEBUFFER_WEBSERVER_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_WEBSERVER_CHANNEL rx N");

    // Wait for latest frame
    let frame = loop {
        if let Some(frame) = rx.try_changed() {
            break frame;
        }

        unsafe {
            vTaskDelay(10);
        }
    };

    log::warn!(
        "webserver /camera got {}x{} framebuffer gen {} @ {:p}\n\n",
        frame.width,
        frame.height,
        frame.generation,
        &frame.fb,
    );

    // Build PGM headers
    let header = format!("P5\n{} {}\n255\n", frame.width, frame.height);
    let headers = [
        ("Content-Type", "image/x-portable-graymap"),
        ("Cache-Control", "no-store"),
    ];

    // Draft response
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(header.as_bytes())?;
    response.write_all(&frame.fb.data())?;
    response.flush()?;

    Ok(())
}
