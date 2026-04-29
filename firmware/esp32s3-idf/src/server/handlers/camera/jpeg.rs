use crate::camera::camera_freertos_task::FRAMEBUFFER_WEBSERVER_CHANNEL;
use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;

pub fn handle_camera_jpeg(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let mut rx = FRAMEBUFFER_WEBSERVER_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_WEBSERVER_CHANNEL rx N");

    let part_boundary = "peeler-mouse";
    let content_type = format!("multipart/x-mixed-replace;boundary={part_boundary}");

    // Consume request ONCE, get a writer back
    let mut response =
        request.into_response(200, Some("OK"), &[("Content-Type", content_type.as_str())])?;

    loop {
        // Block until a new frame arrives
        // Use try_changed for non-blocking, or changed() if you want to block
        let frame = match rx.try_changed() {
            Some(f) => f,
            None => {
                // No new frame yet, yield and retry
                std::thread::sleep(std::time::Duration::from_millis(10));
                continue;
            }
        };

        let data = frame.fb.data();
        let part_header = format!(
            "--{part_boundary}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
            data.len()
        );

        // Write part header, then JPEG bytes, then trailing CRLF
        response.write_all(part_header.as_bytes())?;
        response.write_all(data)?;
        response.write_all(b"\r\n")?;
        response.flush()?;
    }
}
