use crate::camera::camera_freertos_task::FRAMEBUFFER_WEBSERVER_CHANNEL;
use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;
pub fn handle_camera_jpeg(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let mut rx = FRAMEBUFFER_WEBSERVER_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_SD_CHANNEL rx N");

    loop {
        let Some(frame) = rx.try_changed() else {
            let mut resp = request.into_response(
                503,
                Some("Service Unavailable"),
                &[("Content-Type", "text/plain")],
            )?;
            resp.write_all(b"No frame available yet")?;
            resp.flush()?;
            return Ok(());
        };

        let part_boundary = "peeler-mouse";
        let frame_boundary = format!("\r\n--{part_boundary}\r\n");

        let content_type = format!("multipart/x-mixed-replace;boundary={part_boundary}");
        let headers = [("Content-Type", content_type.as_str())];
        let mut response = request.into_response(200, Some("OK"), &headers)?;

        let frame_part = format!(
            "Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
            frame.data.len()
        );
        response.write_all(frame_part.as_bytes())?;
        response.write_all(&frame.data)?;
        response.write_all(frame_boundary.as_bytes())?;
        response.flush()?;
    }

    Ok(())
}
