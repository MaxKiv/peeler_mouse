use crate::camera::camera_freertos_task::FRAMEBUFFER_WEBSERVER_CHANNEL;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex as Cs,
    watch::{Receiver, Sender},
};
use esp_idf_hal::io::Write;
use esp_idf_svc::http::{
    server::{EspHttpConnection, EspHttpServer, Request},
    Method,
};
use log::*;

pub fn handle_camera_grayscale(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // Get latest frame

    let mut rx = FRAMEBUFFER_WEBSERVER_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_WEBSERVER_CHANNEL rx N");

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

    log::info!(
        "webserver got {}x{} framebuffer gen {} @ {:p}\n\n",
        frame.width,
        frame.height,
        frame.generation,
        &frame.data,
    );

    // PGM headers
    let header = format!("P5\n{} {}\n255\n", frame.width, frame.height);
    let headers = [
        ("Content-Type", "image/x-portable-graymap"),
        ("Cache-Control", "no-store"),
    ];

    // Draft response
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(header.as_bytes())?;
    response.write_all(&frame.data)?;
    response.flush()?;

    Ok(())
}
