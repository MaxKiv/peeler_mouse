use std::sync::Arc;

use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex as Cs, watch::Receiver};
use esp_idf_hal::io::Write;
use esp_idf_svc::http::{
    server::{EspHttpConnection, EspHttpServer, Request},
    Method,
};
use log::*;

use crate::{espcam::Camera, wifi::WifiState};

#[embassy_executor::task]
pub async fn server_task(
    camera: Arc<Camera<'static>>,
    mut wifi_state_receiver: Receiver<'static, Cs, WifiState, 1>,
) {
    loop {
        // Set up a HTTP server when wifi is connected
        match wifi_state_receiver.try_get() {
            Some(WifiState::Connected) => {
                match EspHttpServer::new(&esp_idf_svc::http::server::Configuration::default()) {
                    Ok(mut server) => {
                        // Set up HTTP server handlers
                        if let Err(err) = server.fn_handler("/", Method::Get, handle_root) {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }
                        let camera = camera.clone();
                        if let Err(err) =
                            server.fn_handler("/camera", Method::Get, move |request| {
                                handle_camera(request, &camera)
                            })
                        {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        // Keep server alive untill wifi connection drops
                        loop {
                            if let WifiState::Disconnected = wifi_state_receiver.changed().await {
                                warn!("WiFi disconnected, shutting down HTTP server...");
                                break; // drops server -> EspHttpServer::drop stops it
                            }
                        }

                        core::future::pending::<()>().await;
                    }
                    Err(err) => {
                        error!("Unable to set up HTTP Server: {err}, retrying...");
                    }
                }
            }
            _ => {
                warn!("Wifi is not yet connected -> Can't set up webserver, retrying soon...");
                embassy_time::Timer::after_millis(500).await;
            }
        }
    }
}

fn handle_root(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let data = "<html><head><meta name=\"viewport\" content=\"width=device-width; height=device-height;\"><title>esp32cam</title></head><body><img src=\"camera\" alt=\"Failed to load image\" style=\"height: 100%;width: 100%; transform: rotate(180deg);\"></body></html>";

    let headers = [
        ("Content-Type", "text/html"),
        ("Content-Length", &data.len().to_string()),
    ];
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(data.as_bytes())?;
    Ok(())
}

fn handle_camera(
    request: Request<&mut EspHttpConnection<'_>>,
    camera: &Arc<Camera>,
) -> anyhow::Result<()> {
    let part_boundary = "123456789000000000000987654321";
    let frame_boundary = format!("\r\n--{part_boundary}\r\n");

    let content_type = format!("multipart/x-mixed-replace;boundary={part_boundary}");
    let headers = [("Content-Type", content_type.as_str())];
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    loop {
        if let Some(fb) = camera.get_framebuffer() {
            let data = fb.data();
            let frame_part = format!(
                "Content-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
                data.len()
            );
            response.write_all(frame_part.as_bytes())?;
            response.write_all(data)?;
            response.write_all(frame_boundary.as_bytes())?;
            response.flush()?;
        }
    }
}
