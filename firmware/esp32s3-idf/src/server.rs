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

use crate::{
    camera::camera_freertos_task::CAMERA_FRAMEBUFFER, request::ReadableRequest, wifi::WifiState,
    Setpoint,
};

#[embassy_executor::task]
pub async fn server_task(
    mut wifi_state_receiver: Receiver<'static, Cs, WifiState, 1>,
    setpoint_sender: Sender<'static, Cs, Setpoint, 1>,
) {
    loop {
        // Set up a HTTP server when wifi is connected
        match wifi_state_receiver.try_get() {
            Some(WifiState::Connected) => {
                match EspHttpServer::new(&esp_idf_svc::http::server::Configuration::default()) {
                    Ok(mut server) => {
                        info!("HTTP server constructed, setting up handlers");

                        // Set up HTTP server handlers
                        if let Err(err) = server.fn_handler("/", Method::Get, handle_root) {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        if let Err(err) =
                            server.fn_handler("/camera", Method::Get, move |request| {
                                handle_camera(request)
                            })
                        {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        let sender = setpoint_sender.clone();
                        if let Err(err) =
                            server.fn_handler("/setpoint", Method::Post, move |request| {
                                handle_setpoint(request, &sender)
                            })
                        {
                            error!("Unable to set up HTTP Server root handler: {err}, retrying...");
                        }

                        info!(
                            "HTTP server handlers set up, keeping alive until wifi connected is dropped"
                        );

                        // Keep server alive untill wifi connection drops
                        if let WifiState::Disconnected = wifi_state_receiver.changed().await {
                            warn!("Wifi disconnected, dropping & reconfiguring HTTP server");
                        }
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
    // A cursed html + javascript static webpage :)
    let data = r#"
        <html>
          <body>
            <img id="cam" src="/camera" width="640">
            <br>
            <button onclick="refresh()">Fetch new frame</button>

            <script>
              function refresh() {
                const img = document.getElementById("cam");
                img.src = "/camera?t=" + Date.now(); // bust cache
              }
            </script>

            <input type="number" id="depth" placeholder="0">
            <button onclick="sendSetpoint()">Send Setpoint</button>

            <script>
              function sendSetpoint() {
                console.log("MAX triggered sendsetpoint")
                const depth = document.getElementById("depth").value;
                fetch("/setpoint", {
                  method: "POST",
                  headers: {
                    "Content-Type": "application/json",
                    "Connection": "close" // force a new request/connection
                  },
                  body: JSON.stringify({ depth: parseFloat(depth) })
                }).then(resp => resp.text())
                  .then(txt => alert("Response: " + txt))
                  .catch(err => alert("Error: " + err));
              }
            </script>
          </body>
        </html>
    "#;

    let headers = [
        ("Content-Type", "text/html"),
        ("Content-Length", &data.len().to_string()),
    ];

    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(data.as_bytes())?;
    response.flush()?;
    Ok(())
}

fn handle_camera(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    let mut frame_rx = CAMERA_FRAMEBUFFER
        .receiver()
        .expect("Max CAMERA_FRAMEBUFFER receivers reached");

    // Get latest frame
    let Some(frame) = frame_rx.try_get() else {
        let mut resp = request.into_response(
            503,
            Some("Service Unavailable"),
            &[("Content-Type", "text/plain")],
        )?;
        resp.write_all(b"No frame available yet")?;
        resp.flush()?;
        return Ok(());
    };

    // PGM headers
    let header = format!("P5\n{} {}\n255\n", frame.width(), frame.height());
    let headers = [
        ("Content-Type", "image/x-portable-graymap"),
        ("Cache-Control", "no-store"),
    ];

    // Draft response
    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(header.as_bytes())?;
    response.write_all(frame.data())?;
    response.flush()?;

    Ok(())
}

fn handle_setpoint(
    mut request: Request<&mut EspHttpConnection<'_>>,
    sender: &Sender<'static, Cs, Setpoint, 1>,
) -> anyhow::Result<()> {
    log::info!("Received setpoint");

    // Try to deserialize received data into setpoint
    let readable_request = ReadableRequest(&mut request);
    let setpoint: Setpoint = match readable_request.deserialize_into() {
        Ok(r) => r,
        Err(err) => {
            log::warn!("Unable to deserialize get request into setpoint: {err}",);
            let mut response = request.into_response(
                400,
                Some("Bad Request"),
                &[("Content-Type", "text/plain")],
            )?;
            response.write_all(format!("Invalid JSON: {err}").as_bytes())?;
            response.flush()?;
            return Ok(());
        }
    };
    log::info!("Deserialisation success! {setpoint:?}");

    sender.send(setpoint);

    // Success response
    let mut response = request.into_response(200, Some("OK"), &[("Content-Type", "text/plain")])?;
    response.write_all(b"Depth setpoint updated")?;
    response.flush()?;
    Ok(())
}
