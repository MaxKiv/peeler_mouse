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
        <!DOCTYPE html>
        <html>
          <body>
            <br>
            <canvas id="camCanvas"></canvas>
            <br>
            <button onclick="fetchFrame()">Refresh</button>

            <script>
            async function fetchFrame() {
              let res;
              try {
                res = await fetch("/camera", { cache: "no-store" });
              } catch (e) {
                console.error("Fetch failed:", e);
                return;
              }

              if (!res.ok) {
                if (res.status === 503) {
                  console.log("No frame available yet");
                  return;
                }
                console.error("Camera error:", res.status);
                return;
              }

              const buf = await res.arrayBuffer();
              const bytes = new Uint8Array(buf);

              let idx = 0;

              function skipWhitespace() {
                while (idx < bytes.length && bytes[idx] <= 32) idx++;
              }

              function readToken() {
                skipWhitespace();
                if (bytes[idx] === 35) { // '#'
                  while (bytes[idx++] !== 10);
                  return readToken();
                }
                let start = idx;
                while (idx < bytes.length && bytes[idx] > 32) idx++;
                return String.fromCharCode(...bytes.slice(start, idx));
              }

              const magic = readToken();
              if (magic !== "P5") {
                console.error("Not a P5 PGM");
                return;
              }

              const width = parseInt(readToken(), 10);
              const height = parseInt(readToken(), 10);
              const maxval = parseInt(readToken(), 10);

              if (!Number.isFinite(width) || !Number.isFinite(height) || maxval !== 255) {
                console.error("Invalid PGM header");
                return;
              }

              const expected = width * height;
              const pixelData = bytes.slice(idx, idx + expected);
              if (pixelData.length !== expected) {
                console.error("Truncated PGM payload");
                return;
              }

              const canvas = document.getElementById("camCanvas");
              canvas.width = width;
              canvas.height = height;

              const ctx = canvas.getContext("2d");
              const img = ctx.createImageData(width, height);

              for (let i = 0; i < expected; i++) {
                const v = pixelData[i];
                const o = i * 4;
                img.data[o + 0] = v;
                img.data[o + 1] = v;
                img.data[o + 2] = v;
                img.data[o + 3] = 255;
              }

              ctx.putImageData(img, 0, 0);
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

    let content_length = data.len().to_string();
    let headers = [
        ("Content-Type", "text/html"),
        ("Content-Length", content_length.as_str()),
        ("Connection", "close"),
    ];

    let mut response = request.into_response(200, Some("OK"), &headers)?;
    response.write_all(data.as_bytes())?;
    response.flush()?;
    Ok(())
}

fn handle_camera(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // Get latest frame
    let Some(Some(frame)) = CAMERA_FRAMEBUFFER.try_take() else {
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
        frame.width(),
        frame.height(),
        frame.generation,
        &frame.data(),
    );

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
