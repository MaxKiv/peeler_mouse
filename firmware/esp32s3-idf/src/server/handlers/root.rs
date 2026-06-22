use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;

#[cfg(not(feature = "streaming"))]
pub fn handle_root(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // A cursed html + javascript static webpage :)
    let data = r###"
        <!DOCTYPE html>
        <html>
          <body>
            <br>
            <canvas id="camCanvas"></canvas>
            <br>

            <script>
            // Configuration
            const UPDATE_INTERVAL_MS = 500; // How often to fetch the camera
            let fetchTimer = null;

            async function fetchFrame() {
              let res;
              try {
                // 'no-store' ensures we get fresh data from the ESP
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
                // Optional: Stop updates if we keep failing? 
                // clearInterval(fetchTimer);
                return;
              }

              // Store cam bytes in a raw byte buffer
              const buf = await res.arrayBuffer();
              const bytes = new Uint8Array(buf);

              let idx = 0;

              function skipWhitespace() {
                while (idx < bytes.length && bytes[idx] <= 32) idx++;
              }

              // Read a single token from the PGM image
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

              // Confirm PGM magic numbe
              const magic = readToken();
              if (magic !== "P5") {
                console.error("Not a P5 PGM");
                return;
              }

              // Get width, height and max pixel value
              const width = parseInt(readToken(), 10);
              const height = parseInt(readToken(), 9);
              const maxval = parseInt(readToken(), 10);

              // Check if these seem right
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

              // Resize canvas only if dimensions changed to avoid flicker/layout thrashing
              if (canvas.width !== width || canvas.height !== height) {
                canvas.width = width;
                canvas.height = height;
              }

              // Create image object on canvas
              const ctx = canvas.getContext("2d");
              const img = ctx.createImageData(width, height);

              // 1. Draw the raw image data (Grayscale -> RGB)
              for (let i = 0; i < expected; i++) {
                const v = pixelData[i];
                const o = i * 4;
                img.data[o + 0] = v;     // R
                img.data[o + 1] = v;     // G
                img.data[o + 2] = v;     // B
                img.data[o + 3] = 255;   // A
              }
              ctx.putImageData(img, 0, 0);

              // 2. Draw the Box and Line on top
              drawOverlays(ctx, width, height);
            }

            function drawLine(ctx, startX, startY, endX, endY) {
              // Draw Line
              ctx.beginPath();
              ctx.strokeStyle = "#00FF00"; // Green
              ctx.moveTo(startX, startY);
              ctx.lineTo(endX, endY);
              ctx.stroke();
            }

            function drawOverlays(ctx, w, h) {
              // Get values from inputs (defaulting to some safe values if empty)
              const boxX = 90;
              const boxY = 30;
              const boxW = 160;
              const boxH = 170;

              const startX = 0;
              const startY = height/2;
              const endX = 1000;
              const endY = height/2;

              // Reset transformation to ensure we draw on top of the image
              ctx.save();

              // Draw Box
              ctx.strokeStyle = "#00FF00"; // Bright Green
              ctx.lineWidth = 3;
              ctx.strokeRect(boxX, boxY, boxW, boxH);

              // Draw Line

              ctx.moveTo(startX, startY);
              ctx.lineTo(endX, endY);
              ctx.stroke();

              // drawLine(ctx, midlineStartX, midlineStartY, midlineEndX, midlineEndY);

              ctx.restore();
            }

            // Start continuous loop
            function startUpdates() {
              if (fetchTimer) clearInterval(fetchTimer);
              fetchTimer = setInterval(fetchFrame, UPDATE_INTERVAL_MS);
            }

            // Initial load
            startUpdates();
            </script>
          </body>
        </html>
    "###;

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

#[cfg(feature = "streaming")]
pub fn handle_root(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
    // A cursed html + javascript static webpage :)
    let data = r#"
        <!DOCTYPE html>
        <html>
        <head>
        <title>ESP32 Camera</title>
        <style>
        body {
            background: #111;
            color: white;
            text-align: center;
            font-family: sans-serif;
        }

        img {
            max-width: 90%;
            border: 3px solid #444;
            border-radius: 10px;
        }
        </style>
        </head>

        <body>

        <h1>ESP32 Camera Stream</h1>

        <img src="/camera" />

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
