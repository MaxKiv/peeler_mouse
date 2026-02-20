use esp_idf_hal::io::Write;
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use log::*;

pub fn handle_root(request: Request<&mut EspHttpConnection<'_>>) -> anyhow::Result<()> {
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
