// web/app.js

const UPDATE_INTERVAL_MS = 500;
let fetchTimer = null;

function startUpdates() {
  if (fetchTimer) clearInterval(fetchTimer);
  fetchTimer = setInterval(fetchFrame, UPDATE_INTERVAL_MS);
}

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

  const buf = await res.arrayBuffer();
  const bytes = new Uint8Array(buf);

  let idx = 0;

  function skipWhitespace() {
    while (idx < bytes.length && bytes[idx] <= 32) idx++;
  }

  function readToken() {
    skipWhitespace();
    if (bytes[idx] === 35) {
      // '#'
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

  // Resize canvas only if dimensions changed to avoid flicker/layout thrashing
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }

  const ctx = canvas.getContext("2d");
  const img = ctx.createImageData(width, height);

  // 1. Draw the raw image data (Grayscale -> RGB)
  for (let i = 0; i < expected; i++) {
    const v = pixelData[i];
    const o = i * 4;
    img.data[o + 0] = v; // R
    img.data[o + 1] = v; // G
    img.data[o + 2] = v; // B
    img.data[o + 3] = 255; // A
  }
  ctx.putImageData(img, 0, 0);

  // 2. Draw the Box and Line on top
  drawOverlays(ctx, width, height);
}

function drawOverlays(ctx, w, h) {
  // Get values from inputs (defaulting to some safe values if empty)
  const boxX = parseInt(document.getElementById("boxX").value) || w / 2 - 25;
  const boxY = parseInt(document.getElementById("boxY").value) || h / 2 - 25;
  const boxW = 50;
  const boxH = 50;

  const lineX1 = parseInt(document.getElementById("lineX1").value) || 0;
  const lineY1 = parseInt(document.getElementById("lineY1").value) || 0;
  const lineX2 = parseInt(document.getElementById("lineX2").value) || w;
  const lineY2 = parseInt(document.getElementById("lineY2").value) || h;

  // Reset transformation to ensure we draw on top of the image
  ctx.save();

  // Draw Box
  ctx.strokeStyle = "#00FF00"; // Bright Green
  ctx.lineWidth = 2;
  ctx.strokeRect(boxX, boxY, boxW, boxH);

  // Draw Line
  ctx.beginPath();
  ctx.moveTo(lineX1, lineY1);
  ctx.lineTo(lineX2, lineY2);
  ctx.strokeStyle = "#FF0000"; // Red
  ctx.stroke();

  ctx.restore();
}

// Expose to global scope so HTML can call them
window.fetchFrame = fetchFrame;
window.startUpdates = startUpdates;
window.sendSetpoint = (depth) => {
  /* ... */
};
