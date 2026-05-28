use std::sync::Arc;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use messenger_mouse::VisionAlgorithmOutput;

use crate::camera::framebuffer_view::FrameBufferView;

pub static ALGO_INTERMEDIATE_WATCH: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();

// ---- tuning constants -------------------------------------------------------
/// Pixels from the frame centre that counts as "close enough" → Hold
const DEAD_ZONE_PX: i32 = 8;

/// IQR scale factor for outlier rejection (matches Python sf_iqr=1.2)
const IQR_SCALE: i32 = 12; // ×10 fixed-point, so 1.2 → 12

/// Motor speed: full-speed threshold in pixels from centre.
/// Error ≥ this → speed = 255. Scales linearly below.
const FULL_SPEED_PX: i32 = 80;

/// Run the cable-edge detection on one GRAYSCALE frame.
///
/// Returns the direction and magnitude the motor should move so that the
/// detected dark edge is centred horizontally in the frame.
///
/// Returns `None` when the frame format is not GRAYSCALE or too few valid
/// rows survive outlier rejection to make a reliable estimate.
pub fn vision_joris(frame: Arc<FrameBufferView>) -> Option<VisionAlgorithmOutput> {
    // Safety: we only read the pixel buffer, never write it.
    let buf: &[u8] = unsafe {
        let fb_ptr = *frame.fb.fb; // *mut camera_fb_t
        std::slice::from_raw_parts((*fb_ptr).buf, (*fb_ptr).len)
    };

    let w = frame.width;
    let h = frame.height;

    if w < 3 || h < 1 || buf.len() < w * h {
        return None;
    }

    // ── stage 1: per-row peak positions ──────────────────────────────────────
    //
    // For each row we compute the horizontal finite difference
    //   diff[col] = -(pixel[col+1] - pixel[col])
    // and locate the column with the largest positive value (bright→dark edge).
    // Sub-pixel position is refined with a parabolic fit through the 3 samples
    // around the maximum.
    //
    // We store positions as fixed-point ×256 (i32) to stay in integer maths on
    // the ESP32S3 while still carrying sub-pixel information.

    // Pre-allocate on the stack when rows ≤ 480; heap otherwise.
    let mut peaks: Vec<i32> = Vec::with_capacity(h); // ×256 fixed-point columns

    for row in 0..h {
        let row_start = row * w;
        let row_pixels = &buf[row_start..row_start + w];

        // Find argmax of the negated forward difference.
        let mut best_col: usize = 1;
        let mut best_val: i32 = i32::MIN;

        // We skip col 0 and col w-2 so the parabola always has left and right
        // neighbours.
        for col in 1..(w - 2) {
            let diff = row_pixels[col] as i32 - row_pixels[col + 1] as i32; // negated diff
            if diff > best_val {
                best_val = diff;
                best_col = col;
            }
        }

        // Parabolic sub-pixel refinement: fit y = a·x² + b·x + c through the
        // three samples at (best_col-1, best_col, best_col+1), then compute the
        // vertex x = -b / (2a).
        //
        // With y₀ = left, y₁ = centre, y₂ = right and x₀ = best_col-1:
        //   a = (y₀ - 2·y₁ + y₂) / 2
        //   b = (y₂ - y₀) / 2
        //   vertex_offset = -b / (2a) = (y₀ - y₂) / (2·(y₀ - 2·y₁ + y₂))
        //
        // We scale by 256 to avoid floats.

        let y0 = row_pixels[best_col - 1] as i32 - row_pixels[best_col] as i32; // diff at col-1
        let y1 = row_pixels[best_col] as i32 - row_pixels[best_col + 1] as i32; // diff at col (our max)
        let y2 = row_pixels[best_col + 1] as i32 - row_pixels[best_col + 2] as i32; // diff at col+1

        // denom = y0 - 2·y1 + y2  (= 2a, always ≤ 0 at a maximum)
        let denom = y0 - 2 * y1 + y2;

        let peak_fp = if denom != 0 {
            // offset×256 = (y0 - y2)·128 / denom
            // Clamp to ±1 pixel to avoid wild swings on flat rows.
            let numer = (y0 - y2) * 128; // ×256/2
            let offset = numer / denom; // already ×256 units
            let offset = offset.clamp(-256, 256);
            (best_col as i32) * 256 + offset
        } else {
            (best_col as i32) * 256
        };

        peaks.push(peak_fp);
    }

    // ── stage 2: IQR outlier rejection ───────────────────────────────────────
    //
    // Sort a copy, find Q1 and Q3, then keep only rows whose peak is within
    //   [median - IQR_SCALE/10 × IQR,  median + IQR_SCALE/10 × IQR]

    let mut sorted = peaks.clone();
    sorted.sort_unstable();

    let n = sorted.len();
    let q1 = sorted[n / 4];
    let q3 = sorted[3 * n / 4];
    let iqr = q3 - q1; // in ×256 fixed-point

    // threshold = IQR_SCALE * iqr / 10
    let threshold = IQR_SCALE * iqr / 10;

    let median_all = sorted[n / 2];
    let lo = median_all - threshold;
    let hi = median_all + threshold;

    // Collect valid peaks (those inside the fence).
    let valid: Vec<i32> = peaks
        .iter()
        .copied()
        .filter(|&p| p >= lo && p <= hi)
        .collect();

    if valid.len() < (h / 4).max(4) {
        // Fewer than 25% of rows survived — image probably has no clear edge.
        return None;
    }

    // ── stage 3: median of valid peaks → edge position ───────────────────────

    let mut valid_sorted = valid.clone();
    valid_sorted.sort_unstable();
    let edge_fp = valid_sorted[valid_sorted.len() / 2]; // ×256 fixed-point
    let edge_px = edge_fp / 256; // integer pixel column

    // ── stage 4: compare to frame centre → motor command ─────────────────────
    //
    // Positive error  → edge is to the RIGHT of centre → knife needs to go UP
    // Negative error  → edge is to the LEFT of centre  → knife needs to go DOWN
    // (flip the sign convention here to match your motor wiring)

    let centre_px = (w / 2) as i32;
    let error = edge_px - centre_px; // signed pixels

    if error.abs() <= DEAD_ZONE_PX {
        return Some(VisionAlgorithmOutput::Hold);
    }

    // Map |error| → speed 0..=255, clamped.
    let speed = ((error.abs() * 255) / FULL_SPEED_PX).clamp(0, 255) as u8;

    let output = if error > 0 {
        VisionAlgorithmOutput::Up(speed)
    } else {
        VisionAlgorithmOutput::Down(speed)
    };

    Some(output)
}
