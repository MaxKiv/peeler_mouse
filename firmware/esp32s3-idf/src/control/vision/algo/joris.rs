use std::sync::Arc;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use messenger_mouse::{VisionAlgorithmOutput, VisionMotorSetpoint};

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{IDEAL_BLADE_DEPTH_PX, VISION_BOUNDING_BOX},
};

pub static ALGO_INTERMEDIATE_WATCH: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();

// ---- tuning constants -------------------------------------------------------
/// Pixels from the frame centre that counts as "close enough" → Hold
const DEAD_ZONE_PX: i32 = 8;

/// IQR scale factor for outlier rejection (matches Python sf_iqr=1.2)
const IQR_SCALE: i32 = 12; // ×10 fixed-point, so 1.2 → 12

/// Motor speed: full-speed threshold in pixels from centre.
/// Error ≥ this → speed = 255. Scales linearly below.
const FULL_SPEED_PX: i32 = 80;

const BAD_OUTPUT: VisionAlgorithmOutput = VisionAlgorithmOutput {
    knife_setpoint: None,
    target_blade_depth_px: IDEAL_BLADE_DEPTH_PX,
    current_blade_depth_px: None,
};

/// Run the cable-edge detection on one GRAYSCALE frame.
///
/// Returns the direction and magnitude the motor should move so that the
/// detected dark edge is centred horizontally in the frame.
///
/// Returns `None` when the frame format is not GRAYSCALE or too few valid
/// rows survive outlier rejection to make a reliable estimate.
pub fn vision_joris(frame: Arc<FrameBufferView>) -> VisionAlgorithmOutput {
    // Safety: we only read the pixel buffer, never write it.
    let buf: &[u8] = unsafe {
        let fb_ptr = *frame.fb.fb; // *mut camera_fb_t
        std::slice::from_raw_parts((*fb_ptr).buf, (*fb_ptr).len)
    };

    let w = frame.width;
    let h = frame.height;

    // sanity check frame width & height
    if w < 3 || h < 1 || buf.len() < w * h {
        log::error!(
            "VISION: bad width: {} and/or height: {} and/or buf.len(): {}",
            w,
            h,
            buf.len()
        );
        return BAD_OUTPUT;
    }

    // -- stage 1: find per-colum first order derivate peak positions --------------
    //
    // For each column we compute the first order derivative using finite difference
    //   diff[col] = -(pixel[col+1] - pixel[col])
    // and locate the column with the largest positive value (bright -> dark edge).

    // Vertical forward difference peak row indices
    let mut peaks_rows: [usize; VISION_BOUNDING_BOX.width] = [0usize; VISION_BOUNDING_BOX.width];
    // Vertical forward difference peak values
    let mut peaks: [i32; VISION_BOUNDING_BOX.width] = [0i32; VISION_BOUNDING_BOX.width];

    // Iterate (BB, BB+1) rows
    for (row_idx, (curr_row, next_row)) in frame
        .bb_rows(VISION_BOUNDING_BOX)
        .zip(frame.bb_rows(VISION_BOUNDING_BOX).skip(1))
        .enumerate()
    {
        // Iterate colums
        for (col_idx, (curr_col, next_col)) in curr_row.iter().zip(next_row).enumerate() {
            // Compute forward difference
            let forward_diff: i32 = (next_col - curr_col) as i32;

            // Track peaks: largest forward difference per column
            if forward_diff > peaks[col_idx] {
                peaks[col_idx] = forward_diff;
                peaks_rows[col_idx] = row_idx;
            }
        }
    }

    // -- stage 2: find average row index of vertical derivate peak

    let avg_peak_row_idx: usize =
        (peaks_rows.iter().sum() + VISION_BOUNDING_BOX.width / 2) / VISION_BOUNDING_BOX.width;

    // Negative error  → edge is to the LEFT of centre  → knife needs to go DOWN
    // (flip the sign convention here to match your motor wiring)

    let centre_px = (w / 2) as i32;
    let error = edge_px - centre_px; // signed pixels

    if error.abs() <= DEAD_ZONE_PX {
        return Some(VisionMotorSetpoint::Hold);
    }

    // Map error -> speed 0..=255, clamped.
    let speed = ((error.abs() * 255) / FULL_SPEED_PX).clamp(0, 255) as u8;

    let knife_setpoint = if error > 0 {
        VisionMotorSetpoint::Up(speed)
    } else {
        VisionMotorSetpoint::Down(speed)
    };

    VisionAlgorithmOutput {
        knife_setpoint: Some(knife_setpoint),
        target_blade_depth_px: IDEAL_BLADE_DEPTH_PX,
        current_blade_depth_px: None,
    }
}
