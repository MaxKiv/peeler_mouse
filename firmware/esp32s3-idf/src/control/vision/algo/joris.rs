use std::sync::Arc;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use messenger_mouse::{
    control_params::{ControlParams, ZERO_LINE_DEFAULT_PX},
    VisionAlgorithmOutput, VisionMotorSetpoint,
};

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{
        vertical_gradient::VerticalGradient, VISION_BOUNDING_BOX, VISION_TEARING_DETECTION_NUM_COL,
        VISION_TEARING_GRADIENT_VALUE_THRESHOLD,
    },
};

pub static ALGO_INTERMEDIATE_WATCH: Watch<Cs, Arc<FrameBufferView>, 1> = Watch::new();

// ---- tuning constants -------------------------------------------------------
/// Pixels from the zero line that counts as "close enough" -> Hold current depth
const DEAD_ZONE_ROWS: i32 = 8;

/// Motor speed: full-speed threshold in pixels from centre.
/// Error ≥ this -> speed = 255. Scales linearly below.
const FULL_SPEED_PX: i32 = 80;
const GAIN_FACTOR: f32 = 25.5;

/// Run the cable-edge detection on one GRAYSCALE frame.
///
/// Returns the direction and magnitude the motor should move so that the
/// detected dark edge is centred horizontally in the frame.
///
/// Returns `None` when the frame format is not GRAYSCALE or too few valid
/// rows survive outlier rejection to make a reliable estimate.
pub fn vision_joris(
    frame: Arc<FrameBufferView>,
    control_params: &ControlParams,
) -> VisionAlgorithmOutput {
    // Safety: we only read the pixel buffer, never write it.
    let buf: &[u8] = unsafe {
        let fb_ptr = *frame.fb.fb; // *mut camera_fb_t
        std::slice::from_raw_parts((*fb_ptr).buf, (*fb_ptr).len)
    };

    let w = frame.width;
    let h = frame.height;
    let zero_line = control_params.zero_line_px;

    // sanity check frame width & height
    if w < 3 || h < 1 || buf.len() < w * h {
        log::error!(
            "VISION: bad width: {} and/or height: {} and/or buf.len(): {}",
            w,
            h,
            buf.len()
        );
        // Return a default output
        return VisionAlgorithmOutput {
            knife_setpoint: Some(VisionMotorSetpoint::Hold),
            zero_line_height_px: zero_line,
            tearing_detected: false,
            transition_line_height_px: None,
        };
    }

    // -- 1: find per-colum vertical gradient peak positions --------------
    //
    // For each column we compute the gradient using finite difference
    //   diff[col] = -(pixel[col+1] - pixel[col])
    // and locate the column with the largest positive value.

    // Initialise vertical gradent peak row indices
    let mut peaks: [VerticalGradient; VISION_BOUNDING_BOX.width] = [VerticalGradient {
        value: 0,
        row_idx: 0,
    };
        VISION_BOUNDING_BOX.width];
    let mut tearing_cnt = 0;
    let mut tearing_detected: bool = false;

    // Iterate a set of (i, i+1) bounding box rows
    for (row_idx, (curr_row, next_row)) in frame
        .bb_rows(VISION_BOUNDING_BOX)
        .zip(frame.bb_rows(VISION_BOUNDING_BOX).skip(1))
        .enumerate()
    {
        // Iterate colums
        for (col_idx, (curr_col, next_col)) in curr_row.iter().zip(next_row).enumerate() {
            // Compute vertical gradient using forward difference
            let forward_diff: i32 = *next_col as i32 - *curr_col as i32;

            // -- 2: Tearing detection
            // If vertical gradient exceeds tearing threshold, mark it
            // If 80% of columns have are marked, tearing happend
            // If tearing happend, discard current frame
            if forward_diff > VISION_TEARING_GRADIENT_VALUE_THRESHOLD {
                tearing_cnt += 1;
                if tearing_cnt > VISION_TEARING_DETECTION_NUM_COL {
                    tearing_detected = true;
                    // Tearing detected!
                    // return VisionAlgorithmOutput {
                    //     tearing_detected: true,
                    //     knife_setpoint: None,
                    //     zero_line_height_px: VISION_ZERO_LINE_HEIGHT,
                    //     transition_line_height_px: None,
                    // };
                }
            }

            // Track peaks: largest vertical gradient per column
            if forward_diff > peaks[col_idx].value {
                peaks[col_idx] = VerticalGradient {
                    value: forward_diff,
                    row_idx,
                };
            }
        }
    }
    // -- 3: find median row index of vertical derivate peaks
    peaks.sort_by(|a, b| a.value.cmp(&b.value));
    let median = peaks[peaks.len() / 2];

    log::info!("3: median: {:?}", median);

    // -- 4: find transition line & its delta to zero line
    let transition_line: u32 = median.row_idx as u32;
    let delta: i32 = zero_line as i32 - transition_line as i32;

    log::info!("4: transition_line, delta: {}, {}", transition_line, delta);

    // -- 5: Calculate control output
    let abs_delta = delta.abs();
    // Is delta within Dead zone?
    if abs_delta <= DEAD_ZONE_ROWS {
        // Delta within dead zone, hold current depth
        VisionAlgorithmOutput {
            knife_setpoint: Some(VisionMotorSetpoint::Hold),
            zero_line_height_px: zero_line,
            tearing_detected: false,
            transition_line_height_px: Some(transition_line),
        }
    } else {
        // Delta outside dead zone, adjust knife
        // Linear map delta -> knife speed 0..=255, clamped.
        let speed = ((abs_delta * (control_params.gain * GAIN_FACTOR) as i32) / FULL_SPEED_PX)
            .clamp(0, 255) as u8;
        let knife_setpoint = if delta > 0 {
            VisionMotorSetpoint::Up(speed)
        } else {
            VisionMotorSetpoint::Down(speed)
        };

        VisionAlgorithmOutput {
            knife_setpoint: Some(knife_setpoint),
            zero_line_height_px: zero_line,
            tearing_detected,
            transition_line_height_px: Some(transition_line),
        }
    }
}
