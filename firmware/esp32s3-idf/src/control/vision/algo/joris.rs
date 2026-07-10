use std::{
    ops::{Add, Sub},
    sync::Arc,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use messenger_mouse::{
    control_params::{ControlParams, CONTROL_ZERO_LINE_DEFAULT_PX},
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
const DEAD_ZONE_ROWS: i32 = 1;

/// Motor speed: full-speed threshold in pixels from centre.
/// Error ≥ this -> speed = 255. Scales linearly below.
const FULL_SPEED_PX: i32 = 80;
const GAIN_FACTOR: f32 = 25.5;

/// Determine the transition line (black -> white) of one grayscale image frame
pub fn vision_joris(
    frame: Arc<FrameBufferView>,
    control_params: &ControlParams,
) -> VisionAlgorithmOutput {
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

    // 1: find per-colum vertical gradient peak positions
    // For each column we compute the gradient using finite difference
    // using diff[col] = -(pixel[col+1] - pixel[col])
    // and locate the column with the largest positive value.

    // Initialise vertical gradent peak row indices
    let mut peaks: [VerticalGradient; VISION_BOUNDING_BOX.width] = [VerticalGradient {
        value: 0,
        row_idx: 0,
    };
        VISION_BOUNDING_BOX.width];
    let mut tearing_cnt = 0;

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

            // Track peaks: largest vertical gradient per column
            if forward_diff > peaks[col_idx].value {
                peaks[col_idx] = VerticalGradient {
                    value: forward_diff,
                    row_idx,
                };
            }
        }
    }

    // 3: Tearing detection
    let variance = welford_variance(&peaks);
    let std = variance.isqrt() as i32;
    let mut tearing_cnt = 0;
    for p in peaks {
        if p.value > 5 * std {
            tearing_cnt += 1;
        }
    }
    log::info!(
        "VISION: frame gen {} (variance: {}, std: {}, # grad > 5*std: {})",
        frame.generation,
        variance,
        std,
        tearing_cnt
    );
    if tearing_cnt > VISION_BOUNDING_BOX.width * 6 / 10 {
        // Tearing detected
        log::warn!(
            "VISION: TEARING DETECTED in frame gen {} (variance: {}, std: {}, # grad > 5*std: {})",
            frame.generation,
            variance,
            std,
            tearing_cnt
        );

        // If tearing happend, discard current frame
        // return VisionAlgorithmOutput {
        //     tearing_detected: true,
        //     knife_setpoint: None,
        //     zero_line_height_px: zero_line,
        //     transition_line_height_px: None,
        // };
    }

    // 4: IQR filtering
    // Sort peaks on vertical gradient values
    peaks.sort_by(|a, b| a.value.cmp(&b.value));
    // IQR filter on gradient values
    let filtered_size = filter_iqr_inplace(&mut peaks);

    // 5: find median of iqr filter result, this is our transition line
    let median = peaks[filtered_size / 2];

    log::info!("5: median: {:?} (tearing_cnt: {})", median, tearing_cnt);

    // 6: find transition line & its delta to zero line
    let transition_line: u32 = median.row_idx as u32;
    let delta: i32 = zero_line as i32 - transition_line as i32;

    log::info!("6: transition_line: {}, delta: {}", transition_line, delta);

    //  7: Calculate control output
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
            tearing_detected: false,
            transition_line_height_px: Some(transition_line),
        }
    }
}

/// IQR filters peaks, returns size of filtered collection
fn filter_iqr_inplace(arr: &mut [VerticalGradient]) -> usize {
    let q1 = arr[arr.len() / 4].value;
    let q3 = arr[arr.len() * 3 / 4].value;
    let iqr = q3 - q1;

    let mut write = 0;
    for read in 0..arr.len() {
        if iqr_filter(arr[read].value, q1, q3, iqr) {
            if write != read {
                arr.swap(write, read);
            }
            write += 1;
        }
    }
    write // number of elements kept
}

fn iqr_filter<T>(val: T, q1: T, q3: T, iqr: T) -> bool
where
    T: Add<Output = T> + Sub<Output = T> + PartialOrd + Clone + Copy,
{
    val < (q3 + iqr) && val > (q1 - iqr)
}

fn welford_variance(arr: &[VerticalGradient]) -> i64 {
    let mut mean = 0i64;
    let mut m2 = 0i64;

    for (i, &x) in arr.iter().enumerate() {
        let delta = x.value as i64 - mean;
        mean += delta / ((i + 1) as i64);
        m2 += delta * (x.value as i64 - mean);
    }

    let variance = m2 / (arr.len() as i64);
    variance
}
