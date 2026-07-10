use serde::{Deserialize, Serialize};

pub const MAX_ROTATION_VELOCITY_MM_PS: f32 = 10.0;
pub const MAX_TRANSLATION_VELOCITY_MM_PS: f32 = 2.0;
pub const MAX_CUT_VELOCITY_MM_PS: f32 = 10.0;

/// NOTE: Critical Control parameters
pub const CONTROL_ZERO_LINE_DEFAULT_PX: u32 = 132;
pub const CONTROL_GAIN_DEFAULT: f32 = 12.0;
pub const CONTROL_LEAD_DEFAULT: f32 = 1.00;
pub const CONTROL_LEAD_MAX: f32 = 100.0;
pub const CONTROL_SPEED_DEFAULT: f32 = 2.0;
pub const CONTROL_SPEED_MAX: f32 = MAX_ROTATION_VELOCITY_MM_PS;

/// Amount of encoder resolve events to wait to complete seek reverse phase
/// NOTE: the duration of this heavily depends on ENCODER_STALL_DEBOUNCE_DURATION
pub const SEEK_REVERSE_RESOLVE_COUNT: usize = 8;
/// Amount of encoder resolve events to wait to complete seek stall phase
/// NOTE: the duration of this heavily depends on ENCODER_STALL_DEBOUNCE_DURATION
pub const SEEK_FORWARD_STALL_COUNT: usize = 2;
/// Amount of encoder resolve events to wait to complete homing stall phase
pub const HOMING_DETECTED_STALL_COUNT: usize = 2;

/// State of all the motors on the cable peeler
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct ControlParams {
    pub zero_line_px: u32,
    pub gain: f32,
    pub speed: f32,
    pub lead: f32,
}

impl ControlParams {
    pub fn reset() -> Self {
        Self {
            zero_line_px: CONTROL_ZERO_LINE_DEFAULT_PX,
            gain: CONTROL_GAIN_DEFAULT,
            lead: CONTROL_LEAD_DEFAULT,
            speed: CONTROL_SPEED_DEFAULT,
        }
    }
}
