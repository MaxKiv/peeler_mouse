use serde::{Deserialize, Serialize};

pub const CONTROL_ZERO_LINE_DEFAULT_PX: u32 = 130;
pub const CONTROL_GAIN_DEFAULT: f32 = 10.0;
pub const CONTROL_LEAD_DEFAULT: f32 = 23.0;
pub const CONTROL_LEAD_MAX: f32 = 100.0;

/// State of all the motors on the cable peeler
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct ControlParams {
    pub zero_line_px: u32,
    pub gain: f32,
    pub lead: f32,
}

impl ControlParams {
    pub fn reset() -> Self {
        Self {
            zero_line_px: CONTROL_ZERO_LINE_DEFAULT_PX,
            gain: CONTROL_GAIN_DEFAULT,
            lead: CONTROL_LEAD_DEFAULT,
        }
    }
}
