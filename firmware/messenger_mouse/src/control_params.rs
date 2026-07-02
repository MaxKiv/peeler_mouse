use serde::{Deserialize, Serialize};

pub const ZERO_LINE_DEFAULT_PX: u32 = 120;
pub const GAIN_DEFAULT: f32 = 1.0; // TODO
pub const LEAD_DEFAULT: f32 = 0.0; // TODO
pub const LEAD_MAX: f32 = 100.0; // TODO

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
            zero_line_px: ZERO_LINE_DEFAULT_PX,
            gain: GAIN_DEFAULT,
            lead: LEAD_DEFAULT,
        }
    }
}
