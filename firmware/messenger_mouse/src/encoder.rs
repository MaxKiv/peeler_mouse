use serde::{Deserialize, Serialize};

/// Maximum angle value (14-bit: 0-16383, representing 0-360°)
pub const ANGLE_MAX: u16 = 0x3FFF + 1;
pub const MAX_COUNT: i32 = ANGLE_MAX as i32;
pub const WRAP_THRESHOLD: i32 = MAX_COUNT / 2;

/// mm per revolution
pub const KNIFE_AXIS_LEAD: f32 = 0.7;

/// Knife position in mm
pub type KnifePosition = f32;

#[derive(Deserialize, Serialize, Clone, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum EncoderValidity {
    NotHomedYet,
    Valid,
    EncoderError(EncoderError),
}

#[derive(Deserialize, Serialize, Clone, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum EncoderError {
    /// Communication error with the sensor
    Communication,
    /// Parity error in received data
    ParityError,
    /// Error flag set by the sensor (invalid command or parity error)
    SensorError,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct EncoderState {
    pub angle: i32,
    pub revolution: i32,
}

impl EncoderState {
    pub fn new() -> Self {
        Self {
            angle: 0,
            revolution: 0,
        }
    }

    pub fn reset(&mut self) {
        self.angle = 0;
        self.revolution = 0;
    }

    pub fn update(&mut self, angle: u16) {
        let delta: i32 = self.angle - angle as i32;
        if delta > WRAP_THRESHOLD {
            // large positive delta => encoder wrapped from High to Low => forward
            self.revolution += 1;
        } else if delta < -WRAP_THRESHOLD {
            // large negative delta => encoder wrapped from Low to High => reverse
            self.revolution -= 1;
        }

        // Remember current measured angle
        self.angle = angle as i32;
    }

    // Returns absolute count
    pub fn absolute_count(&self) -> i32 {
        self.revolution * MAX_COUNT + self.angle
    }
}

#[derive(Deserialize, Serialize, Clone, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct KnifeState {
    pub encoder_state: EncoderState,
    pub validity: EncoderValidity,
}

impl KnifeState {
    pub fn new() -> Self {
        Self {
            encoder_state: EncoderState::new(),
            validity: EncoderValidity::NotHomedYet,
        }
    }

    pub fn get_position(&self) -> KnifePosition {
        let abs = self.encoder_state.absolute_count();
        let mm = abs as f32 * KNIFE_AXIS_LEAD;

        mm
    }
}
