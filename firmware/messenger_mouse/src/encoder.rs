use serde::{Deserialize, Serialize};

/// Maximum angle value (14-bit: 0-16383, representing 0-360°)
pub const ANGLE_MAX: u16 = 0x3FFF + 1;
pub const MAX_COUNT: i32 = ANGLE_MAX as i32;
pub const WRAP_THRESHOLD: i32 = MAX_COUNT / 2;

/// mm per revolution of worm wheel
pub const KNIFE_AXIS_LEAD_MM: f32 = 0.7;
// gear ratio -> revolutions of stepper axis per revolution of worm wheel
pub const KNIFE_AXIS_GEAR_RATIO: f32 = 20.0;
// Stepperdriver Microstep settings -> STEP pulses per driver full step, depends on <
// TMC2209 DS: https://www.analog.com/media/en/technical-documentation/data-sheets/TMC2209_datasheet_rev1.08.pdf
pub const KNIFE_AXIS_MICROSTEPS_PER_STEP: f32 = 8.0;
// How many steps in a full axis rotation? 1.8deg per step according to nema 11 datasheet
// NEMA11 DS: https://www.mouser.com/pdfdocs/nema11-amt112s.pdf
pub const KNIFE_AXIS_STEPS_PER_ROTATION: f32 = 20.0;

/// Knife position in mm
pub type KnifePosition = f32;

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum EncoderValidity {
    #[default]
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

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct EncoderData {
    pub angle: i32,
    pub revolution: i32,
}

impl EncoderData {
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

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct EncoderState {
    pub encoder_data: EncoderData,
    pub validity: EncoderValidity,
}

impl EncoderState {
    pub fn new() -> Self {
        Self {
            encoder_data: EncoderData::new(),
            validity: EncoderValidity::NotHomedYet,
        }
    }

    pub fn get_position(&self) -> KnifePosition {
        let abs = self.encoder_data.absolute_count();
        let mm = abs as f32 * KNIFE_AXIS_LEAD_MM;

        mm
    }

    pub fn on_homed(&mut self) {
        self.validity = EncoderValidity::Valid;
        self.encoder_data.reset();
    }

    pub fn on_home_lost(&mut self) {
        self.validity = EncoderValidity::NotHomedYet;
    }
}
