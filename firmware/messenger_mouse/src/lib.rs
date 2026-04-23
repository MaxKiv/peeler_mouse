#![no_std]

pub mod encoder;
pub mod motor;

use serde::{Deserialize, Serialize};

use crate::{
    encoder::EncoderState,
    motor::{KnifeManager, MotorAction},
};

pub const REPORT_BYTES: usize = core::mem::size_of::<Report>();
pub const SETPOINT_BYTES: usize = core::mem::size_of::<Setpoint>();
pub const BAUDRATE: u32 = 115200;

pub fn serialize_report(report: Report, buf: &mut [u8]) -> postcard::Result<&mut [u8]> {
    postcard::to_slice_cobs(&report, buf)
}

pub fn deserialize_report(buf: &mut [u8]) -> postcard::Result<Report> {
    postcard::from_bytes_cobs(buf)
}

pub fn serialize_setpoint(setpoint: Setpoint, buf: &mut [u8]) -> postcard::Result<&mut [u8]> {
    postcard::to_slice_cobs(&setpoint, buf)
}

pub fn deserialize_setpoint(buf: &mut [u8]) -> postcard::Result<Setpoint> {
    postcard::from_bytes_cobs(buf)
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct Report {
    pub setpoint: Setpoint,
    pub app_state: AppState,
    pub measurements: Measurements,
}

#[derive(Debug, Deserialize, Serialize, Clone, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct Setpoint {
    pub knife_manager: KnifeManager,
    pub knife_setpoint: MotorAction,
    pub led_setpoint: LedSetpoint,
}

impl Setpoint {
    pub fn new_safe() -> Self {
        Self {
            knife_manager: KnifeManager::Manual,
            knife_setpoint: MotorAction::Hold,
            led_setpoint: LedSetpoint { brightness: 0.0 },
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct Measurements {
    pub vision_data: Option<VisionData>,
    pub knife_encoder_state: EncoderState,
}

#[derive(Debug, Deserialize, Serialize, Clone, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct LedSetpoint {
    pub brightness: f32, // Percentage brightness [0.0, 1.0]
}

#[derive(PartialEq, Clone, Copy, Deserialize, Serialize, Default, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum AppState {
    #[default]
    StandBy,
    Active,
    Fault,
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub enum VisionAlgorithmOutput {
    Up,
    #[default]
    Hold,
    Down,
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct VisionData {
    pub generation: u32,
    pub timestamp_s: i64,  /* Seconds since boot of DMA completion */
    pub timestamp_us: i32, /* Microseconds.  */
    pub camera_fps: f64,
    pub vision_output: VisionAlgorithmOutput,
}
