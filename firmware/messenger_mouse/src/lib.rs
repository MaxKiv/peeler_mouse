#![no_std]

use defmt::Format;
use serde::{Deserialize, Serialize};
use uom::si::f32::Length;
use uom::si::length::millimeter;

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

#[derive(Deserialize, Serialize, Clone, Format, Debug)]
pub struct Report {
    pub setpoint: Setpoint,
    pub app_state: AppState,
    pub measurements: Measurements,
}

#[derive(Debug, Deserialize, Serialize, Clone, Default)]
pub struct Setpoint {
    pub enable: bool,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct Measurements {
    /// microseconds since boot of mcu
    pub timestamp: u64,

    pub camera_fps: f32,

    pub current_knife_depth: Length,

    pub controller_output: VisionAlgorithmOutput,
}

#[derive(Deserialize, Serialize, Clone, Debug, Format)]
pub enum VisionAlgorithmOutput {
    Up,
    Hold,
    Down,
}

#[derive(PartialEq, Clone, Copy, Deserialize, Serialize, Format, Default, Debug)]
pub enum AppState {
    #[default]
    StandBy,
    Running, // Frequency in Hz
    Fault,
}

// defmt Format impls from here
impl Format for Measurements {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "Measurement({}ms -> depth {} - {:?})",
            self.timestamp,
            self.current_knife_depth.get::<millimeter>(),
            self.controller_output,
        );
    }
}

impl Format for Setpoint {
    fn format(&self, fmt: defmt::Formatter) {
        use defmt::write;

        write!(fmt, "Knife Controller -> ");
        match &self.enable {
            true => write!(fmt, "ENABLED",),
            false => write!(fmt, "DISABLED"),
        };
    }
}
