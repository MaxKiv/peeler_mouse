#![no_std]

pub mod control_params;
pub mod encoder;
pub mod motor;

use serde::{Deserialize, Serialize};

use crate::{
    control_params::ControlParams,
    encoder::EncoderState,
    motor::{ControlMode, MotorAction, MotorSetpoints},
};

pub enum FrameSize {
    FramesizeQvga, // 320x240
}

impl FrameSize {
    pub const fn get_dimensions(&self) -> (usize, usize) {
        match self {
            FrameSize::FramesizeQvga => (320, 240),
        }
    }
}

pub const REPORT_BYTES: usize = core::mem::size_of::<Esp32Report>();
pub const SETPOINT_BYTES: usize = core::mem::size_of::<Esp32Setpoint>();
pub const BAUDRATE: u32 = 115200;
pub const LED_BRIGHTNESS: f32 = 0.005;
// pub const LED_BRIGHTNESS: f32 = 0.0;
pub const FRAME_SIZE: FrameSize = FrameSize::FramesizeQvga; // 320x240

pub fn serialize_report(report: Esp32Report, buf: &mut [u8]) -> postcard::Result<&mut [u8]> {
    postcard::to_slice_cobs(&report, buf)
}

pub fn deserialize_report(buf: &mut [u8]) -> postcard::Result<Esp32Report> {
    postcard::from_bytes_cobs(buf)
}

pub fn serialize_setpoint(setpoint: Esp32Setpoint, buf: &mut [u8]) -> postcard::Result<&mut [u8]> {
    postcard::to_slice_cobs(&setpoint, buf)
}

pub fn deserialize_setpoint(buf: &mut [u8]) -> postcard::Result<Esp32Setpoint> {
    postcard::from_bytes_cobs(buf)
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Cycle Report produced by esp32
/// Collects its current setpoint, status, measurements and corresponding controller output
pub struct Esp32Report {
    pub current_setpoint: Esp32Setpoint,
    pub status: Esp32Status,
    pub measurements: Measurements,
    pub control_output: ControlOutput,
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Esp32 controller output
pub enum ControlOutput {
    #[default]
    Manual,
    Vision(ControlEffort),
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Esp32 vision controller output
pub struct ControlEffort {
    pub motor_setpoints: MotorSetpoints,
    pub led: LedSetpoint,
}

#[derive(Debug, Deserialize, Serialize, Clone, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Esp32 setpoint, produced by stm32
pub struct Esp32Setpoint {
    pub control_mode: ControlMode,
    pub control_params: ControlParams,
    pub knife_setpoint: MotorAction,
}

impl Esp32Setpoint {
    pub fn new_safe() -> Self {
        Self {
            control_mode: ControlMode::Manual,
            knife_setpoint: MotorAction::Coast,
            control_params: ControlParams::reset(),
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Collection of all measurements in a single discrete step by the esp32
pub struct Measurements {
    pub vision_data: Option<VisionData>,
    pub knife_encoder_state: EncoderState,
}

#[derive(Debug, Deserialize, Serialize, Clone, Default, PartialEq)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// LED setpoint
pub struct LedSetpoint {
    pub brightness: f32, // Percentage brightness [0.0, 1.0]
}

#[derive(PartialEq, Clone, Copy, Deserialize, Serialize, Default, Debug)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Esp32 firmware status
pub enum Esp32Status {
    #[default]
    StandBy,
    Active,
    Fault,
}

/*
    Blade Depth Explanation

                 \\                                                       // -
                  \\                       KNIFE                         //  |
                   \\    Knife blade                                    //   |
    Zero Line     ->\\                                                 //    | <- Blade
#################### \\ <- Current Transition Line ~= 20% blade depth //     |
##################### \\---------------------------------------------//      |
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~... -
                                                                             |
                                                                             | <- Cable Inner Layer
                                                                             |
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~... -
####################### CABLE ISOLATION #################################... | <- Cable Outer Layer
#########################################################################... -

    Transition Line should be controlled to blade depth / 2.
    Else we are cutting too deep or shallow.
*/
#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
pub struct VisionAlgorithmOutput {
    pub knife_setpoint: Option<VisionMotorSetpoint>,
    /// Statically determined blade depth target
    /// Should correspond to blade depth /2
    /// Depends on camera and blade geometry
    pub zero_line_height_px: u32,
    /// Current calculated blade depth
    pub transition_line_height_px: Option<u32>,
    /// Was tearing detected during this control loop pass?
    pub tearing_detected: bool,
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Esp32 Vision algorithm output
pub enum VisionMotorSetpoint {
    Up(u8),
    #[default]
    Hold,
    Down(u8),
}

impl core::fmt::Display for VisionMotorSetpoint {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            VisionMotorSetpoint::Up(value) => write!(f, "Up({})", value),
            VisionMotorSetpoint::Hold => write!(f, "Hold"),
            VisionMotorSetpoint::Down(value) => write!(f, "Down({})", value),
        }
    }
}

#[derive(Deserialize, Serialize, Clone, Debug, Default)]
#[cfg_attr(feature = "use-defmt", derive(defmt::Format))]
/// Collection of vision data used by the esp32 to calculate control effort
pub struct VisionData {
    pub generation: u32,                      // Frame generation
    pub timestamp_s: i64,                     /* Seconds since boot of DMA completion */
    pub timestamp_us: i32,                    /* Microseconds.  */
    pub camera_fps: f64,                      // Camera FPS at frame collection time
    pub vision_output: VisionAlgorithmOutput, // Calculated vision algo output
}
