//! Supervisor module, manages HMI input
//! Outputs appstate changes, which are picked up by the main controller
//! ┌─────────────┐                ┌────┐ ┌────┐
//! │             │      xxxxxxx   │ A  │ │ B  │
//! │             │     xxx   xxx  │    │ │    │
//! │  Screen     │    xxx     xxx └────┘ └────┘
//! │             │    xxx     xxx ┌────┐ ┌────┐
//! │             │     xxx   xxx  │ C  │ │ D  │
//! │             │      xxxxxxx   │    │ │    │
//! └─────────────┘                └────┘ └────┘

use defmt::*;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};

use crate::hmi::button::BUTTON_WATCH_SIZE;
use crate::hmi::encoder::data::EncoderData;
use crate::supervisor::HmiState;
use crate::supervisor::appstate::manage_appstate;
use crate::supervisor::hmi::supervise_hmi;

pub static HMI_STATE_WATCH: Watch<Cs, HmiState, 3> = Watch::new();

pub static BUTTON_C: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static BUTTON_A: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static BUTTON_B: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static BUTTON_D: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static ENCODER_PRESSED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static ENCODER_DATA: Watch<Cs, EncoderData, 2> = Watch::new();

pub const MOTOR_SPEED_STEPS: usize = 10;
pub const MAX_ROTATION_VELOCITY_MM_PS: f32 = 10.0;
pub const MAX_TRANSLATION_VELOCITY_MM_PS: f32 = 2.0;
pub const MAX_CUT_VELOCITY_MM_PS: f32 = 10.0;

pub fn setup(spawner: &Spawner) {
    info!("Setting up Supervisor");

    spawner.spawn(supervise_hmi()).unwrap();
    spawner.spawn(manage_appstate()).unwrap();
}
