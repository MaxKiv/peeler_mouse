use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use messenger_mouse::{control_params::ControlParams, VisionAlgorithmOutput, VisionMotorSetpoint};

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{HIGH_THRESHOLD, LOW_THRESHOLD},
};

// Switches between moving up and down periodically
pub fn periodic_encoder_test(
    _: Arc<FrameBufferView>,
    control_params: &ControlParams,
) -> VisionAlgorithmOutput {
    const SPEED: u8 = 100;
    const HI: u32 = HIGH_THRESHOLD as u32 + 1;
    const LO: u32 = LOW_THRESHOLD as u32 - 1;

    static STATE: AtomicU32 = AtomicU32::new(0);
    static OUT: AtomicU32 = AtomicU32::new(LO);

    let state = STATE.fetch_add(1, Ordering::Relaxed);
    log::debug!("state: {}", state);

    if state % 10 == 0 {
        let current = OUT.load(Ordering::Relaxed);
        OUT.store(if current == LO { HI } else { LO }, Ordering::Relaxed);
    }

    let out = OUT.load(Ordering::Relaxed) as u64;

    let knife_setpoint = if out > HIGH_THRESHOLD {
        VisionMotorSetpoint::Up(SPEED)
    } else if out < LOW_THRESHOLD {
        VisionMotorSetpoint::Down(SPEED)
    } else {
        VisionMotorSetpoint::Hold
    };

    VisionAlgorithmOutput {
        knife_setpoint: Some(knife_setpoint),
        zero_line_height_px: control_params.zero_line_px,
        transition_line_height_px: None,
        tearing_detected: false,
    }
}
