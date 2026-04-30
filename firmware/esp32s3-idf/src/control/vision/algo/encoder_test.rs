use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use messenger_mouse::VisionAlgorithmOutput;

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{HIGH_THRESHOLD, LOW_THRESHOLD},
};

// Switches between moving up and down periodically
pub fn periodic_encoder_test(_: Arc<FrameBufferView>) -> Option<VisionAlgorithmOutput> {
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

    if out > HIGH_THRESHOLD {
        Some(VisionAlgorithmOutput::Up(SPEED))
    } else if out < LOW_THRESHOLD {
        Some(VisionAlgorithmOutput::Down(SPEED))
    } else {
        Some(VisionAlgorithmOutput::Hold)
    }
}
