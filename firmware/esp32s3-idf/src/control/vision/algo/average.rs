use std::sync::Arc;

use messenger_mouse::VisionAlgorithmOutput;

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{HIGH_THRESHOLD, LOW_THRESHOLD},
};

// Simple average
pub fn simple_average(frame: Arc<FrameBufferView>) -> Option<VisionAlgorithmOutput> {
    let mut out = frame.data().into_iter().map(|x| *x as u64).sum::<u64>();
    let frame_size = (frame.height * frame.width) as u64;
    out = out / frame_size;

    if out > HIGH_THRESHOLD {
        Some(VisionAlgorithmOutput::Up(out as u8))
    } else if out < LOW_THRESHOLD {
        Some(VisionAlgorithmOutput::Down(out as u8))
    } else {
        Some(VisionAlgorithmOutput::Hold)
    }
}
