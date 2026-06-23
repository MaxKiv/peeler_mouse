use std::sync::Arc;

use messenger_mouse::{VisionAlgorithmOutput, VisionMotorSetpoint};

use crate::{
    camera::framebuffer_view::FrameBufferView,
    control::vision::algo::{HIGH_THRESHOLD, IDEAL_BLADE_DEPTH_PX, LOW_THRESHOLD},
};

// Simple average
pub fn simple_average(frame: Arc<FrameBufferView>) -> VisionAlgorithmOutput {
    let mut out = frame.data().into_iter().map(|x| *x as u64).sum::<u64>();
    let frame_size = (frame.height * frame.width) as u64;
    out = out / frame_size;

    let knife_setpoint = if out > HIGH_THRESHOLD {
        VisionMotorSetpoint::Up(out as u8)
    } else if out < LOW_THRESHOLD {
        VisionMotorSetpoint::Down(out as u8)
    } else {
        VisionMotorSetpoint::Hold
    };

    VisionAlgorithmOutput {
        knife_setpoint: Some(knife_setpoint),
        target_blade_depth_px: IDEAL_BLADE_DEPTH_PX,
        current_blade_depth_px: None,
    }
}
