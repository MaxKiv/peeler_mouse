use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use messenger_mouse::{motor::MotorAction, VisionAlgorithmOutput};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::{
    camera::{framebuffer::FrameBuffer, framebuffer_view::FrameBufferView},
    control::vision::HORIZONTAL_SOBEL_KERNEL,
};

enum Algo {
    PeriodicEncoderTest,
    SimpleAverage,
    Complex,
}

const ALGO: Algo = Algo::PeriodicEncoderTest;
const HIGH_THRESHOLD: u64 = 200;
const LOW_THRESHOLD: u64 = 100;

const VISION_KNIFE_SPEED_MM_PS: f32 = 1.0;

pub fn vision_output_to_motorcommand(algo_out: VisionAlgorithmOutput) -> MotorAction {
    match algo_out {
        VisionAlgorithmOutput::Hold => MotorAction::Hold,
        VisionAlgorithmOutput::Up => {
            MotorAction::MoveVelocity(messenger_mouse::motor::MotorVelocitySetpoint::new_forward(
                Velocity::new::<millimeter_per_second>(VISION_KNIFE_SPEED_MM_PS),
            ))
        }
        VisionAlgorithmOutput::Down => {
            MotorAction::MoveVelocity(messenger_mouse::motor::MotorVelocitySetpoint::new_reverse(
                Velocity::new::<millimeter_per_second>(-VISION_KNIFE_SPEED_MM_PS),
            ))
        }
    }
}

pub async fn calculate_control_effort(frame: Arc<FrameBufferView>) -> VisionAlgorithmOutput {
    let out = match ALGO {
        Algo::SimpleAverage => simple_average(frame),
        Algo::Complex => complex_algo(frame),
        Algo::PeriodicEncoderTest => periodic_encoder_test(frame),
    };

    // log::info!("\n\nVISION: {}\n\n", out);

    if out > HIGH_THRESHOLD {
        VisionAlgorithmOutput::Up
    } else if out < LOW_THRESHOLD {
        VisionAlgorithmOutput::Down
    } else {
        VisionAlgorithmOutput::Hold
    }
}

// Switches between moving up and down periodically
pub fn periodic_encoder_test(_: Arc<FrameBufferView>) -> u64 {
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

    OUT.load(Ordering::Relaxed) as u64
}

// Simple average
pub fn simple_average(frame: Arc<FrameBufferView>) -> u64 {
    let mut out = frame.data().into_iter().map(|x| *x as u64).sum::<u64>();
    let frame_size = (frame.height * frame.width) as u64;
    out = out / frame_size;
    out
}

// 3x3 Convolution with horizontal sobel kernel to determine midline point
pub fn complex_algo(frame: Arc<FrameBufferView>) -> u64 {
    let out = 0;

    log::info!("VISION: starting for GEN {}", frame.generation);

    // Iterate the framebuffer in 3 row windows
    let mut rows = frame.rows();
    let r0 = rows.next();
    let r1 = rows.next();

    while let (Some(a), Some(b), Some(c)) = (r0, r1, rows.next()) {
        // log::info!("VISION: rows: \n{:?}\n{:?}\n{:?}", a, b, c);

        // 3-pixel sliding window per row
        let iter = a.windows(3).zip(b.windows(3)).zip(c.windows(3));

        for ((w0, w1), w2) in iter {
            // log::info!("VISION: 3x3 window: \n{:?}\n{:?}\n{:?}", w0, w1, w2);

            let a: &[u8; 3] = w0.try_into().unwrap();
            let b: &[u8; 3] = w1.try_into().unwrap();
            let c: &[u8; 3] = w2.try_into().unwrap();

            // 3x3 convolution
            let conv = conv3x3(a, b, c) as i32;
            if conv > 3 * 80 {
                log::info!(
                    "VISION: output {} -> horizontal edge inside 3x3 window: \n{:?}\n{:?}\n{:?}",
                    conv,
                    w0,
                    w1,
                    w2
                );
            }
        }
    }

    log::info!("VISION: outputs {}", out);

    out
}

// 3x3 Convolution
fn conv3x3(a: &[u8; 3], b: &[u8; 3], c: &[u8; 3]) -> usize {
    let rows = [a, b, c];

    rows.iter()
        .zip(HORIZONTAL_SOBEL_KERNEL.iter())
        .map(|(pix, krow)| {
            pix.iter()
                .zip(krow.iter())
                .map(|(&p, &k)| (p as usize) * k as usize)
                .sum::<usize>()
        })
        .sum()
}
