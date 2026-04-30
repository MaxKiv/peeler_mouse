pub mod average;
pub mod encoder_test;
pub mod joris;

use std::sync::{
    atomic::{AtomicU32, Ordering},
    Arc,
};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use messenger_mouse::{motor::MotorAction, VisionAlgorithmOutput};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::{
    actuation::stepper::motor_task::VISION_MAX_SPEED_MM_PS,
    camera::{framebuffer::FrameBuffer, framebuffer_view::FrameBufferView},
    control::vision::{
        algo::{average::simple_average, encoder_test::periodic_encoder_test, joris::vision_joris},
        HORIZONTAL_SOBEL_KERNEL,
    },
};

enum Algo {
    PeriodicEncoderTest,
    SimpleAverage,
    Complex,
    Joris,
}

const ALGO: Algo = Algo::Joris;

const HIGH_THRESHOLD: u64 = (u8::MAX / 2 + 20) as u64;
const LOW_THRESHOLD: u64 = (u8::MAX / 2 - 20) as u64;

pub fn vision_output_to_motorcommand(algo_out: VisionAlgorithmOutput) -> MotorAction {
    match algo_out {
        VisionAlgorithmOutput::Hold => MotorAction::Hold,
        VisionAlgorithmOutput::Up(speed) => {
            MotorAction::MoveVelocity(messenger_mouse::motor::MotorVelocitySetpoint::new_forward(
                Velocity::new::<millimeter_per_second>(
                    speed as f32 / u8::MAX as f32 * VISION_MAX_SPEED_MM_PS,
                ),
            ))
        }
        VisionAlgorithmOutput::Down(speed) => {
            MotorAction::MoveVelocity(messenger_mouse::motor::MotorVelocitySetpoint::new_reverse(
                Velocity::new::<millimeter_per_second>(
                    -(speed as f32 / u8::MAX as f32 * VISION_MAX_SPEED_MM_PS),
                ),
            ))
        }
    }
}

pub async fn calculate_control_effort(frame: Arc<FrameBufferView>) -> VisionAlgorithmOutput {
    let out = match ALGO {
        Algo::SimpleAverage => simple_average(frame),
        Algo::Complex => complex_algo(frame),
        Algo::PeriodicEncoderTest => periodic_encoder_test(frame),
        Algo::Joris => vision_joris(frame),
    };

    match out {
        Some(output) => output,
        None => {
            log::error!("VISION: Algorithm returned None -> using default HOLD");
            VisionAlgorithmOutput::Hold
        }
    }
}

// 3x3 Convolution with horizontal sobel kernel to determine midline point
pub fn complex_algo(frame: Arc<FrameBufferView>) -> Option<VisionAlgorithmOutput> {
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

    if out > HIGH_THRESHOLD {
        Some(VisionAlgorithmOutput::Up(100))
    } else if out < LOW_THRESHOLD {
        Some(VisionAlgorithmOutput::Down(100))
    } else {
        Some(VisionAlgorithmOutput::Hold)
    }
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
