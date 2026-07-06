pub mod average;
pub mod encoder_test;
pub mod joris;
pub mod vertical_gradient;

use std::sync::Arc;

use messenger_mouse::{
    control_params::{ControlParams, LEAD_MAX},
    motor::{MotorAction, MotorDirection, MotorSetpoints, MotorVelocitySetpoint},
    ControlEffort, LedSetpoint, VisionAlgorithmOutput, VisionMotorSetpoint, LED_BRIGHTNESS,
};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::{
    actuation::stepper::motor_task::VISION_MAX_SPEED_MM_PS,
    camera::{
        framebuffer_view::{BoundingBox, FrameBufferView, Pixel},
        FRAME_SIZE,
    },
    control::vision::{
        algo::{
            average::simple_average, encoder_test::periodic_encoder_test, joris::vision_joris,
            vertical_gradient::VerticalGradient,
        },
        HORIZONTAL_SOBEL_KERNEL,
    },
};

/// Enum of possible vision algorithms
pub enum Algo {
    PeriodicEncoderTest,
    SimpleAverage,
    Complex,
    Joris,
}

// QVGA (width, height) = (320x240)
const HIGH_THRESHOLD: u64 = (u8::MAX / 2 + 20) as u64;
const LOW_THRESHOLD: u64 = (u8::MAX / 2 - 20) as u64;

/// Which vision algorithm to use
pub const ALGO: Algo = Algo::Joris;

/// Vertical gradient threshold value to decide tearing happend in this column
pub const VISION_TEARING_GRADIENT_VALUE_THRESHOLD: i32 = 100;
/// Decide tearing happend if 80% of columns have a vertical gradient > tearing threshold
pub const VISION_TEARING_DETECTION_NUM_COL: usize =
    (FRAME_SIZE.get_dimensions().0 as f32 * 0.8) as usize;

/// Pixel to place top left corner of bounding box
pub const VISION_BOUNDING_BOX_START_PIXEL: Pixel = Pixel {
    // x: (FRAME_SIZE.get_dimensions().0 as f32 * 0.3) as usize,
    x: (FRAME_SIZE.get_dimensions().0 as f32 * 0.3) as usize,
    y: 0,
};
/// Bounding box size definition
pub const VISION_BOUNDING_BOX: BoundingBox = BoundingBox {
    start: VISION_BOUNDING_BOX_START_PIXEL, // Place top left corner of BB at this pixel
    width: (FRAME_SIZE.get_dimensions().0 as f32 * 0.40) as usize, // BB width: 40% of image width
    height: FRAME_SIZE.get_dimensions().1,  // BB height: 100% of image height
};

const VISION_TEARING_VEL_ROT_MM_PS: f32 = 1.0;
const VISION_TEARING_VEL_LIN_MM_PS: f32 = 0.0;
const VISION_DEFAULT_VEL_ROT_MM_PS: f32 = 1.0;
const VISION_DEFAULT_VEL_LIN_MM_PS: f32 = 0.1; // Defaults to a lead of 1%
const VISION_MAX_VEL_LIN_MM_PS: f32 = 0.1; // Defaults to a lead of 1%

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

    Transition Line should be controlled to Zero line
    Else we are cutting too deep or shallow.
*/

pub fn get_control_output_from_vision(
    algo_out: VisionAlgorithmOutput,
    control_params: &ControlParams,
) -> ControlEffort {
    if algo_out.tearing_detected {
        log::error!("CONTROL: tearing detected! -> discarding frame");

        ControlEffort {
            motor_setpoints: MotorSetpoints {
                translation: MotorAction::new_velocity(
                    MotorDirection::Forward,
                    Velocity::new::<millimeter_per_second>(VISION_TEARING_VEL_LIN_MM_PS),
                ),
                rotation: MotorAction::new_velocity(
                    MotorDirection::Forward,
                    Velocity::new::<millimeter_per_second>(VISION_TEARING_VEL_ROT_MM_PS),
                ),
                knife: MotorAction::Hold,
            },
            led: LedSetpoint {
                brightness: LED_BRIGHTNESS,
            },
        }
    } else {
        // No tearing
        let knife_action = match algo_out.knife_setpoint {
            Some(VisionMotorSetpoint::Up(speed)) => {
                MotorAction::MoveVelocity(MotorVelocitySetpoint::new_forward(Velocity::new::<
                    millimeter_per_second,
                >(
                    speed as f32 / u8::MAX as f32 * VISION_MAX_SPEED_MM_PS,
                )))
            }
            Some(VisionMotorSetpoint::Down(speed)) => {
                MotorAction::MoveVelocity(MotorVelocitySetpoint::new_reverse(Velocity::new::<
                    millimeter_per_second,
                >(
                    -(speed as f32 / u8::MAX as f32 * VISION_MAX_SPEED_MM_PS),
                )))
            }
            Some(VisionMotorSetpoint::Hold) => MotorAction::Hold,
            None => MotorAction::Hold,
        };

        let lin_speed: f32 = (control_params.lead / LEAD_MAX * VISION_DEFAULT_VEL_LIN_MM_PS)
            .clamp(0.0, VISION_MAX_VEL_LIN_MM_PS);

        ControlEffort {
            motor_setpoints: MotorSetpoints {
                translation: MotorAction::new_velocity(
                    MotorDirection::Forward,
                    Velocity::new::<millimeter_per_second>(lin_speed),
                ),
                rotation: MotorAction::new_velocity(
                    MotorDirection::Forward,
                    Velocity::new::<millimeter_per_second>(VISION_DEFAULT_VEL_ROT_MM_PS),
                ),
                knife: knife_action,
            },
            led: LedSetpoint {
                brightness: LED_BRIGHTNESS,
            },
        }
    }
}

pub async fn calculate_control_effort(
    frame: Arc<FrameBufferView>,
    control_params: &ControlParams,
) -> VisionAlgorithmOutput {
    match ALGO {
        Algo::SimpleAverage => simple_average(frame, control_params),
        Algo::Complex => complex_algo(frame, control_params),
        Algo::PeriodicEncoderTest => periodic_encoder_test(frame, control_params),
        Algo::Joris => vision_joris(frame, control_params),
    }
}

// 3x3 Convolution with horizontal sobel kernel to determine midline point
pub fn complex_algo(
    frame: Arc<FrameBufferView>,
    control_params: &ControlParams,
) -> VisionAlgorithmOutput {
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

    let knife_setpoint = if out > HIGH_THRESHOLD {
        VisionMotorSetpoint::Up(100)
    } else if out < LOW_THRESHOLD {
        VisionMotorSetpoint::Down(100)
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
