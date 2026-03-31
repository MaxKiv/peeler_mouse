use std::time::SystemTime;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use embassy_time::{Duration, Ticker, Timer, WithTimeout};
use esp_idf_hal::ledc::LedcDriver;
use log::*;
use messenger_mouse::{
    encoder::KnifeState,
    motor::{KnifeManager, MotorCommand},
    AppState, Setpoint, VisionAlgorithmOutput,
};

use crate::{
    actuation::stepper::{
        motor_task::{KNIFE_MOTOR_HOME, KNIFE_MOTOR_SETPOINT},
        HomeStatus,
    },
    camera::{camera_freertos_task::FRAMEBUFFER_CONTROL_LOOP_CHANNEL, framebuffer::FrameBuffer},
    comms::comms_task::REPORT_WATCH,
    control::vision::algo::{calculate_control_effort, vision_output_to_motorcommand},
    encoder::encoder_task::KNIFE_STATE,
};

const CONTROL_LOOP_FREQUENCY: Duration = Duration::from_millis(1000);

#[embassy_executor::task]
pub async fn control_loop(
    mut setpoint_receiver: Receiver<'static, Cs, Setpoint, 2>,
    mut led: LedcDriver<'static>,
) {
    info!("starting control task");

    // Track latest setpoint
    let mut latest_setpoint = messenger_mouse::Setpoint::default();

    // Task timekeeper
    let mut ticker = Ticker::every(CONTROL_LOOP_FREQUENCY);

    // Latest framebuffer signal
    let mut framebuffer_rx = FRAMEBUFFER_CONTROL_LOOP_CHANNEL
        .receiver()
        .expect("not enough FRAMEBUFFER_CONTROL_LOOP_CHANNEL rx N");

    let mut encoder_rx = KNIFE_STATE.receiver().expect("not enough KNIFE_STATE rx N");

    let motor_tx = KNIFE_MOTOR_SETPOINT.sender();
    let mut motor_rx = KNIFE_MOTOR_HOME
        .receiver()
        .expect("not enough KNIFE_MOTOR_HOME N");
    let report_tx = REPORT_WATCH.sender();

    // Startup: Home motor
    loop {
        motor_tx.send(MotorCommand::Home);
        let home_status = motor_rx.changed().with_timeout(Duration::from_hz(1)).await;
        match home_status {
            Ok(HomeStatus::Homed) => {
                info!("CONTROL: Motor indicates succesful homing, running main loop");
                break;
            }
            Ok(HomeStatus::Lost) => {
                warn!("CONTROL: Knife motor not homed yet");
                continue;
            }
            _ => continue,
        }
    }

    // Main Control loop
    loop {
        // Update to latest setpoint, if any
        if let Some(new_setpoint) = setpoint_receiver.try_get() {
            latest_setpoint = new_setpoint;
        }

        // update appstate
        let mut current_appstate = match latest_setpoint.knife_manager {
            KnifeManager::Vision => AppState::Active,
            KnifeManager::Manual => AppState::StandBy,
        };

        // Act on latest setpoint
        // if latest_setpoint.enable {
        // Get latest framebuffer from camera
        let frame = framebuffer_rx.changed().await;

        let gen = frame.generation;
        let timestamp_us = frame.timestamp_us;
        let camera_fps = frame.fps;
        let led_brightness = latest_setpoint.led_setpoint.brightness;

        // Get latest encoder value
        let current_knife_state = encoder_rx.try_get().unwrap_or_else(|| {
            log::error!("CONTROL: unable to get valid knife state, using default...");
            KnifeState::new()
        });

        // Calculate control effort
        let vision_output = get_control_effort(frame).await;

        // Knife motor actuation
        let motor_cmd = match latest_setpoint.knife_manager {
            KnifeManager::Manual => {
                // Translate knife motor commannd to
                latest_setpoint.knife_setpoint.clone()
            }
            KnifeManager::Vision => {
                // Convert into motor command
                let motor_cmd = vision_output_to_motorcommand(vision_output.clone());

                log::info!(
                    "CONTROL: frame {} -> vision alg: {:?} -> control effort: {:?}",
                    gen,
                    vision_output,
                    motor_cmd,
                );

                motor_cmd
            }
        };

        // Actuate Knife adjustment motor
        motor_tx.send(motor_cmd);

        // Actuate LED
        // Note: Esp driver clamps DC value
        if let Err(err) = led.set_duty((led_brightness * (led.get_max_duty() as f32)) as u32) {
            log::error!("CONTROL: unable to set LED duty cycle: {err}");
            current_appstate = AppState::Fault;
        }

        // Collect & Send Report to stm32
        let measurements = messenger_mouse::Measurements {
            timestamp_us,
            camera_fps,
            controller_output: vision_output,
            current_knife_state,
        };

        let report = messenger_mouse::Report {
            setpoint: latest_setpoint.clone(),
            app_state: current_appstate,
            measurements,
        };

        info!("CONTROL: sending  report {:?}", report);
        report_tx.send(report);

        ticker.next().await;
    }
}

// Calculate control effort + telemetry
async fn get_control_effort(frame: FrameBuffer) -> VisionAlgorithmOutput {
    let start = SystemTime::now();

    let out = calculate_control_effort(frame).await;

    let dur = SystemTime::now().duration_since(start).unwrap_or_default();
    log::info!(
        "CONTROL: took {}ms to find new output: {:?}",
        dur.as_millis(),
        out,
    );

    out
}
