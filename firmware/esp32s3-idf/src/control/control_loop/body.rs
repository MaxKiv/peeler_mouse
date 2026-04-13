use std::time::Instant;

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use embassy_time::{Duration, Ticker, Timer, WithTimeout};
use esp_idf_hal::ledc::LedcDriver;
use log::*;
use messenger_mouse::{
    encoder::KnifeState,
    motor::{KnifeManager, MotorCommand},
    AppState, Setpoint, VisionAlgorithmOutput, VisionData,
};

use crate::{
    actuation::stepper::{
        motor_task::{KNIFE_MOTOR_HOME_STATUS, KNIFE_MOTOR_SETPOINT},
        HomeStatus, MotorAction,
    },
    camera::{
        camera_freertos_task::FRAMEBUFFER_CONTROL_LOOP_CHANNEL, framebuffer::FrameBuffer,
        framebuffer_view::FrameBufferView,
    },
    comms::comms_task::REPORT_WATCH,
    control::vision::algo::{calculate_control_effort, vision_output_to_motorcommand},
    encoder::encoder_task::KNIFE_STATE,
};

const CONTROL_LOOP_FREQUENCY: Duration = Duration::from_millis(1000);
const HOME_TIMEOUT: Duration = Duration::from_secs(30);

#[embassy_executor::task]
pub async fn control_loop(
    mut setpoint_receiver: Receiver<'static, Cs, Setpoint, 2>,
    mut led: LedcDriver<'static>,
) {
    info!("CONTROL: Entering Startup");

    // Track latest setpoint
    let mut latest_setpoint = messenger_mouse::Setpoint::default();

    // Track latest frame generation
    let mut last_gen = 0u32;

    // Task timekeeper
    let mut ticker = Ticker::every(CONTROL_LOOP_FREQUENCY);

    // Latest framebuffer signal
    let mut framebuffer_rx = FRAMEBUFFER_CONTROL_LOOP_CHANNEL.receiver();

    let mut encoder_rx = KNIFE_STATE.receiver().expect("not enough KNIFE_STATE rx N");

    let motor_tx = KNIFE_MOTOR_SETPOINT.sender();
    let mut motor_home_rx = KNIFE_MOTOR_HOME_STATUS
        .receiver()
        .expect("not enough KNIFE_MOTOR_HOME N");
    let report_tx = REPORT_WATCH.sender();

    // Startup: Ask motor controller to start homing
    info!("CONTROL: Startup -> Homing motor");
    motor_tx.send(MotorCommand::Home);
    loop {
        let home_status = motor_home_rx.changed().with_timeout(HOME_TIMEOUT).await;
        info!(
            "CONTROL: Startup -> Received home status: {:?}",
            home_status
        );

        if let Ok(HomeStatus::Homed { position: _ }) = home_status {
            info!("CONTROL: Motor indicates succesful homing, running main loop");
            break;
        }
    }

    // Main Control loop
    loop {
        // ----- Fetch Control Input -----
        if let Some(new_setpoint) = setpoint_receiver.try_get() {
            if new_setpoint != latest_setpoint {
                // info!("CONTROL: NEW setpoint: {:?}", new_setpoint);
                latest_setpoint = new_setpoint;
            }
        } else {
            // info!("CONTROL: CURRENT setpoint: {:?}", latest_setpoint);
        }
        let led_brightness = latest_setpoint.led_setpoint.brightness;

        // Get latest encoder value
        let current_knife_state = encoder_rx.try_get().unwrap_or_else(|| {
            log::error!("CONTROL: unable to get valid knife state, using default...");
            KnifeState::new()
        });
        // info!("CONTROL: Encoder state: {:?}", current_knife_state);

        // update appstate
        let mut current_appstate = match latest_setpoint.knife_manager {
            KnifeManager::Vision => AppState::Active,
            KnifeManager::Manual => AppState::StandBy,
        };

        let (motor_cmd, vision_data) = match latest_setpoint.knife_manager.clone() {
            KnifeManager::Manual => {
                // Translate knife motor command to
                // info!("CONTROL: MANUAL setpoint: {:?}", latest_setpoint);
                let motor_cmd = latest_setpoint.knife_setpoint.clone();

                (motor_cmd, None)
            }

            KnifeManager::Vision => {
                // Get latest framebuffer from camera
                let frame = framebuffer_rx.receive().await;

                let timestamp = frame.timestamp.clone();
                let gen = frame.generation;
                let camera_fps = frame.fps;

                // info!("CONTROL: got framebuffer gen {}", gen);

                // Tearing detection
                let current_gen = frame.generation;
                let current_hash = frame.calculate_checksum();
                let last_hash = frame.hash;
                detect_tearing(current_gen, last_gen, current_hash, last_hash);

                // ----- Calculate control effort -----
                // Calculate control effort through vision algorithm
                let vision_output = get_control_effort(frame).await;

                // Convert into motor command
                let motor_cmd = vision_output_to_motorcommand(vision_output.clone());

                // info!(
                //     "CONTROL: VISION frame {} -> vision alg: {:?} -> control effort: {:?}",
                //     gen, vision_output, motor_cmd,
                // );

                let vision_data = VisionData {
                    generation: gen,
                    timestamp_s: timestamp.tv_sec,
                    timestamp_us: timestamp.tv_usec,
                    camera_fps,
                    vision_output,
                };

                // Bookkeeping
                last_gen = vision_data.generation;

                (motor_cmd, Some(vision_data))
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
            vision_data,
            current_knife_state,
        };

        // Combine into report
        let report = messenger_mouse::Report {
            setpoint: latest_setpoint.clone(),
            app_state: current_appstate,
            measurements,
        };

        // ----- Reporting -----
        info!("CONTROL: sending  report {:?}", report);
        report_tx.send(report);

        ticker.next().await;
    }
}

fn detect_tearing(current_gen: u32, last_gen: u32, current_hash: u32, last_hash: u32) -> bool {
    let hash_mismatch = current_hash != last_hash;
    let gen_mismatch = (current_gen <= last_gen) && last_gen != 0;

    if hash_mismatch {
        error!(
            "CONTROL: Aliasing detected! old & new checksum mismatch: {} != {}, continuing...",
            current_hash, last_hash
        );
        return true;
    }

    if gen_mismatch {
        error!(
            "CONTROL: Aliasing detected! last gen >= current gen {} >= {}, continuing...",
            last_gen, current_gen
        );
        return true;
    }

    info!(
        "CONTROL: NO tearing detected for gen {} - hash {}",
        current_gen, current_hash
    );
    false
}

// Calculate control effort + telemetry
async fn get_control_effort(frame: FrameBufferView) -> VisionAlgorithmOutput {
    let start = Instant::now();

    let out = calculate_control_effort(frame).await;

    let dur = Instant::now().duration_since(start);
    log::info!(
        "CONTROL: took {}ms to find new output: {:?}",
        dur.as_millis(),
        out,
    );

    out
}
