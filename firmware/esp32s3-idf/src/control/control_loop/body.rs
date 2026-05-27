use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Sender;
use esp_idf_sys::EspError;
use std::{sync::Arc, time::Instant};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use embassy_time::Timer;
use embassy_time::{Duration, Ticker, WithTimeout};
use esp_idf_hal::ledc::LedcDriver;
use log::*;
use messenger_mouse::{
    encoder::EncoderState,
    motor::{KnifeManager, MotorAction},
    AppState, Esp32Setpoint, VisionAlgorithmOutput, VisionData,
};
use messenger_mouse::{ControlEffort, LED_BRIGHTNESS};

use crate::{
    actuation::stepper::{
        motor_task::{KNIFE_MOTOR_HOME_STATUS, KNIFE_MOTOR_SETPOINT},
        HomeStatus,
    },
    camera::{
        camera_freertos_task::FRAMEBUFFER_CONTROL_LOOP_CHANNEL,
        framebuffer_view::{FrameBufferView, FRAME_DONE_SIGNAL},
    },
    comms::comms_task::REPORT_WATCH,
    control::vision::algo::{calculate_control_effort, get_control_output_from_vision},
    encoder::encoder_task::ENCODER_STATE,
};

const CONTROL_LOOP_FREQUENCY: Duration = Duration::from_hz(5);
const HOME_TIMEOUT: Duration = Duration::from_secs(30);

#[embassy_executor::task]
pub async fn control_loop(
    mut setpoint_receiver: Receiver<'static, Cs, Esp32Setpoint, 2>,
    mut led: LedcDriver<'static>,
) {
    info!("CONTROL: Entering Startup");

    // Boot indicator
    let _ = led.set_duty(led.get_max_duty());
    let _ = led.enable();
    Timer::after(Duration::from_millis(250)).await;
    let _ = led.disable();
    Timer::after(Duration::from_millis(250)).await;
    let _ = led.enable();
    Timer::after(Duration::from_millis(250)).await;
    let _ = led.disable();
    Timer::after(Duration::from_millis(250)).await;

    info!("CONTROL: Initialising");

    // Track latest setpoint
    let mut latest_setpoint = messenger_mouse::Esp32Setpoint::default();

    // Track latest frame generation
    let mut last_gen = 0u32;

    // Task timekeeper
    let mut ticker = Ticker::every(CONTROL_LOOP_FREQUENCY);

    // Latest framebuffer signal
    let framebuffer_rx = FRAMEBUFFER_CONTROL_LOOP_CHANNEL.receiver();

    let report_tx = REPORT_WATCH.sender();

    let mut encoder_rx = ENCODER_STATE
        .receiver()
        .expect("not enough ENCODER_STATE rx N");

    let motor_tx = KNIFE_MOTOR_SETPOINT.sender();
    let mut motor_home_rx = KNIFE_MOTOR_HOME_STATUS
        .receiver()
        .expect("not enough KNIFE_MOTOR_HOME N");

    // Startup: Ask motor controller to start homing

    #[cfg(feature = "home_on_startup")]
    {
        info!("CONTROL: Startup -> Homing motor");
        loop {
            motor_tx.send(MotorAction::Home);
            let home_status = motor_home_rx.changed().with_timeout(HOME_TIMEOUT).await;
            error!(
                "CONTROL: Startup -> Received home status: {:?}",
                home_status
            );

            if let Ok(HomeStatus::Homed { position: _ }) = home_status {
                error!("CONTROL: Motor indicates succesful homing, running main loop");
                break;
            }
        }
    }

    // Main Control loop
    loop {
        // ----- Fetch Control Input -----
        if let Some(new_setpoint) = setpoint_receiver.try_get() {
            if new_setpoint != latest_setpoint {
                debug!("CONTROL: NEW setpoint: {:?}", new_setpoint);
                latest_setpoint = new_setpoint;
            }
        }

        // Get latest encoder value
        let knife_encoder_state = encoder_rx.try_get().unwrap_or_else(|| {
            warn!("CONTROL: unable to get encoder state, position mode unreliable!");
            EncoderState::new()
        });
        info!("CONTROL: Encoder state: {:?}", knife_encoder_state);

        // update appstate
        let mut current_appstate = match latest_setpoint.knife_manager {
            KnifeManager::Vision => AppState::Active,
            KnifeManager::Manual => AppState::StandBy,
        };

        // Run vision algorithm if enabled
        let (control_effort, vision_data) =
            if let KnifeManager::Vision = latest_setpoint.knife_manager {
                // Run vision routine
                // - Get latest framebuffer
                // - Attempt to detect tearing
                // - Calculating vision algorithm output
                // - Transform this output into a MotorAction

                // Get latest framebuffer from camera
                let frame = framebuffer_rx.receive().await;
                warn!("CONTROL: frame: {}", frame.generation);

                let timestamp = frame.timestamp.clone();
                let gen = frame.generation;
                let camera_fps = frame.fps;

                // info!("CONTROL: got framebuffer gen {}", gen);

                // Tearing detection
                let current_gen = frame.generation;
                let current_hash = frame.calculate_checksum();
                let last_hash = frame.hash;
                detect_tearing(current_gen, last_gen, current_hash, last_hash);

                // Calculate control effort through vision algorithm
                let vision_output = {
                    let result = get_control_effort(frame).await;
                    result
                };
                // Processing is done; Tell the camera task
                warn!("CONTORL: vision done -> signalling camera_freertos_task");
                FRAME_DONE_SIGNAL.signal(());

                // Convert into motor command
                let control_effort = get_control_output_from_vision(vision_output.clone());

                info!(
                    "CONTROL: VISION frame {} -> vision alg: {:?} -> control effort: {:?}",
                    gen, vision_output, control_effort,
                );

                // Export in nice type
                let vision_data = VisionData {
                    generation: gen,
                    timestamp_s: timestamp.tv_sec,
                    timestamp_us: timestamp.tv_usec,
                    camera_fps,
                    vision_output,
                };

                // Bookkeeping
                last_gen = vision_data.generation;

                (Some(control_effort), Some(vision_data))
            } else {
                (None, None)
            };

        // Actuate LED
        let brightness = if let Some(ControlEffort { led, .. }) = &control_effort {
            led.brightness
        } else {
            LED_BRIGHTNESS
        };
        if let Err(err) = actuate_led(&mut led, brightness) {
            log::error!("CONTROL: unable to set LED duty cycle: {err}");
            current_appstate = AppState::Fault;
        }

        // Actuate knife motor
        let knife_motor_action = if let Some(ControlEffort { knife, .. }) = &control_effort {
            knife.clone()
        } else {
            latest_setpoint.knife_setpoint.clone()
        };
        actuate_knife_motor(knife_motor_action, &motor_tx);

        // Collect measurement for report
        let measurements = messenger_mouse::Measurements {
            vision_data,
            knife_encoder_state,
        };

        // Combine with vision data and control_effort into report
        let report = messenger_mouse::Report {
            setpoint: latest_setpoint.clone(),
            app_state: current_appstate,
            measurements,
            control_effort,
        };

        // Report to STM32
        warn!("CONTROL: sending  report {:?}", report);
        report_tx.send(report);

        // Throttle control loop
        ticker.next().await;
    }
}

fn actuate_knife_motor(
    knife_motor_action: MotorAction,
    motor_tx: &Sender<CriticalSectionRawMutex, MotorAction, 2>,
) {
    // Actuate Knife adjustment motor
    info!("CONTROL: MOTOR CMD {:?}", knife_motor_action);
    motor_tx.send(knife_motor_action);
}

fn actuate_led(led: &mut LedcDriver<'static>, brightness: f32) -> Result<(), EspError> {
    // Note: Esp driver clamps DC value
    led.set_duty((brightness * (led.get_max_duty() as f32)) as u32)
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
async fn get_control_effort(frame: Arc<FrameBufferView>) -> VisionAlgorithmOutput {
    let start = Instant::now();

    let out = calculate_control_effort(frame).await;

    let dur = Instant::now().duration_since(start);
    log::warn!(
        "CONTROL: took {}ms to find new output: {:?}",
        dur.as_millis(),
        out,
    );

    out
}
