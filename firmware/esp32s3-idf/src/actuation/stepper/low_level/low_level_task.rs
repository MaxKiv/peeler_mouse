use embassy_futures::select::{select3, Either3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::PinDriver,
    rmt::{TxRmtConfig, TxRmtDriver},
};
use log::*;
use messenger_mouse::motor::Steps;
use rmt_stepper_driver::RmtStepper;
use uom::si::{f32::Length, length::millimeter};

use crate::actuation::stepper::{
    low_level::{
        state_machine::{RampState, StepperState, StepperStateMachine},
        StepperAction, STEP_INTERVAL,
    },
    motor_task::{KNIFE_MOTOR_POS, KNIFE_MOTOR_POS_RESET, MINIMUM_SPS},
    peripherals::StepperPeripherals,
};

/// Internal: motor control task -> stepper task
pub static STEPPER_ACTION: Watch<CriticalSectionRawMutex, StepperAction, 1> = Watch::new();

/// Low level stepper task
/// Manages stepper state machine
/// Transforms velocity setpoints into pulse intervals
/// Facilitates RMT stepper driver do_n_steps by combining stepper pulses into intervals
/// Responsible for ramping stepper velocity up and down
/// Tracks steps made
#[embassy_executor::task]
pub async fn low_lvl_stepper_task(p: StepperPeripherals) {
    log::info!("MOTOR: initialising LOW level control task");

    // --- hardware init ---
    info!("MOTOR LOW LVL: Init hardware");
    // RMT Clock divider, RMT takes base AHB clock, which defaults to 80MHz
    // Maximum RMT pulse width = u16::Max, which together with clock divider above determines maximum frequency
    // For clock_divider of 80 -> minimum pulse width = 1us and min frequency of ~15Hz
    let clock_divider = 80;
    let rmt_cfg = TxRmtConfig::new().clock_divider(clock_divider);
    let mut rmt_driver = TxRmtDriver::new(p.rmt_channel, p.step_rmt_pin, &rmt_cfg).unwrap();
    let _ = rmt_driver.set_looping(esp_idf_hal::rmt::config::Loop::Count(1));
    let dir_pin = PinDriver::output(p.dir_pin).unwrap();
    let driver = RmtStepper::new("KNIFE", rmt_driver, dir_pin, Delay, clock_divider);

    // --- Communication channels ----
    info!("MOTOR LOW LVL: Init comms");
    let pos_tx = KNIFE_MOTOR_POS.sender();
    let mut pos_reset_rx = KNIFE_MOTOR_POS_RESET.receiver().unwrap();
    let mut cmd_rx = STEPPER_ACTION.receiver().unwrap();

    // --- Local State ----
    info!("MOTOR LOW LVL: Init state");

    let mut enable = PinDriver::output(p.enable_pin).unwrap();
    let _ = enable.set_high();

    let mut sm = StepperStateMachine::new(driver, enable);
    pos_tx.send(sm.current_position);

    info!("MOTOR LOW LVL: Entering main loop");

    // Main stepper control loop, functions in parallel:
    // 1. Check for new stepper command
    // 2. Service current stepper command by stepping at the correct period
    // 3. Listen for position reset requests
    loop {
        // Is current and target velocity less than allowed minimum requested?
        if sm.vel_state.target_speed < MINIMUM_SPS
            && sm.vel_state.current_speed < MINIMUM_SPS
            && sm.state == StepperState::Velocity
        {
            // Special case; transition into hold mode
            info!(
                "MOTOR LOW LVL: current & target speed < MINIMUM_SPS -> transition into hold mode"
            );
            sm.transition_to(StepperAction::Hold).await;
        }

        // Update step timer for velocity and position modes
        // This step timer is used to re-arm the RMT driver STEP pulses, which is only required if
        // we want to move the motor
        // Note: StepperState::SingleStep is handled in its state transition
        let step_timer = match &sm.state {
            StepperState::Velocity | StepperState::Position => {
                // info!("MOTOR LOW LVL: in StepperState::Velocity");

                // Check if we are required to switch direction
                if sm.vel_state.ramp_state == RampState::Decelerating {
                    // Check if we are ready to switch direction
                    if sm.ready_for_direction_switch() {
                        info!(
                        "MOTOR LOW LVL: below direction switch transition speed and decelerating -> switching direction and starting acceleration {:?}", sm.vel_state,
                    );

                        // We are -> Do so
                        sm.switch_direction().await;
                    }
                }

                // Update current velocity using ramping state and acceleration
                // This also updates stepper driver RMT stuff, like steps per interval
                sm.update_velocity();

                // Set up RMT re-arm timer to perform steps
                Timer::after(STEP_INTERVAL + Duration::from_millis(10))
            }
            _ => {
                // We are not in velocity mode, we don't want to do any steps
                // Large duration timer avoids type and future boxing headache
                Timer::after(Duration::from_secs(6000))
            }
        };

        match select3(cmd_rx.changed(), step_timer, pos_reset_rx.changed()).await {
            // New step command requested -> Transition Stepper State Machine
            Either3::First(new_action) => {
                debug!("MOTOR LOW LVL: NEW StepperAction: {:?}", new_action);

                // Perform state transition to requested state
                // This
                // - updates velocity & direction target and ramping state
                // - sets enable pin to required state
                sm.transition_to(new_action).await;
            }

            // Velocity mode step timer expired ->
            // Re-arm RMT
            // Attempt to track steps
            Either3::Second(()) => {
                // Re-arm RMT TX STEP pulse driver when movement is required
                match sm.state {
                    StepperState::Velocity => {
                        // We are in velocity mode
                        if sm.interval_cfg.is_some() {
                            sm.on_step_timer_expire().await;
                            pos_tx.send(sm.current_position);
                        }
                    }
                    StepperState::Position => {
                        // We are in Position mode
                        if sm.target_is_reached() {
                            info!(
                                "TARGET REACHED XXX {:?} -> {:?}",
                                sm.current_position, sm.target_position
                            );

                            // Target reached, transition to hold
                            sm.transition_to(StepperAction::Hold).await;
                        } else {
                            debug!(
                                "YYY Target NOT reached {:?} -> {:?} @ {:?} & {:?}",
                                sm.current_position,
                                sm.target_position,
                                sm.vel_state,
                                sm.interval_cfg
                            );

                            if sm.interval_cfg.is_some() {
                                sm.on_step_timer_expire().await;
                                pos_tx.send(sm.current_position);
                            }
                        }
                    }

                    _ => {
                        //
                    }
                }
            }

            // Position reset requested
            Either3::Third(()) => {
                info!("MOTOR LOW LVL: Position reset requested");

                // Reset position
                sm.current_position = Steps(0);
                // Inform upstream
                pos_tx.send(sm.current_position);
            }
        }
    }
}

/// Converts a position target (distance from home) into step pulses for the stepper driver
pub fn position_to_steps(target: Length) -> Steps {
    use messenger_mouse::encoder::*;
    let mm = target.get::<millimeter>();

    let out = (mm / KNIFE_AXIS_LEAD_MM)
        * KNIFE_AXIS_GEAR_RATIO
        * KNIFE_AXIS_MICROSTEPS_PER_STEP
        * KNIFE_AXIS_STEPS_PER_ROTATION;

    debug!("target_mm_to_steps: target {}mm -> {}steps", mm, out);

    Steps(out as i32)
}
