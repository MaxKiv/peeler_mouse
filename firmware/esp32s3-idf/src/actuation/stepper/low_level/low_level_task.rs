use embassy_futures::select::{select3, Either3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::PinDriver,
    rmt::{TxRmtConfig, TxRmtDriver},
};
use log::*;
use messenger_mouse::motor::MotorDirection;
use rmt_stepper_driver::RmtStepper;

use crate::actuation::stepper::{
    low_level::{
        state_machine::{RampState, StepperState, StepperStateMachine},
        STEP_INTERVAL,
    },
    motor_task::{
        StepperCommand, KNIFE_MOTOR_POS, KNIFE_MOTOR_POS_RESET, MAXIMUM_SPS, MINIMUM_SPS,
        MINIMUM_TRANSITION_SPS,
    },
    peripherals::StepperPeripherals,
    Steps,
};

/// Internal: motor control task -> stepper task
pub static STEPPER_CMD: Watch<CriticalSectionRawMutex, StepperCommand, 1> = Watch::new();
const STEPS_PER_INTERVAL: u32 = 10; // Steps per interval (interval n = n RMT pulses before re-arm)

/// Low level stepper task
/// Manages stepper state machine
/// Transforms velocity setpoints into pulse intervals
/// Facilitates RMT stepper driver do_n_steps by combining stepper pulses into intervals
/// Responsible for ramping stepper velocity up and down
/// Tracks steps made
#[embassy_executor::task]
pub async fn low_lvl_stepper_task(p: StepperPeripherals) {
    // --- hardware init ---
    // RMT Clock divider, RMT takes base AHB clock, which defaults to 80MHz
    // Maximum RMT pulse width = u16::Max, which together with clock divider above determines maximum frequency
    // For clock_divider of 80 -> minimum pulse width = 1us and min frequency of ~15Hz
    let clock_divider = 80;
    let rmt_cfg = TxRmtConfig::new().clock_divider(clock_divider);
    let rmt_driver = TxRmtDriver::new(p.rmt_channel, p.step_rmt_pin, &rmt_cfg).unwrap();
    let dir_pin = PinDriver::output(p.dir_pin).unwrap();
    let mut driver = RmtStepper::new("KNIFE", rmt_driver, dir_pin, Delay, clock_divider);

    // --- Communication channels ----
    let pos_tx = KNIFE_MOTOR_POS.sender();
    let mut pos_reset_rx = KNIFE_MOTOR_POS_RESET.receiver().unwrap();
    let mut cmd_rx = STEPPER_CMD.receiver().unwrap();

    // --- Local State ----

    // let accel_per_step = ACCEL_PER_INTERVAL / STEPS_PER_INTERVAL;
    let mut current_step_period = MINIMUM_SPS;
    let mut position = Steps(0);
    pos_tx.send(position);

    let mut enable = PinDriver::output(p.enable_pin).unwrap();
    let _ = enable.set_high();

    let mut sm = StepperStateMachine::new(driver, enable);

    // Main stepper control loop, functions in parallel:
    // 1. Check for new stepper command
    // 2. Service current stepper command by stepping at the correct period
    // 3. Listen for position reset requests
    loop {
        // Velocity mode specific; Calculate steps to do this interval
        let interval_cfg = if let StepperState::Velocity = sm.state {
            // Check if we are required to switch direction
            if sm.vel_state.ramp_state == RampState::Decelerating {
                // Check if we are ready to switch direction
                if sm.ready_for_direction_switch() {
                    // We are -> Do so
                    sm.switch_direction().await;
                }
            }

            // Update current velocity using ramping state and acceleration
            let sps = sm.update_velocity();

            // Calculate how many steps to take & ticks per step for this interval
            // Update the RMT driver step pulse period duration
            let interval_cfg = sm.set_interval_config();

            Some(interval_cfg)
        } else {
            // We are not in velocity mode, but the ergonomics of select3 force us to return
            // something here
            None
        };

        // Velocity mode specific; Set up timer to re-arm RMT
        let step_timer = if let StepperState::Velocity = sm.state {
            // In velocity mode; Set up RMT driver to perform steps
            Timer::after(STEP_INTERVAL)
        } else {
            // We are not in velocity mode, we don't want to do any steps
            // Infinite duration timer avoids type and future boxing headache
            Timer::after(Duration::MAX)
        };

        match select3(cmd_rx.changed(), step_timer, pos_reset_rx.changed()).await {
            // New step command requested -> Transition Stepper State Machine
            Either3::First(new_cmd) => {
                // Perform state transition to requested state
                // This
                // - updates velocity & direction target and ramping state
                // - sets enable pin to required state
                sm.transition_to(new_cmd);
            }

            // Velocity mode step timer expired ->
            // Re-arm RMT
            // Attempt to track steps
            Either3::Second(()) => {
                // A RMT STEP pulse timer expired, we should be in velocity mode
                if let Some(interval_cfg) = interval_cfg {
                    sm.on_step_timer_expire();
                    pos_tx.send(position);
                }
            }

            // Position reset requested
            Either3::Third(()) => {
                // Reset position
                position = Steps(0);
                // Inform upstream
                pos_tx.send(position);
            }
        }
    }
}
