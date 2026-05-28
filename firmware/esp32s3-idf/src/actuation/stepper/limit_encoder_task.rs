use embassy_futures::select::select;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Sender, Watch},
};
use embassy_time::{Duration, Timer};
use messenger_mouse::encoder::EncoderState;

use crate::{
    actuation::stepper::low_level::state_machine::StepperState,
    encoder::encoder_task::ENCODER_STATE,
};

pub static STALL_EVENT: Watch<CriticalSectionRawMutex, StallEvent, 1> = Watch::new();
pub static START_STALL_MONITOR: Watch<CriticalSectionRawMutex, StallMonitorCmd, 1> = Watch::new();
pub const ENCODER_STALL_DEBOUNCE_DURATION: Duration = Duration::from_millis(300);

#[derive(Debug, Clone, PartialEq)]
pub enum StallMonitorCmd {
    Start,
    Stop,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub enum StallEvent {
    Stalled,
    #[default]
    Resolved,
}

/// Starts disabled, enable via StallMonitorCmd::Start
/// If enabled: Watches encoder, yields StallEvents.
/// Can be canceled any time using StallMonitorCmd::Stop
#[embassy_executor::task]
pub async fn monitor_encoder_stall() {
    log::info!("ENCODER STALL: initialising monitor encoder stalls task");

    // Comms
    let mut rx_cmd = START_STALL_MONITOR.receiver().unwrap();
    let mut rx_encoder = ENCODER_STATE.receiver().unwrap();
    let tx = STALL_EVENT.sender();

    // State
    let mut previously_stalled = false;

    loop {
        // Upstream indicates we need to start watching for stalls
        wait_for_stall_command(&mut rx_cmd, StallMonitorCmd::Start).await;
        log::info!("ENCODER STALL: Start watching for stalls");

        // Race stall detection and cancelation command
        select(
            detect_stalls(&mut rx_encoder, &tx, &mut previously_stalled),
            wait_for_stall_command(&mut rx_cmd, StallMonitorCmd::Stop),
        )
        .await;
    }
}

/// Watch for encoder stalls, inform upstream through tx watch
/// Infinite future, never completes
async fn detect_stalls(
    rx_encoder: &mut Receiver<'_, CriticalSectionRawMutex, EncoderState, 2>,
    tx: &Sender<'_, CriticalSectionRawMutex, StallEvent, 1>,
    previously_stalled: &mut bool,
) {
    loop {
        // Race stall detection and upstream cancel command
        let before = match rx_encoder.try_get() {
            Some(before) => before,
            None => {
                log::error!("ENCODER STALL: unable to get encoder state");
                Timer::after(ENCODER_STALL_DEBOUNCE_DURATION).await;
                continue;
            }
        };
        Timer::after(ENCODER_STALL_DEBOUNCE_DURATION).await;

        let after = match rx_encoder.try_get() {
            Some(after) => after,
            None => {
                log::error!("ENCODER STALL: unable to get encoder state");
                Timer::after(ENCODER_STALL_DEBOUNCE_DURATION).await;
                continue;
            }
        };

        let is_stalled = before == after;

        // if is_stalled && !(*previously_stalled) {
        if is_stalled {
            *previously_stalled = true;
            tx.send(StallEvent::Stalled);
            log::warn!(
                "ENCODER STALL: ~~~~ STALLED ~~~~ {}:{} -> {}:{}",
                before.encoder_data.angle,
                before.encoder_data.revolution,
                after.encoder_data.angle,
                after.encoder_data.revolution,
            );
        } else if !is_stalled && *previously_stalled {
            *previously_stalled = false;
            tx.send(StallEvent::Resolved);
            log::warn!(
                "ENCODER STALL: ~~~~ RESOLVED ~~~~ {}:{} -> {}:{}",
                before.encoder_data.angle,
                before.encoder_data.revolution,
                after.encoder_data.angle,
                after.encoder_data.revolution,
            );
        }
    }
}

// Watches encoder for motor stalls
// Informs others about them
// #[embassy_executor::task]
// pub async fn encoder_limit_switch() {
//     log::info!("ENCODER STALL: starting encoder_limit_switch task");
//
//     let tx = LIMIT_EVENT.sender();
//     let mut rx_ll_stepper_state = LOW_LEVEL_STEPPER_STATE.receiver().unwrap();
//     let mut rx_stall_event = STALL_EVENT.receiver().unwrap();
//
//     loop {
//         wait_for_stepper_state(&mut rx_ll_stepper_state, is_moving).await;
//
//         log::info!("ENCODER STALL: ");
//
//         let stalled = select(
//             wait_for_stall_event(&mut rx_stall_event, StallEvent::Stalled),
//             wait_for_stepper_state(&mut rx_ll_stepper_state, is_not_moving),
//         )
//         .await;
//
//         if stalled.is_first() {
//             tx.send(LimitSwitchState::Active);
//
//             select(
//                 wait_for_stall_event(&mut rx_stall_event, StallEvent::Resolved),
//                 wait_for_stepper_state(&mut rx_ll_stepper_state, is_not_moving),
//             )
//             .await;
//
//             // Send Inactive regardless of which branch won
//             // motor stopped normally or stall resolved; upstream needs a clean reset
//             tx.send(LimitSwitchState::Inactive);
//         }
//     }
// }

async fn wait_for_stepper_state(
    rx: &mut Receiver<'_, CriticalSectionRawMutex, StepperState, 1>,
    predicate: impl Fn(StepperState) -> bool,
) {
    if rx.try_get().is_some_and(&predicate) {
        return;
    }
    loop {
        if predicate(rx.changed().await) {
            return;
        }
    }
}

fn is_moving(state: StepperState) -> bool {
    matches!(state, StepperState::Velocity | StepperState::Position)
}
fn is_not_moving(state: StepperState) -> bool {
    !is_moving(state)
}

async fn wait_for_stall_event(
    rx: &mut Receiver<'_, CriticalSectionRawMutex, StallEvent, 1>,
    target: StallEvent,
) {
    loop {
        if rx.changed().await == target {
            return;
        }
    }
}

async fn wait_for_stall_command(
    rx: &mut Receiver<'_, CriticalSectionRawMutex, StallMonitorCmd, 1>,
    command: StallMonitorCmd,
) {
    if rx.try_get().is_some_and(|cmd| cmd == command) {
        return;
    }
    loop {
        let cmd = rx.changed().await;
        if cmd == command {
            return;
        }
    }
}
