use embassy_futures::select::select;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Receiver, Watch},
};
use embassy_time::{Duration, Timer};

use crate::{
    actuation::stepper::{
        low_level::state_machine::{StepperState, LOW_LEVEL_STEPPER_STATE},
        LimitSwitchState, LIMIT_EVENT,
    },
    encoder::encoder_task::ENCODER_STATE,
};

static STALL_EVENT: Watch<CriticalSectionRawMutex, StallEvent, 1> = Watch::new();
const ENCODER_STALL_DEBOUNCE_DURATION: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, PartialEq)]
pub enum StallEvent {
    Stalled,
    Resolved,
}

/// Watches encoder, yields StallEvents.
/// Cancellation is handled by the caller via select().
#[embassy_executor::task]
pub async fn monitor_encoder_stall() {
    log::info!("MOTOR: initialising monitor encoder stalls task");

    let mut rx_encoder = ENCODER_STATE.receiver().unwrap();
    let tx = STALL_EVENT.sender();
    let mut stalled = false;

    loop {
        let before = rx_encoder.try_get();
        Timer::after(ENCODER_STALL_DEBOUNCE_DURATION).await;
        let after = rx_encoder.try_get();

        let is_stalled = before == after;

        if is_stalled && !stalled {
            stalled = true;
            tx.send(StallEvent::Stalled);
        } else if !is_stalled && stalled {
            stalled = false;
            tx.send(StallEvent::Resolved);
        }
    }
}

// Watches encoder for motor stalls
// Informs others about them
#[embassy_executor::task]
pub async fn encoder_limit_switch() {
    log::info!("MOTOR: Init encoder limit switch task");

    let tx = LIMIT_EVENT.sender();
    let mut rx_ll_stepper_state = LOW_LEVEL_STEPPER_STATE.receiver().unwrap();
    let mut rx_stall_event = STALL_EVENT.receiver().unwrap();

    loop {
        wait_for_stepper_state(&mut rx_ll_stepper_state, is_moving).await;

        let stalled = select(
            wait_for_stall_event(&mut rx_stall_event, StallEvent::Stalled),
            wait_for_stepper_state(&mut rx_ll_stepper_state, is_not_moving),
        )
        .await;

        if stalled.is_first() {
            tx.send(LimitSwitchState::Active);

            select(
                wait_for_stall_event(&mut rx_stall_event, StallEvent::Resolved),
                wait_for_stepper_state(&mut rx_ll_stepper_state, is_not_moving),
            )
            .await;

            // Send Inactive regardless of which branch won
            // motor stopped normally or stall resolved; upstream needs a clean reset
            tx.send(LimitSwitchState::Inactive);
        }
    }
}

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
