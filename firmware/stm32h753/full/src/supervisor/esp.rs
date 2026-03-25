use defmt::*;
use messenger_mouse::{LedSetpoint, Setpoint};

use crate::{comms::task::SETPOINT_WATCH, supervisor::task::APPSTATE_WATCH};

/// Main supervisor loop, manages appstate
#[embassy_executor::task]
pub async fn supervise_esp() {
    info!("Starting to supervise ESP");

    let mut appstate_rx = APPSTATE_WATCH.receiver().unwrap();
    let setpoint_tx = SETPOINT_WATCH.sender();

    // Send known default setpoint on boot
    setpoint_tx.send(Setpoint::default());

    loop {
        // get latest appstate
        let state = appstate_rx.changed().await;
        info!(
            "Got knife management state: {:?}",
            state.knife_management_state
        );

        // Inform esp32 whether it should be managing the knife motor
        setpoint_tx.send(Setpoint {
            knife_management_state: state.knife_management_state,
            led_setpoint: LedSetpoint { brightness: 0.0 },
        });
    }
}
