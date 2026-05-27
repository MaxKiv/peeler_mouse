use defmt::*;
use embassy_time::{Duration, Ticker};
use messenger_mouse::{
    LedSetpoint, Setpoint,
    motor::{KnifeManager, MotorAction},
};

use crate::{comms::task::SETPOINT_WATCH, supervisor::task::APPSTATE_WATCH};

const TASK_PERIOD: Duration = Duration::from_millis(300);
const LED_BRIGHTNESS: f32 = 0.1;

/// Uses latest appstate to instruct ESP32
#[embassy_executor::task]
pub async fn supervise_esp() {
    info!("Starting to supervise ESP");

    let mut ticker = Ticker::every(TASK_PERIOD);

    let mut appstate_rx = APPSTATE_WATCH.receiver().unwrap();
    let setpoint_tx = SETPOINT_WATCH.sender();

    // Send known default setpoint on boot
    setpoint_tx.send(Setpoint::default());

    loop {
        // get latest appstate
        let state = appstate_rx.changed().await;

        if state.enable {
            if state.knife_manager == KnifeManager::Manual {
                warn!(
                    "ESP: Managed MANUAL with command {:?}",
                    state.knife_setpoint,
                );
            } else {
                warn!("ESP: Managed by VISION");
            }

            // Inform esp32 whether it should enable the vision algorithm
            // and if not what the knife motor should be doing.
            setpoint_tx.send(Setpoint {
                knife_manager: state.knife_manager,
                knife_setpoint: if state.enable {
                    state.knife_setpoint
                } else {
                    MotorAction::Coast
                },
                led_setpoint: LedSetpoint {
                    brightness: LED_BRIGHTNESS,
                },
            });
        } else {
            setpoint_tx.send(Setpoint::new_safe());
        }

        ticker.next().await;
    }
}
