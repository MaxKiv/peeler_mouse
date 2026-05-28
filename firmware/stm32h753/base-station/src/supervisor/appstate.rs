use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use embassy_time::{Duration, Ticker};
use messenger_mouse::{
    ControlOutput,
    motor::{ControlMode, MotorSetpoints},
};

use crate::{
    comms::task::REPORT_WATCH,
    supervisor::{MotorTypes, task::HMI_STATE_WATCH},
};

// Throttle uart traffic
const TASK_PERIOD: Duration = Duration::from_millis(100);
pub static APP_STATE_WATCH: Watch<Cs, AppState, 3> = Watch::new();

pub const MOTORS: [MotorTypes; 3] = [
    MotorTypes::Cut,
    MotorTypes::Translation,
    MotorTypes::Rotation,
];

#[derive(Debug, Clone, defmt::Format)]
pub struct AppState {
    pub hmi_enable: bool,
    pub hmi_motor_setpoints: MotorSetpoints,
    pub hmi_control_mode: ControlMode,
    pub esp_motor_setpoints: MotorSetpoints,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            hmi_enable: false,
            hmi_motor_setpoints: MotorSetpoints::default(),
            hmi_control_mode: ControlMode::Manual,
            esp_motor_setpoints: MotorSetpoints::default(),
        }
    }
}

/// Watches HMI state and ESP Report for changes, combines into single Application state
#[embassy_executor::task]
pub async fn manage_appstate() {
    info!("Starting to manage appstate");

    // let mut ticker = Ticker::every(TASK_PERIOD);

    // ---- Receivers -----
    let mut hmi_state_rx = HMI_STATE_WATCH.receiver().unwrap();
    let mut report_rx = REPORT_WATCH.receiver().expect("increase REPORT_WATCH N");

    // ----- Motor controller setpoint tx -----
    let appstate_tx = APP_STATE_WATCH.sender();

    // ----- Init Application State -----
    let mut appstate = AppState::default();

    // Send known default appstate on boot
    appstate_tx.send(appstate.clone());

    loop {
        match select(hmi_state_rx.changed(), report_rx.changed()).await {
            Either::First(hmi_state) => {
                info!("APPSTATE: new hmi state: {:?}", hmi_state);
                appstate.hmi_enable = hmi_state.enable;
                appstate.hmi_control_mode = hmi_state.control_mode;
                appstate.hmi_motor_setpoints = hmi_state.motor_setpoints;
            }

            Either::Second(report) => {
                info!("APPSTATE: new report: {:?}", report);
                if let ControlOutput::Vision(effort) = report.control_output {
                    appstate.esp_motor_setpoints = effort.motor_setpoints;
                }
            }
        }

        appstate_tx.send(appstate.clone());

        // ticker.next().await;
    }
}
