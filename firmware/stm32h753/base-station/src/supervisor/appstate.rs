use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use embassy_time::{Duration, Instant, Ticker};
use messenger_mouse::{
    ControlOutput,
    motor::{ControlMode, MotorSetpoints},
};

use crate::{
    comms::task::REPORT_WATCH,
    supervisor::{HmiState, MotorTypes, task::HMI_STATE_WATCH},
};

// Throttle uart traffic
const TASK_PERIOD: Duration = Duration::from_millis(100);
pub static APP_STATE_WATCH: Watch<Cs, AppState, 3> = Watch::new();

pub const MOTORS: [MotorTypes; 3] = [
    MotorTypes::Cut,
    MotorTypes::Rotation,
    MotorTypes::Translation,
];

#[derive(Debug, Clone, defmt::Format)]
pub struct AppState {
    pub hmi_state: HmiState,
    pub esp_motor_setpoints: MotorSetpoints,
    pub camera_fps_data: heapless::Vec<(f64, f64), 32>,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            hmi_state: HmiState::default(),
            esp_motor_setpoints: MotorSetpoints::default(),
            camera_fps_data: heapless::Vec::new(),
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
                appstate.hmi_state = hmi_state;
            }

            Either::Second(report) => {
                info!("APPSTATE: new report: {:?}", report);
                if let ControlOutput::Vision(effort) = report.control_output {
                    appstate.esp_motor_setpoints = effort.motor_setpoints;
                }

                if let Some(data) = report.measurements.vision_data {
                    if let Err(err) = appstate
                        .camera_fps_data
                        .insert(0, (Instant::now().as_millis() as f64, data.camera_fps))
                    {
                        defmt::error!("APPSTATE: {:?}", err);
                    }
                } else {
                    defmt::warn!("APPSTATE: report.measurements.vision_data = NONE");
                }
            }
        }

        appstate_tx.send(appstate.clone());

        // ticker.next().await;
    }
}
