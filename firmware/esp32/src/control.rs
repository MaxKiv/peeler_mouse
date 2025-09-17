use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use embassy_time::{Duration, Ticker};
use log::*;

use crate::Setpoint;

/// Period at which this task is ticked
const TASK_DURATION: Duration = Duration::from_millis(1000);

#[embassy_executor::task]
pub async fn control_loop(mut setpoint_receiver: Receiver<'static, Cs, Setpoint, 1>) {
    info!("starting control task");

    // Task timekeeper
    let mut ticker = Ticker::every(TASK_DURATION);

    loop {
        match select(ticker.next(), setpoint_receiver.changed()).await {
            embassy_futures::select::Either::First(_) => {
                // Timer passed, wait for next tick
                log::info!("Control task tick");
            }
            embassy_futures::select::Either::Second(setpoint) => {
                log::info!("Control task received new setpoint: {setpoint:?}");
            }
        }
    }
}
