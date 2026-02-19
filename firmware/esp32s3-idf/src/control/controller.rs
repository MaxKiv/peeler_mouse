use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Receiver};
use embassy_time::{Duration, Ticker};
use log::*;
use messenger_mouse::Setpoint;
const CONTROL_LOOP_FREQUENCY: Duration = Duration::from_hz(10);

#[embassy_executor::task]
pub async fn control_loop(mut setpoint_receiver: Receiver<'static, Cs, Setpoint, 1>) {
    info!("starting control task");

    // Track latest setpoint
    let mut latest_setpoint = messenger_mouse::Setpoint::default();

    // Task timekeeper
    let mut ticker = Ticker::every(CONTROL_LOOP_FREQUENCY);

    loop {
        // Update to latest setpoint, if any
        if let Some(new_setpoint) = setpoint_receiver.try_get() {
            latest_setpoint = new_setpoint;
        }

        // Act on latest setpoint
        if latest_setpoint.enable {}

        ticker.next().await;
    }
}

// // Construct pwms
// let mut pwm_a =
//     LedcDriver::new(channel_a, timer, pwm_pin_a).expect("unable to construct pwm a driver");
// let mut pwm_b =
//     LedcDriver::new(channel_b, timer, pwm_pin_b).expect("unable to construct pwm a driver");
//
// // Reset duty cycles
// if let Err(err) = pwm_a.set_duty(0) {
//     warn!("Control: {err}");
// }
// if let Err(err) = pwm_b.set_duty(0) {
//     warn!("Control: {err}");
// }
//
// loop {
//     match select(ticker.next(), setpoint_receiver.changed()).await {
//         embassy_futures::select::Either::First(_) => {
//             // Timer passed, wait for next tick
//             log::info!("Control task tick");
//         }
//         embassy_futures::select::Either::Second(setpoint) => {
//             let dc = setpoint.get_depth_dutycycle();
//
//             log::info!("Control task received new setpoint: {setpoint:?} - setting dc: {dc}");
//
//             if let Err(err) = pwm_a.set_duty(setpoint.get_depth_dutycycle()) {
//                 error!("Control: {err}");
//             }
//         }
//     }
// }
