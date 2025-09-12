use embassy_time::{Duration, Ticker};
use esp_idf_hal::gpio::{AnyOutputPin, Output, OutputMode, OutputPin, PinDriver};
use log::*;

/// Period at which this task is ticked
const LED_DURATION: Duration = Duration::from_millis(1000);

#[embassy_executor::task]
pub async fn blink_led(mut led: PinDriver<'static, AnyOutputPin, Output>) {
    info!("starting LED task");

    // Task timekeeper
    let mut ticker = Ticker::every(LED_DURATION / 2);

    loop {
        let _ = led.toggle();

        ticker.next().await;
    }
}
