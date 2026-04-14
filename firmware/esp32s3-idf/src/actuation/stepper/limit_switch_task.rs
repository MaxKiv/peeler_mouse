use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Timer};
use esp_idf_hal::gpio::{self, Level, PinDriver, Pull};
use log::info;

pub static LIMIT_EVENT: Watch<CriticalSectionRawMutex, LimitSwitchState, 1> = Watch::new();
pub const LIMIT_SWITCH_ENGAGE_LEVEL: gpio::Level = Level::Low;
pub const LIMIT_SWITCH_DEBOUNCE_DURATION: Duration = Duration::from_millis(5);

#[derive(Clone, PartialEq, Debug)]
pub enum LimitSwitchState {
    Active,
    Inactive,
}

impl From<gpio::Level> for LimitSwitchState {
    fn from(value: gpio::Level) -> Self {
        if value == LIMIT_SWITCH_ENGAGE_LEVEL {
            LimitSwitchState::Active
        } else {
            LimitSwitchState::Inactive
        }
    }
}

// Watches for legit (debounced) limit switch state changes
// Informs others about these changes
#[embassy_executor::task]
pub async fn manage_limit_switch(pin: esp_idf_hal::gpio::Gpio45) {
    log::info!("MOTOR: initialising limit switch task");
    let mut limit = PinDriver::input(pin).unwrap();
    limit.set_pull(Pull::Up).unwrap();

    let tx = LIMIT_EVENT.sender();

    loop {
        // Wait for any state change
        limit.wait_for_any_edge().await;
        // Save current state
        let edge = limit.get_level();

        info!("LIMIT SWITCH {:?}", edge);

        // Debounce this limit switch press
        Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;

        if limit.get_level() == edge {
            // Legit press: inform others
            info!("LIMIT SWITCH {:?}", edge);
            tx.send(edge.into());
        }
    }
}
