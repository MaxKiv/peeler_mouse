use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use esp_idf_hal::gpio::{self, Level, PinDriver, Pull};

pub static LIMIT_EVENT: Watch<CriticalSectionRawMutex, LimitSwitchState, 1> = Watch::new();
pub const LIMIT_SWITCH_ENGAGE_LEVEL: gpio::Level = Level::Low;

#[derive(Clone, PartialEq)]
pub enum LimitSwitchState {
    Active,
    Inactive,
}

// Informs others about changes in limit switch state
#[embassy_executor::task]
pub async fn manage_limit_switch(pin: esp_idf_hal::gpio::Gpio45) {
    let mut limit = PinDriver::input(pin).unwrap();
    limit.set_pull(Pull::Up).unwrap();

    let tx = LIMIT_EVENT.sender();

    loop {
        limit.wait_for_any_edge().await;

        if limit.get_level() == LIMIT_SWITCH_ENGAGE_LEVEL {
            tx.send(LimitSwitchState::Active);
        } else {
            tx.send(LimitSwitchState::Inactive);
        }
    }
}
