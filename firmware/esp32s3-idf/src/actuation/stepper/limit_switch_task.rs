use embassy_futures::select::select;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Timer};
use esp_idf_hal::gpio::{self, Gpio1, Input, InterruptType, Level, PinDriver, Pull};
use log::*;

use crate::actuation::stepper::{LimitSwitchState, LIMIT_EVENT};

pub const LIMIT_SWITCH_ENGAGE_LEVEL: gpio::Level = Level::Low;
pub const LIMIT_SWITCH_DEBOUNCE_DURATION: Duration = Duration::from_millis(10);

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
pub async fn manage_limit_switch(
    #[cfg(feature = "devkit")] pin: esp_idf_hal::gpio::Gpio45,
    #[cfg(feature = "pcb")] pin: esp_idf_hal::gpio::Gpio1,
) {
    log::info!("MOTOR: initialising limit switch task");
    let mut limit = PinDriver::input(pin).unwrap();
    limit.set_pull(Pull::Up).unwrap();

    let tx = LIMIT_EVENT.sender();

    let mut current = limit.get_level();
    if LimitSwitchState::Active == current.into() {
        tx.send(current.into());
    }
    warn!("LIMIT SWITCH: Start main routine with lvl {:?}", current);
    loop {
        let target = match current {
            Level::High => Level::Low, // happy path: seek active
            Level::Low => Level::High, // unhappy path: back off first
        };

        wait_for_level(&mut limit, target).await;
        error!("DETECTED LIMIT SWITCH {:?}", target);
        tx.send(target.into());

        current = target;
    }
}

async fn wait_for_level(limit: &mut PinDriver<'static, Gpio1, Input>, target: Level) {
    loop {
        let _ = match target {
            Level::Low => limit.wait_for_falling_edge().await,
            Level::High => limit.wait_for_rising_edge().await,
        };

        // Debounce
        Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;
        if limit.get_level() == target {
            return;
        }
    }
}
