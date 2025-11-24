use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_stm32::peripherals::*;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Timer};
use tb6600::Tb6600;

use crate::button::{ButtonPressed, WATCH_BUTTON};

static SHOULD_MOTOR_MOVE: Signal<CriticalSectionRawMutex, bool> = Signal::new();

pub struct LinearAxisMotorPeripherals {
    pub step: Peri<'static, PE3>,
    pub dir: Peri<'static, PF8>,
}

pub fn setup(p: LinearAxisMotorPeripherals, spawner: &Spawner) {
    info!("Setting up motors");

    let step = Output::new(p.step, Level::Low, Speed::Low);
    let dir = Output::new(p.dir, Level::Low, Speed::Low);

    let tb = Tb6600::new(step, dir, embassy_time::Delay, 5);

    spawner.spawn(manage_linear_motor(tb)).unwrap();
}

#[embassy_executor::task]
pub async fn manage_linear_motor(mut tb: Tb6600<Output<'static>, Output<'static>, Delay>) {
    let mut rx = WATCH_BUTTON
        .receiver()
        .expect("Not enough watch button receivers");

    info!("Starting to manage motors");

    loop {
        let button = rx.changed().await;

        use ButtonPressed::*;
        match button {
            b @ Button2 => {
                info!(
                    "Motor task received button press: {:?} - stepping linear motor",
                    b
                );
                if let Err(err) = tb.step_n(1000).await {
                    error!("Err: {}", err);
                }
            }
            b @ Button3 => {
                info!(
                    "Motor task received button press: {:?} - reversing direction",
                    b
                );
                if let Err(err) = tb.flip_direction().await {
                    error!("Err: {}", err);
                }
            }
            b => info!("Motor task ignoring button {:?}", b),
        }
    }
}
