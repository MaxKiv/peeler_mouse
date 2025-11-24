use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::*;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
};
use embassy_time::{Delay, Duration};
use tb6600::Tb6600;

use crate::button::{ButtonPressed, WATCH_BUTTON};

pub struct RotationalAxisMotorPeripherals {
    pub step: Peri<'static, PF7>,
    pub dir: Peri<'static, PF9>,
}

pub fn setup(p: RotationalAxisMotorPeripherals, spawner: &Spawner) {
    info!("Setting up motors");

    let step = Output::new(p.step, Level::Low, Speed::Low);
    let dir = Output::new(p.dir, Level::Low, Speed::Low);

    let tb = Tb6600::new(step, dir, embassy_time::Delay, 5);

    spawner.spawn(manage_rotational_motor(tb)).unwrap();
}

#[embassy_executor::task]
pub async fn manage_rotational_motor(mut tb: Tb6600<Output<'static>, Output<'static>, Delay>) {
    let mut rx = WATCH_BUTTON
        .receiver()
        .expect("Not enough watch button receivers");

    info!("Starting to manage motors");

    loop {
        let button = rx.changed().await;

        use ButtonPressed::*;
        match button {
            b @ Button4 => {
                info!(
                    "Motor task received button press: {:?} - stepping rotational motor",
                    b
                );
                if let Err(err) = tb.step_n(100).await {
                    error!("Err: {}", err);
                }
            }
            b @ Button5 => {
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
