use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::peripherals::*;
use embassy_stm32::{
    Peri,
    gpio::{Level, Output, Speed},
};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Timer};
use tb6600::{Direction, Tb6600};

use crate::button::{ButtonPressed, WATCH_BUTTON};
use crate::pot::WATCH_POT;

static MOTOR_ENABLED: Watch<CriticalSectionRawMutex, bool, 1> = Watch::new();
static MOTOR_DIRECTION: Watch<CriticalSectionRawMutex, Direction, 1> = Watch::new();

pub struct RotationalAxisMotorPeripherals {
    pub step: Peri<'static, PF7>,
    pub dir: Peri<'static, PF9>,
}

pub fn setup(p: RotationalAxisMotorPeripherals, spawner: &Spawner) {
    info!("Setting up motors");

    let step = Output::new(p.step, Level::Low, Speed::Low);
    let dir = Output::new(p.dir, Level::Low, Speed::Low);

    let tb = Tb6600::new(step, dir, embassy_time::Delay, 5);

    spawner.spawn(latch_motor_movement(tb)).unwrap();
    spawner.spawn(manage_rotational_motor()).unwrap();
}

#[embassy_executor::task]
pub async fn manage_rotational_motor() {
    let mut rx_enabled = WATCH_BUTTON
        .receiver()
        .expect("Not enough watch button receivers");

    let tx_enabled = MOTOR_ENABLED.sender();
    let tx_dir = MOTOR_DIRECTION.sender();

    info!("Starting to manage motors");

    let mut dir = Direction::Forward;
    let mut moving = false;
    loop {
        let button = rx_enabled.changed().await;

        use ButtonPressed::*;
        match button {
            b @ Button4 => {
                moving = !moving;
                info!(
                    "Motor task received button press: {:?} - {} motor",
                    b,
                    if moving { "moving" } else { "stopping" }
                );
                tx_enabled.send(moving);
            }
            b @ Button5 => {
                dir = match dir.clone() {
                    Direction::Forward => Direction::Reverse,
                    Direction::Reverse => Direction::Forward,
                };

                info!(
                    "Motor task received button press: {:?} - switched direction to {:?}",
                    b, dir
                );
                tx_dir.send(dir.clone())
            }

            b => info!("Motor task ignoring button {:?}", b),
        }
    }
}

#[embassy_executor::task]
async fn latch_motor_movement(mut tb: Tb6600<Output<'static>, Output<'static>, Delay>) {
    let mut rx_enabled = MOTOR_ENABLED.receiver().expect("increase MOTOR_ENABLED N");
    let mut rx_direction = MOTOR_DIRECTION
        .receiver()
        .expect("increase MOTOR_DIRECTION N");
    let mut rx_speed = WATCH_POT.receiver().expect("increase WATCH_POT N");

    loop {
        let moving = rx_enabled.get().await;
        let pulse_period_us = rx_speed.get().await;

        if moving {
            // Motor running: step OR react to state change
            match select3(
                tb.step_once_with_period(pulse_period_us.into()),
                rx_enabled.changed(),
                rx_direction.changed(),
            )
            .await
            {
                Either3::First(_) => {
                    // step_once finished, loop again -> next step
                }
                Either3::Second(_) => {
                    // new value arrived, restart loop
                }
                Either3::Third(direction) => {
                    // new direction, change direction and restart loop
                    if let Err(err) = tb.set_direction(direction).await {
                        error!("Unable to change TB6600 direction: {:?}", err);
                    }
                }
            }
        } else {
            // Motor stopped: idle OR react to change
            match select3(
                Timer::after_millis(100),
                rx_enabled.changed(),
                rx_direction.changed(),
            )
            .await
            {
                Either3::First(_) => {}  // stay stopped
                Either3::Second(_) => {} // changed → next iteration
                Either3::Third(direction) => {
                    // new direction, change direction and restart loop
                    if let Err(err) = tb.set_direction(direction).await {
                        error!("Unable to change TB6600 direction: {:?}", err);
                    }
                }
            }
        }
    }
}
