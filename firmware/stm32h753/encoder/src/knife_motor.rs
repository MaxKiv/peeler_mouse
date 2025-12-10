use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{Either3, select3};
use embassy_stm32::gpio::Output;
use embassy_stm32::peripherals::*;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Delay, Duration, Ticker};
use l9110::{Direction, L9110};
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

use crate::button::{
    BUTTON_WATCH_SIZE, ButtonPeripherals, ButtonPressed, DebouncedButton, WATCH_BUTTON,
};
use crate::pot::WATCH_POT;

static MOTOR_ENABLED: Watch<CriticalSectionRawMutex, bool, BUTTON_WATCH_SIZE> = Watch::new();
static MOTOR_DIRECTION: Watch<CriticalSectionRawMutex, Direction, BUTTON_WATCH_SIZE> = Watch::new();

pub struct KnifeMotorPeripherals {
    pub pwm: SimplePwm<'static, TIM12>,
    pub enable_button: ButtonPeripherals<PE2>,
    pub direction_button: ButtonPeripherals<PB15>,
}

pub fn setup(p: KnifeMotorPeripherals, spawner: &Spawner) {
    info!("Setting up motors");

    let enable_button =
        DebouncedButton::new(p.enable_button, &MOTOR_ENABLED, "Knife enable", spawner);
    let dir_button = DebouncedButton::new(
        p.direction_button,
        &MOTOR_ENABLED,
        "Knife direction",
        spawner,
    );

    let l9110 = L9110::try_new("Knife motor", p.pwm, p.dir, p.enable, embassy_time::Delay).unwrap();

    spawner.spawn(latch_motor_movement(l9110)).unwrap();
    spawner.spawn(manage_knife_motor()).unwrap();
}

#[embassy_executor::task]
pub async fn manage_knife_motor() {
    let mut rx_enabled = WATCH_BUTTON
        .receiver()
        .expect("Not enough watch button receivers");

    let tx_enabled = MOTOR_ENABLED.sender();
    let tx_dir = MOTOR_DIRECTION.sender();

    info!("Starting to manage translation motor");

    let mut moving = false;
    let mut dir = Direction::Forward;
    loop {
        let button = rx_enabled.changed().await;

        use ButtonPressed::*;
        match button {
            b @ Button2 => {
                moving = !moving;
                info!(
                    "Motor task received button press: {:?} - {} motor",
                    b,
                    if moving { "moving" } else { "stopping" }
                );
                tx_enabled.send(moving);
            }
            b @ Button3 => {
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
async fn latch_motor_movement(mut l9110: L9110<TIM12, Output<'static>, Delay>) {
    const UPDATE_PERIOD: Duration = Duration::from_millis(100);

    let mut ticker = Ticker::every(UPDATE_PERIOD);

    let mut rx_enabled = MOTOR_ENABLED.receiver().expect("increase MOTOR_ENABLED N");
    let mut rx_direction = MOTOR_DIRECTION
        .receiver()
        .expect("increase MOTOR_DIRECTION N");
    let mut rx_pot = WATCH_POT.receiver().expect("increase WATCH_POT N");

    loop {
        let should_step = rx_enabled.get().await;
        if should_step {
            l9110.start();
        } else {
            l9110.stop();
        }

        let pot = rx_pot.get().await;
        let speed = pot_to_speed(pot);
        l9110.set_speed(speed);

        // continue for 100ms or until a new enable or direction setpoint is received
        match select3(ticker.next(), rx_enabled.changed(), rx_direction.changed()).await {
            Either3::First(_) => {} // 100ms expired -> check for speed changes in next iteration
            Either3::Second(_) => {} // motor enabled/disabled -> next iteration
            Either3::Third(direction) => {
                // new direction, change direction and restart loop
                l9110.set_direction(direction);
            }
        }
    }
}

fn pot_to_speed(pot: u16) -> Velocity {
    const MAX_POT: u16 = u16::MAX;
    const MAX_VELOCITY_MM_PS: f32 = 1.0;

    Velocity::new::<millimeter_per_second>((pot / MAX_POT) as f32 * MAX_VELOCITY_MM_PS)
}
