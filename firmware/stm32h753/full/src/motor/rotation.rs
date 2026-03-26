use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::{Peri, peripherals::*};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Delay;
use messenger_mouse::motor::MotorCommand;
use tb6600::Tb6600;
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

use crate::hmi::button::BUTTON_WATCH_SIZE;

pub static ROTATION_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();

pub struct RotationMotorPeripherals {
    pub pwm: SimplePwm<'static, TIM4>,
    pub dir: Peri<'static, PF9>,
}

pub fn setup(p: RotationMotorPeripherals, spawner: &Spawner) {
    info!("Setting up rotational motor");

    // Set up TB6600 Direction pin
    let dir = Output::new(p.dir, Level::Low, Speed::VeryHigh);
    // Set up TB6600 motordriver
    let tb = Tb6600::new("Rotational", p.pwm, dir, embassy_time::Delay);

    spawner.spawn(manage_rotational_motor(tb)).unwrap();
}

#[embassy_executor::task]
pub async fn manage_rotational_motor(mut tb: Tb6600<TIM4, Output<'static>, Delay>) {
    info!("Starting to manage rotational motor");

    // start disabled
    tb.stop();
    let mut rx = ROTATION_SETPOINT
        .receiver()
        .expect("increase ROTATION_WATCH N");

    loop {
        let cmd = rx.changed().await;

        match cmd {
            MotorCommand::Halt => tb.stop(),
            MotorCommand::Home => {
                error!("Home not implemented yet");
            }
            MotorCommand::MoveVelocity(sp) => {
                let freq = rotational_speed_to_step_freq(sp.speed);

                if let Err(err) = tb.run_hertz(freq, sp.dir).await {
                    error!("Unable to MoveVelocity: {:?}", err);
                }
            }
            MotorCommand::MovePosition(_sp) => {
                error!("MovePosition not implemented yet");
            }
        };
    }
}

// Calculates the required step frequency to obtain the desired rotational velocity
// NOTE: this takes a linear velocity and transforms it into a rotational one,
// using 1mm/s == 1 rot/minute
// TODO: figure out how to make MotorVelocitySetpoints generic to lin/rot.
fn rotational_speed_to_step_freq(speed: Velocity) -> Hertz {
    const ROTATIONS_PM_PER_HZ: f32 = 4.897 / 2500.0; // Experimentally validated
    const HZ_FOR_1_ROT_PM: f32 = 1.0 / ROTATIONS_PM_PER_HZ; // ~510Hz for 1 rot/min

    let rot_pm = speed.get::<millimeter_per_second>().abs(); // Yea sorry
    let freq: u32 = (rot_pm * HZ_FOR_1_ROT_PM) as u32;

    info!("ROT: converted {}rot/min > {}hz", rot_pm, freq);

    Hertz(freq)
}
