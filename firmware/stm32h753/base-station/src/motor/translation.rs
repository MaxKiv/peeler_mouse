use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_stm32::{Peri, peripherals::*};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::Delay;
use messenger_mouse::motor::MotorAction;
use tb6600::Tb6600;
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

pub static TRANSLATION_SETPOINT: Watch<ThreadModeRawMutex, MotorAction, 2> = Watch::new();

pub struct TranslationMotorPeripherals {
    pub pwm: SimplePwm<'static, TIM8>,
    pub dir: Peri<'static, PF8>,
}

pub fn setup(p: TranslationMotorPeripherals, spawner: &Spawner) {
    info!("Setting up translation motor");

    // Set up TB6600 Direction pin
    let dir = Output::new(p.dir, Level::Low, Speed::VeryHigh);
    // Set up TB6600 motordriver
    let tb = Tb6600::new("Translation", p.pwm, dir, embassy_time::Delay);

    spawner.spawn(manage_translational_motor(tb)).unwrap();
}

#[embassy_executor::task]
/// MotorAction adapter for the translational motor
pub async fn manage_translational_motor(mut tb: Tb6600<TIM8, Output<'static>, Delay>) {
    info!("Starting to manage translational motor");

    // start disabled
    tb.stop();
    let mut rx = TRANSLATION_SETPOINT
        .receiver()
        .expect("increase TRANSLATION_WATCH N");

    loop {
        let cmd = rx.changed().await;

        match cmd {
            MotorAction::Hold => tb.stop(),
            MotorAction::Coast => tb.stop(),
            MotorAction::Home => {
                error!("Home not implemented yet");
            }

            MotorAction::MoveVelocity(sp) => {
                let freq = translational_speed_to_step_freq(sp.speed);

                if let Err(err) = tb.run_hertz(freq, sp.dir).await {
                    error!("Unable to MoveVelocity: {:?}", err);
                }
            }
            MotorAction::MovePosition(_sp) => {
                error!("MovePosition not implemented yet");
            }
        };
    }
}

// Calculates the required step frequency to obtain the desired rotational velocity
// NOTE: this takes a linear velocity and transforms it into a rotational one,
// using 1mm/s == 1 rot/minu
// TODO: figure out how to make MotorVelocitySetpoints generic to lin/rot.
fn translational_speed_to_step_freq(speed: Velocity) -> Hertz {
    const MM: f32 = 75.2; // experimental result
    const MM_PS: f32 = 0.3; // experimental result
    const SECONDS: f32 = 22.74; // experimental result
    const HERTZ: f32 = 750.0; // 0.3mm/s = 750hz

    const SPEED: f32 = MM / SECONDS; // 3.307 mm/s @ 750hz
    const HZ_PER_MM_PS: f32 = HERTZ / SPEED;

    let speed = speed.get::<millimeter_per_second>().abs();
    let freq = (speed * HZ_PER_MM_PS) as u32;

    info!("TRANS: converted {}mm/s > {}hz", speed, freq);

    Hertz(freq)
}
