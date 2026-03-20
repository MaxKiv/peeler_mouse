use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration};
use esp_idf_hal::{
    gpio::PinDriver,
    ledc::{config::TimerConfig, LedcTimerDriver},
};
use log::{error, info};
use simple_stepper_driver::SimpleStepperDriver;

use crate::actuation::stepper::{
    command::{MotorCommand, MotorDirection},
    peripherals::MotorPeripherals,
};

pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 1.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;
pub const LIMIT_SWITCH_DEBOUNCE_DURATION: Duration = Duration::from_millis(5);

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: MotorPeripherals) -> anyhow::Result<()> {
    log::info!("initialising knife motor task");

    spawner.spawn(manage_knife_motor(p))?;

    Ok(())
}

#[embassy_executor::task]
pub async fn manage_knife_motor(p: MotorPeripherals) {
    info!("KNIFE: initialising motor driver");

    let timer = LedcTimerDriver::new(p.timer, &TimerConfig::default())
        .expect("unable to start motor timer driver");

    let dir = PinDriver::output(p.dir_pin).expect("unable to create the DIR pin driver");
    let limit_switch =
        PinDriver::input(p.limit_switch).expect("unable to create the limit switch pin driver");

    let driver = SimpleStepperDriver::try_new("Knife", timer, p.channel, p.pwm_pin, dir, Delay)
        .expect("Unable to construct knife motor driver");

    info!("Starting to manage knife motor");
    // start disabled
    driver.stop();

    let mut rx = KNIFE_MOTOR_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");

    let mut homed = false;

    loop {
        let cmd = rx.changed().await;

        if let Err(err) = match cmd {
            MotorCommand::Halt => driver.stop().await,
            MotorCommand::MoveVelocity(sp) => driver.run(sp.dir.into()).await,
            MotorCommand::Home => {
                // Move in homing direction
                driver.run(HOMING_DIRECTION.into());

                // Wait for limit_switch to indicate home reached
                loop {
                    p.limit_switch.wait_for_falling_edge().await;

                    // debounce
                    Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;
                    if lf.limit_switch.is_low() {
                        // valid home position, stop motor
                        driver.stop().await;

                        homed = true;

                        return;
                    }
                }
            } // MotorCommand::MovePosition(sp) => todo!(),
        } {
            error!("KNIFE: error handling command {:?} => {:?}", cmd, err);
        }
    }
}
