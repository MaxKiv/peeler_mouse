use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::{PinDriver, Pull},
    ledc::{config::TimerConfig, LedcTimerDriver},
};
use log::{error, info};
use simple_stepper_driver::SimpleStepperDriver;

use crate::actuation::stepper::{
    command::{MotorCommand, MotorDirection},
    peripherals::MotorPeripherals,
    HomeStatus,
};

pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();
pub static KNIFE_MOTOR_HOME: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 1.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
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
    let mut limit_switch =
        PinDriver::input(p.limit_switch).expect("unable to create the limit switch pin driver");
    limit_switch
        .set_pull(Pull::Up)
        .expect("Unable to set limit switch to pull up");

    let mut driver = SimpleStepperDriver::try_new("Knife", timer, p.channel, p.pwm_pin, dir, Delay)
        .expect("Unable to construct knife motor driver");

    let mut rx = KNIFE_MOTOR_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");

    let home_tx = KNIFE_MOTOR_HOME.sender();

    info!("Starting to manage knife motor");

    // start disabled & lost
    driver
        .stop()
        .await
        .expect("Unable to stop motor driver before manage_knife_motor main loop");
    home_tx.send(HomeStatus::Lost);

    loop {
        let cmd = rx.changed().await;

        if let Err(err) = match cmd.clone() {
            MotorCommand::Halt => {
                info!("KNIFE: Halting motor");
                driver.stop().await
            }
            MotorCommand::MoveVelocity(sp) => {
                info!("KNIFE: Moving in direction {:?}", sp.dir);
                driver.run(sp.dir.into()).await
            }
            MotorCommand::Home => {
                info!("KNIFE: Homing motor");

                // Move in homing direction
                if let Err(err) = driver.run(HOMING_DIRECTION.into()).await {
                    error!(
                        "KNIFE: homing: unable to move in HOMING_DIRECTION: {:?}, ignoring home command",
                        err
                    );
                    continue;
                }

                // Wait for limit_switch to indicate home reached
                loop {
                    if let Err(err) = limit_switch.wait_for_falling_edge().await {
                        error!(
                            "KNIFE: Unable to wait for limit_switch edge: {:?}, retrying",
                            err
                        );
                        // try again
                        continue;
                    }

                    // debounce limit switch
                    Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;
                    if limit_switch.is_low() {
                        // valid home position, stop ze motor!
                        if let Err(err) = driver.stop().await {
                            break Err(err);
                        }

                        home_tx.send(HomeStatus::Homed);

                        info!("KNIFE: Home reached");
                        break Ok(());
                    }
                }
            }
        } {
            error!("KNIFE: error handling command {:?} => {:?}", cmd, err);
            home_tx.send(HomeStatus::Lost);

            let _ = driver.stop().await;
        }
    }
}
