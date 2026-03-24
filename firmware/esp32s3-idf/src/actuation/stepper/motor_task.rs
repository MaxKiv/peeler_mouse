use embassy_executor::Spawner;
use embassy_futures::select::Either;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::{PinDriver, Pull},
    ledc::{config::TimerConfig, LedcTimerDriver},
    rmt::{TxRmtConfig, TxRmtDriver},
};
use log::{error, info};
use rmt_stepper_driver::{RmtStepper, StepperError};
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

    // let timer = LedcTimerDriver::new(p.timer, &TimerConfig::default())
    //     .expect("unable to start motor timer driver");

    let dir = PinDriver::output(p.dir_pin).expect("unable to create the DIR pin driver");
    let mut limit_switch =
        PinDriver::input(p.limit_switch).expect("unable to create the limit switch pin driver");
    limit_switch
        .set_pull(Pull::Up)
        .expect("Unable to set limit switch to pull up");

    let rmt_cfg = TxRmtConfig::new();
    let mut rmt_driver = TxRmtDriver::new(p.rmt_channel, p.rmt_pin, &rmt_cfg)
        .expect("Stepper: unable to construct TxRmtDriver");

    let mut driver = RmtStepper::new("Knife", rmt_driver, dir, Delay);

    let mut rx = KNIFE_MOTOR_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");

    let home_tx = KNIFE_MOTOR_HOME.sender();

    info!("Starting to manage knife motor");

    // start disabled & lost
    driver.stop();
    home_tx.send(HomeStatus::Lost);

    loop {
        let cmd = rx.changed().await;

        if let Err(err) = match cmd.clone() {
            MotorCommand::Halt => {
                info!("KNIFE: Halting motor");
                // driver.stop().await
                Ok(driver.stop())
            }
            MotorCommand::MoveVelocity(sp) => {
                info!("KNIFE: Moving in direction {:?}", sp.dir);
                driver.set_direction(sp.dir.into());
                driver.run().await
            }
            MotorCommand::Home => {
                info!("KNIFE: Homing motor");

                // Move in homing direction
                driver.set_direction(HOMING_DIRECTION.into()).await;
                let mut found = false;

                for freq in 50..10000 {
                    info!("KNIFE: Homing at freq {}hz", freq);

                    driver.set_speed_hz(freq);
                    for _ in 1..10 {
                        driver.step_once().await;

                        if limit_switch.is_low() {
                            // debounce limit switch
                            Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;
                            if limit_switch.is_low() {
                                // valid home position, stop ze motor!
                                driver.stop();

                                home_tx.send(HomeStatus::Homed);

                                info!("KNIFE: Home reached");
                                found = true;
                                break;
                            }
                        }
                    }
                    if found {
                        break;
                    }
                }

                Ok(())
            }
        } {
            error!("KNIFE: error handling command {:?} => {:?}", cmd, err);
            home_tx.send(HomeStatus::Lost);

            let _ = driver.stop();
        }
    }
}
