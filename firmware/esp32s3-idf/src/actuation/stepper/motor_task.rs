use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::{PinDriver, Pull},
    rmt::{TxRmtConfig, TxRmtDriver},
};
use log::{error, info};
use messenger_mouse::motor::{MotorCommand, MotorDirection};
use rmt_stepper_driver::RmtStepper;
use uom::si::f32::Velocity;

use crate::actuation::stepper::{peripherals::MotorPeripherals, HomeStatus};

pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();
pub static KNIFE_MOTOR_HOME: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
static MOTOR_CONTROLLER: Watch<CriticalSectionRawMutex, MotorAction, 1> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 1.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;
pub const LIMIT_SWITCH_DEBOUNCE_DURATION: Duration = Duration::from_millis(5);

#[derive(Clone)]
enum MotorAction {
    Stop,
    Velocity {
        dir: MotorDirection,
        speed: Velocity,
    },
    Home,
}

#[derive(Clone)]
enum MotorMode {
    Stopped,
    Homing,
    Velocity {
        dir: MotorDirection,
        speed: Velocity,
    },
    Position,
}

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: MotorPeripherals) -> anyhow::Result<()> {
    log::info!("initialising knife motor task");

    spawner.spawn(control_knife_motor(p))?;
    spawner.spawn(manage_knife_motor())?;

    Ok(())
}

#[embassy_executor::task]
pub async fn control_knife_motor(p: MotorPeripherals) {
    info!("KNIFE CONTROL: initialising motor driver");

    let dir = PinDriver::output(p.dir_pin).expect("unable to create the DIR pin driver");
    let mut limit_switch =
        PinDriver::input(p.limit_switch).expect("unable to create the limit switch pin driver");
    limit_switch
        .set_pull(Pull::Up)
        .expect("Unable to set limit switch to pull up");

    let rmt_cfg = TxRmtConfig::new();
    let rmt_driver = TxRmtDriver::new(p.rmt_channel, p.step_rmt_pin, &rmt_cfg)
        .expect("Stepper: unable to construct TxRmtDriver");

    let mut driver = RmtStepper::new("KNIFE", rmt_driver, dir, Delay);

    let home_tx = KNIFE_MOTOR_HOME.sender();

    info!("KNIFE CONTROL: Starting to control knife motor");

    // start disabled & lost
    let mut home_status = HomeStatus::Lost;
    home_tx.send(home_status.clone());
    driver.stop();
    let mut mode = MotorMode::Stopped;

    let mut motorcontroller_rx = MOTOR_CONTROLLER.receiver().unwrap();

    // Big state machine
    loop {
        let action = motorcontroller_rx.get().await;

        // Check for state transitions
        match (mode.clone(), action) {
            (MotorMode::Stopped, MotorAction::Velocity { dir, speed }) => {
                mode = MotorMode::Velocity { dir, speed };
            }
            (MotorMode::Stopped, MotorAction::Home) => {
                mode = MotorMode::Homing;
            }
            (MotorMode::Homing, MotorAction::Stop) => {
                mode = MotorMode::Stopped;
            }
            (MotorMode::Homing, MotorAction::Velocity { dir, speed }) => {
                if home_status == HomeStatus::Homed {
                    mode = MotorMode::Velocity { dir, speed };
                }
            }
            (MotorMode::Velocity { dir: _, speed: _ }, MotorAction::Stop) => {
                mode = MotorMode::Stopped;
            }
            (MotorMode::Velocity { dir: _, speed: _ }, MotorAction::Velocity { dir, speed }) => {
                mode = MotorMode::Velocity { dir, speed };
            }
            (MotorMode::Velocity { dir: _, speed: _ }, MotorAction::Home) => {
                mode = MotorMode::Homing;
            }
            (MotorMode::Position, MotorAction::Stop) => {
                mode = MotorMode::Stopped;
            }
            (MotorMode::Position, MotorAction::Velocity { dir, speed }) => {
                mode = MotorMode::Velocity { dir, speed };
            }
            (MotorMode::Position, MotorAction::Home) => {
                mode = MotorMode::Homing;
            }
            _ => {}
        }

        // Actuate state machine
        match mode {
            MotorMode::Stopped => {
                driver.stop();
            }
            MotorMode::Velocity {
                dir: new_dir,
                speed: _,
            } => {
                driver.set_direction(new_dir).await;
                driver.step_once().await;
            }
            MotorMode::Position => {
                driver.step_once().await;
            }
            MotorMode::Homing => {
                if limit_switch.is_low() {
                    // debounce limit switch
                    Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;
                    if limit_switch.is_low() {
                        // valid home position, stop the motor!
                        driver.stop();
                        home_tx.send(HomeStatus::Homed);
                        info!("KNIFE: Home reached");

                        // Inform other of new homing status
                        home_status = HomeStatus::Homed;
                        home_tx.send(home_status);
                    }
                }
                // Continue moving in home direction
                driver.set_direction(HOMING_DIRECTION).await;
                driver.step_once().await;
            }
        }

        break;
    }
}

#[embassy_executor::task]
pub async fn manage_knife_motor() {
    let motorcontroller_tx = MOTOR_CONTROLLER.sender();

    let mut rx = KNIFE_MOTOR_SETPOINT
        .receiver()
        .expect("increase KNIFE_SETPOINT N");

    loop {
        let cmd = rx.changed().await;

        match cmd {
            MotorCommand::Halt => {
                motorcontroller_tx.send(MotorAction::Stop);
            }
            MotorCommand::MoveVelocity(sp) => {
                motorcontroller_tx.send(MotorAction::Velocity {
                    dir: sp.dir,
                    speed: sp.speed,
                });
            }
            MotorCommand::Home => {
                motorcontroller_tx.send(MotorAction::Home);
            }
            MotorCommand::MovePosition(sp) => {
                info!("KNIFE: actuating {:?}", sp);
                error!("TODO: position setpoints");
                todo!();
            }
        }
    }
}
