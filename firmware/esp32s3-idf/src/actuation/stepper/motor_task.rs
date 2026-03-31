use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Timer};
use esp_idf_hal::{
    gpio::{Level, PinDriver, Pull},
    rmt::{TxRmtConfig, TxRmtDriver},
};
use log::*;
use messenger_mouse::motor::MotorDirection;
use rmt_stepper_driver::RmtStepper;
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::actuation::stepper::{
    limit_switch_task::{manage_limit_switch, LimitSwitchState, LIMIT_EVENT},
    peripherals::{MotorPeripherals, StepperPeripherals},
    HomeStatus, MotorAction,
};

// pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();
// pub static KNIFE_MOTOR_HOME: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
// static MOTOR_CONTROLLER: Watch<CriticalSectionRawMutex, MotorAction, 1> = Watch::new();

/// Public: callers write MotorAction here
pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorAction, 2> = Watch::new();
/// Public: anyone can read homing status
pub static KNIFE_MOTOR_HOME: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
/// Public: live step-position (0 = home)
pub static KNIFE_MOTOR_POS: Watch<CriticalSectionRawMutex, i32, 2> = Watch::new();
/// Internal: control task → stepper task
static STEPPER_CMD: Watch<CriticalSectionRawMutex, StepperCommand, 1> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 1.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;
pub const LIMIT_SWITCH_DEBOUNCE_DURATION: Duration = Duration::from_millis(5);

#[derive(Clone, Debug, PartialEq)]
pub struct StepperCommand {
    pub dir: MotorDirection,
    pub interval: Duration, // derived from Velocity
    pub running: bool,      // false = hold position / coast
}

impl StepperCommand {
    pub const STOPPED: Self = Self {
        dir: MotorDirection::Forward,
        interval: Duration::from_millis(10),
        running: false,
    };
}

// Spawn all COMMS & FRAMING tasks required for external communications
pub fn run(spawner: &Spawner, p: MotorPeripherals) -> anyhow::Result<()> {
    log::info!("initialising knife motor task");

    spawner.spawn(manage_limit_switch(p.limit_switch))?;
    spawner.spawn(control_knife_motor())?;
    spawner.spawn(stepper_task(p.stepper))?;

    Ok(())
}

// Control task
#[embassy_executor::task]
pub async fn control_knife_motor() {
    let cmd_tx = STEPPER_CMD.sender();
    let home_tx = KNIFE_MOTOR_HOME.sender();
    let mut limit_rx = LIMIT_EVENT.receiver().unwrap();
    let mut action_rx = KNIFE_MOTOR_SETPOINT.receiver().unwrap();

    // Start stopped and lost
    cmd_tx.send(StepperCommand::STOPPED);
    home_tx.send(HomeStatus::Lost);

    let mut current = StepperCommand::STOPPED;
    let mut home_status = HomeStatus::Lost;

    // Main motor control loop
    // Transform target speed into appropriate step period and relay to stepper driver
    // Check for limit switch activation
    loop {
        match select(action_rx.changed(), limit_rx.changed()).await {
            // New target action received
            Either::First(action) => {
                let next = match action {
                    MotorAction::Stop => StepperCommand::STOPPED,

                    MotorAction::Velocity { dir, speed } => StepperCommand {
                        dir,
                        interval: velocity_to_interval(speed),
                        running: true,
                    },

                    MotorAction::Home => {
                        // Entering home mode always resets status to Lost
                        home_status = HomeStatus::Lost;
                        home_tx.send(home_status.clone());
                        StepperCommand {
                            dir: HOMING_DIRECTION,
                            interval: velocity_to_interval(Velocity::new::<millimeter_per_second>(
                                HOMING_SPEED_MM_PS,
                            )),
                            running: true,
                        }
                    }
                };

                current = next;
                cmd_tx.send(current.clone());
            }

            // limit_switch event
            Either::Second(level) => {
                // Stop the motor immediately, then debounce
                cmd_tx.send(StepperCommand::STOPPED);
                Timer::after(LIMIT_SWITCH_DEBOUNCE_DURATION).await;

                // Re-read the pin from here (borrow the pin, or read through a shared Watch)
                // If the line is still low → genuine home
                if limit_rx.get().await == LimitSwitchState::Active {
                    // TODO Possibly move back untill switch != LIMIT_SWITCH_ENGAGE_LEVEL

                    info!("KNIFE: home confirmed");
                    // Record home. Stepper task zeroes its counter when it sees running=false
                    // at a known position — or we can send a dedicated zero command if needed.
                    home_status = HomeStatus::Homed { position: 0 };
                    home_tx.send(home_status.clone());
                    // Stay stopped; caller must send next MotorAction
                } else {
                    warn!("KNIFE: spurious limit trigger, resuming");
                    cmd_tx.send(current.clone()); // resume homing
                }
            }
        }
    }
}

fn velocity_to_interval(_speed: Velocity) -> Duration {
    warn!("TODO: velocity_to_interval");
    Duration::from_hz(1_000)
}

// Stepper task
#[embassy_executor::task]
pub async fn stepper_task(p: StepperPeripherals) {
    // --- hardware init ---
    let rmt_driver = TxRmtDriver::new(p.rmt_channel, p.step_rmt_pin, &TxRmtConfig::new()).unwrap();
    let dir_pin = PinDriver::output(p.dir_pin).unwrap();
    let mut driver = RmtStepper::new("KNIFE", rmt_driver, dir_pin, Delay);

    let pos_tx = KNIFE_MOTOR_POS.sender();
    let limit_tx = LIMIT_EVENT.sender();
    let mut cmd_rx = STEPPER_CMD.receiver().unwrap();

    let mut cmd = StepperCommand::STOPPED;
    let mut position: i32 = 0;

    let mut step_timer = Timer::after(cmd.interval);
    // Main stepper control loop: does in parallel:
    // 1. Check for new stepper command
    // 2. Looks at the limit switch, informs others if it is hit
    // 3. Service current stepper command by stepping at the correct period
    // Note: During execution of 1 & 3 it is possible to miss a limit switch edge
    // I think this is no problem as the execution times of a single step are of period ~1ms
    loop {
        match select3(cmd_rx.changed(), step_timer).await {
            Either3::First(new_cmd) => {
                if new_cmd.dir != cmd.dir {
                    driver.set_direction(new_cmd.dir.clone()).await;
                }
                if !new_cmd.running {
                    driver.stop();
                }
                cmd = new_cmd;
                // Reset timer so we don't immediately step at the old interval
                step_timer = Timer::after(cmd.interval);
            }

            Either3::Second(_) => {
                // Notify motor controller of new limit switch state
                limit_tx.send(limit.get_level());
            }

            Either3::Third(()) => {
                if cmd.running {
                    driver.step_once().await;

                    // Update position
                    position += match cmd.dir {
                        MotorDirection::Forward => 1,
                        MotorDirection::Reverse => -1,
                    };
                    pos_tx.send(position);
                }
                // Re-arm for next step
                step_timer = Timer::after(cmd.interval);
            }
        }
    }
}
