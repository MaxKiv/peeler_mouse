use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use log::*;
use messenger_mouse::motor::{
    MotorCommand, MotorDirection, MotorPositionSetpoint, MotorVelocitySetpoint,
};
use uom::si::{
    f32::{Length, Velocity},
    length::millimeter,
    velocity::millimeter_per_second,
};

use crate::actuation::stepper::{
    limit_switch_task::{LimitSwitchState, LIMIT_EVENT},
    low_level::{low_level_task::STEPPER_CMD, state_machine::SPS},
    peripherals::{MotorPeripherals, StepperPeripherals},
    HomeStatus, MotorAction, PositionModeStatus, Steps,
};

/// Public: callers write MotorAction here
pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorCommand, 2> = Watch::new();
/// Public: anyone can read homing status
pub static KNIFE_MOTOR_HOME_STATUS: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
/// Public: upstream position mode status
pub static KNFIE_MOTOR_POS_STATUS: Watch<CriticalSectionRawMutex, PositionModeStatus, 2> =
    Watch::new();
/// Public: live step-position (0 = home)
pub static KNIFE_MOTOR_POS: Watch<CriticalSectionRawMutex, Steps, 2> = Watch::new();
pub static KNIFE_MOTOR_POS_RESET: Watch<CriticalSectionRawMutex, (), 1> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 0.1;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;

/// const MINIMUM_SPS: Duration = Duration::from_hz(100); // SPS
pub const MINIMUM_SPS: SPS = SPS(100);
/// pub const MINIMUM_SPS: Duration = Duration::from_hz(100); // SPS
pub const MAXIMUM_SPS: SPS = SPS(10_000); // SPS
/// Minimum SPS to switch direction
pub const MINIMUM_TRANSITION_SPS: SPS = SPS(200);

#[derive(Clone, Debug, PartialEq)]
pub enum StepperCommand {
    Coast,
    Holding,
    SingleStep,
    Velocity(Velocity),
}

impl StepperCommand {
    pub fn stopped() -> Self {
        Self::Coast
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VelocityCommand {
    pub dir: MotorDirection,
    pub velocity: Velocity,
}

// Control task
#[embassy_executor::task]
pub async fn control_knife_motor() {
    info!("MOTOR: control_knife_motor entry");

    let cmd_tx = STEPPER_CMD.sender();
    let home_tx = KNIFE_MOTOR_HOME_STATUS.sender();
    let pos_tx = KNFIE_MOTOR_POS_STATUS.sender();
    let mut limit_rx = LIMIT_EVENT.receiver().unwrap();
    let mut action_rx = KNIFE_MOTOR_SETPOINT.receiver().unwrap();
    let mut pos_rx = KNIFE_MOTOR_POS.receiver().unwrap();

    // Start stopped and lost
    cmd_tx.send(StepperCommand::stopped());
    home_tx.send(HomeStatus::Lost);
    pos_tx.send(PositionModeStatus::Reached);

    let mut current_action = MotorCommand::default();
    let mut current_cmd = StepperCommand::stopped();
    let mut home_status = HomeStatus::Lost;
    let mut current_pos_steps = pos_rx.get().await;
    let mut target_pos_steps = Steps(0);

    // Main motor control loop
    // Transform target speed into appropriate step period and relay to stepper driver
    // Check for limit switch activation
    loop {
        match select3(action_rx.changed(), limit_rx.changed(), pos_rx.changed()).await {
            // New motor command received
            Either3::First(cmd) => {
                info!("MOTOR: new command received: {:?}", cmd);
                let next_cmd = match cmd.clone() {
                    MotorCommand::Halt => StepperCommand::stopped(),
                    MotorCommand::MoveVelocity(MotorVelocitySetpoint { dir, speed }) => {
                        StepperCommand {
                            dir,
                            period: velocity_to_interval(speed),
                            running: true,
                        }
                    }
                    MotorCommand::Home => {
                        home_status = HomeStatus::Lost;
                        home_tx.send(home_status.clone());
                        StepperCommand {
                            dir: HOMING_DIRECTION,
                            period: velocity_to_interval(Velocity::new::<millimeter_per_second>(
                                HOMING_SPEED_MM_PS,
                            )),
                            running: true,
                        }
                    }
                    MotorCommand::MovePosition(MotorPositionSetpoint { target, speed }) => {
                        target_pos_steps = position_to_steps(target);

                        if target_pos_steps == current_pos_steps {
                            // Target already reached, inform upstream and stop stepper
                            pos_tx.send(PositionModeStatus::Reached);
                            StepperCommand::STOPPED
                        } else {
                            let dir = if target_pos_steps > current_pos_steps {
                                MotorDirection::Forward
                            } else {
                                MotorDirection::Reverse
                            };

                            // Inform upstream we are starting a new position mode action
                            pos_tx.send(PositionModeStatus::InProgress);

                            StepperCommand {
                                dir,
                                period: velocity_to_interval(speed),
                                running: true,
                            }
                        }
                    }
                };

                // Send new StepperCommand & Bookkeeping
                current_cmd = next_cmd;
                cmd_tx.send(current_cmd.clone());
                current_action = cmd;
            }

            // limit_switch event
            Either3::Second(level) => {
                info!("MOTOR: Limit switch event: {:?}", level);
                if level == LimitSwitchState::Active {
                    info!("KNIFE: home switch active detected, moving back");
                    // Move back untill switch disengages
                    current_cmd = StepperCommand {
                        dir: HOMING_DIRECTION.get_opposite(),
                        period: velocity_to_interval(Velocity::new::<millimeter_per_second>(
                            HOMING_SPEED_MM_PS,
                        )),
                        running: true,
                    };

                    cmd_tx.send(current_cmd.clone());
                } else {
                    // Limit switch disengaged, we are now Homed
                    // Stop stepping
                    cmd_tx.send(StepperCommand::stopped());

                    // Record home
                    info!("KNIFE: home confirmed");
                    home_status = HomeStatus::Homed { position: 0 };
                    home_tx.send(home_status.clone());

                    // Stay stopped _> caller must send next MotorAction
                }
            }

            // Track position information from stepper task
            Either3::Third(new_pos) => {
                // info!("MOTOR: new position information: {:?}", new_pos);
                // Track current pos
                current_pos_steps = new_pos;

                // Are we doing a position mode action?
                if let MotorCommand::MovePosition(_) = current_action {
                    // Did we reach position target?
                    if current_pos_steps == target_pos_steps {
                        info!("MOTOR: Reached target position: {:?}", target_pos_steps);

                        // Inform upstream
                        pos_tx.send(PositionModeStatus::Reached);

                        // Position target reached -> stop motors
                        cmd_tx.send(StepperCommand::stopped());
                    }
                }
            }
        }
    }
}

/// Converts a position target (distance from home) into step pulses for the stepper driver
fn position_to_steps(target: Length) -> Steps {
    use messenger_mouse::encoder::*;
    let mm = target.get::<millimeter>();

    let out = (mm / KNIFE_AXIS_LEAD_MM)
        * KNIFE_AXIS_GEAR_RATIO
        * KNIFE_AXIS_MICROSTEPS_PER_STEP
        * KNIFE_AXIS_STEPS_PER_ROTATION;

    info!("target_mm_to_steps: target {}mm -> {}steps", mm, out);

    Steps(out as i32)
}

fn velocity_to_interval(speed: Velocity) -> Duration {
    use messenger_mouse::encoder::*;
    let mm_ps = speed.get::<millimeter_per_second>();

    let sps = (mm_ps / KNIFE_AXIS_LEAD_MM)
        * KNIFE_AXIS_GEAR_RATIO
        * KNIFE_AXIS_MICROSTEPS_PER_STEP
        * KNIFE_AXIS_STEPS_PER_ROTATION;
    let sps = sps.abs();

    info!("velocity_to_interval: target {}mm/s -> {}sps", mm_ps, sps);

    if sps < 0.1 {
        Duration::MAX
    } else {
        Duration::from_hz(sps as u64)
    }
}
