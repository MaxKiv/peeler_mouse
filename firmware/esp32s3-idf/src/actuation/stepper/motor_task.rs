use embassy_executor::Spawner;
use embassy_futures::select::{select, select3, Either, Either3};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Delay, Duration, Instant, Timer};
use log::*;
use messenger_mouse::motor::{
    MotorAction, MotorDirection, MotorPositionSetpoint, MotorVelocitySetpoint,
    StepperPositionSetpoint, Steps, POSITION_MODE_VELOCITY_MM_PS,
};
use uom::si::{
    f32::{Length, Velocity},
    length::millimeter,
    velocity::millimeter_per_second,
};

use crate::actuation::stepper::{
    limit_switch_task::{LimitSwitchState, LIMIT_EVENT},
    low_level::{
        low_level_task::{position_to_steps, STEPPER_ACTION},
        state_machine::SPS,
        StepperAction,
    },
    HomeStatus, PositionModeStatus,
};

/// Public: callers write MotorAction here
pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorAction, 2> = Watch::new();
/// Public: anyone can read homing status
pub static KNIFE_MOTOR_HOME_STATUS: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
/// Public: upstream position mode status
pub static KNFIE_MOTOR_POS_STATUS: Watch<CriticalSectionRawMutex, PositionModeStatus, 2> =
    Watch::new();
/// Public: live step-position (0 = home)
pub static KNIFE_MOTOR_POS: Watch<CriticalSectionRawMutex, Steps, 2> = Watch::new();
pub static KNIFE_MOTOR_POS_RESET: Watch<CriticalSectionRawMutex, (), 1> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 10.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
/// TODO: Motor speed for 1 revolution per second
pub const SPEED_REV_PS: f32 = 1.0;

/// const MINIMUM_SPS: Duration = Duration::from_hz(100); // SPS
pub const MINIMUM_SPS: SPS = SPS(100);
/// pub const MINIMUM_SPS: Duration = Duration::from_hz(100); // SPS
pub const MAXIMUM_SPS: SPS = SPS(10_000); // SPS
/// Minimum SPS to switch direction
pub const MINIMUM_TRANSITION_SPS: SPS = SPS(500);

// Control task
#[embassy_executor::task]
pub async fn control_knife_motor() {
    log::info!("MOTOR: initialising HIGH level control task");

    let cmd_tx = STEPPER_ACTION.sender();
    let home_tx = KNIFE_MOTOR_HOME_STATUS.sender();
    let pos_tx = KNFIE_MOTOR_POS_STATUS.sender();
    let mut limit_rx = LIMIT_EVENT.receiver().unwrap();
    let mut action_rx = KNIFE_MOTOR_SETPOINT.receiver().unwrap();
    let mut pos_rx = KNIFE_MOTOR_POS.receiver().unwrap();
    let mut pos_reset_tx = KNIFE_MOTOR_POS_RESET.sender();

    // Start stopped and lost
    cmd_tx.send(StepperAction::new_stopped());
    home_tx.send(HomeStatus::Lost);
    pos_tx.send(PositionModeStatus::Reached);

    let mut current_action = MotorAction::default();
    let mut current_cmd = StepperAction::new_stopped();
    let mut home_status = HomeStatus::Lost;
    let mut current_pos_steps = pos_rx.get().await;
    let mut target_pos_steps = Steps(0);

    // Main motor control loop
    // Transform target speed into appropriate step period and relay to stepper driver
    // Check for limit switch activation
    loop {
        match select3(action_rx.changed(), limit_rx.changed(), pos_rx.changed()).await {
            // New motor command received
            Either3::First(action) => {
                // Note: Only act when a new action is requested
                // This state tracking is ergonomic to do here due to the nature of UART comms

                debug!(
                    "MOTOR: RX motor action: {:?} {} {:?} OLD motor action",
                    action,
                    if action != current_action { "!=" } else { "==" },
                    current_action
                );

                // Are we homed?
                if let HomeStatus::Homed { position: _ } = home_status {
                    // Is this a new action?
                    // Note: custom implementation of MotorVelocitySetpoint/MotorPositionSetpoint PartialEq
                    if action != current_action {
                        let next_cmd = match action.clone() {
                            MotorAction::Hold => StepperAction::new_stopped(),

                            MotorAction::MoveVelocity(sp) => {
                                debug!("MOTOR HI: new velocity setpoint: {:?}", sp);
                                StepperAction::MoveVelocity(sp)
                            }

                            MotorAction::Home => {
                                // Home command; Reset home status
                                home_status = HomeStatus::Lost;
                                home_tx.send(home_status.clone());

                                StepperAction::new_homing()
                            }

                            MotorAction::MovePosition(MotorPositionSetpoint { target, speed }) => {
                                let target_pos_steps = position_to_steps(target);

                                // Inform upstream we are starting a new position mode action
                                pos_tx.send(PositionModeStatus::InProgress);

                                StepperAction::MovePosition(StepperPositionSetpoint {
                                    target: target_pos_steps,
                                    speed,
                                })
                            }

                            MotorAction::Coast => StepperAction::Coast,
                        };

                        // Send new StepperCommand & Bookkeeping
                        info!("MOTOR: send NEW steppercmd -> low lvl {:?}", next_cmd);
                        current_cmd = next_cmd;
                        cmd_tx.send(current_cmd.clone());
                        current_action = action;
                    }
                } else {
                    if current_action != MotorAction::Home {
                        // Currently lost -> Home motor
                        info!("MOTOR: Homing status == LOST -> homing motor");
                        current_action = MotorAction::Home;
                        current_cmd = StepperAction::new_homing();
                        cmd_tx.send(current_cmd.clone());
                    }
                }
            }

            // limit_switch event
            Either3::Second(level) => {
                if level == LimitSwitchState::Active {
                    error!("Limit switch ACTIVE");

                    home_status = HomeStatus::Lost;
                    home_tx.send(home_status);

                    match current_action {
                        MotorAction::Home => {
                            // All good
                            info!("MOTOR: HOMING limit switch active detected, moving back");
                        }
                        _ => {
                            // Woops; We are not doing a homing action but hit the limit switch
                            warn!("MOTOR: Limit switch hit but NOT in homing mode");
                        }
                    }

                    // Move back untill switch disengages
                    // Notice we move in reverse homing direction
                    let home_vel = match HOMING_DIRECTION {
                        MotorDirection::Forward => -HOMING_SPEED_MM_PS,
                        MotorDirection::Reverse => HOMING_SPEED_MM_PS,
                    };
                    current_action = MotorAction::Home;
                    current_cmd = StepperAction::MoveVelocity(MotorVelocitySetpoint {
                        dir: HOMING_DIRECTION.get_opposite(),
                        speed: Velocity::new::<millimeter_per_second>(home_vel),
                    });

                    error!("Limit switch hit outside of HOMING mode, sending HOMING cmd");

                    cmd_tx.send(current_cmd.clone());
                } else {
                    // Limit switch disengaged, we are now Homed
                    // Stop stepping
                    cmd_tx.send(StepperAction::new_stopped());
                    current_action = MotorAction::Hold;

                    // Record home
                    info!("MOTOR: HOME CONFIRMED");
                    home_status = HomeStatus::Homed { position: 0 };
                    home_tx.send(home_status.clone());

                    // Ask low level stepper task to reset position to zero
                    pos_reset_tx.send(());

                    // Stay stopped -> upstream controller must send next MotorAction
                }
            }

            // Track position information from stepper task
            // Update
            Either3::Third(new_pos) => {
                // info!("MOTOR: new position information: {:?}", new_pos);
                // Track current pos
                current_pos_steps = new_pos;

                // Are we doing a position mode action?
                if let MotorAction::MovePosition(_) = current_action {
                    // Did we reach position target?
                    if current_pos_steps == target_pos_steps {
                        info!("MOTOR: Reached target position: {:?}", target_pos_steps);

                        // Inform upstream
                        pos_tx.send(PositionModeStatus::Reached);

                        // Position target reached -> stop motors
                        cmd_tx.send(StepperAction::new_stopped());

                        // Stay at target position until further notice; Transition to halt command
                        current_action = MotorAction::Hold;
                        cmd_tx.send(StepperAction::new_stopped());
                    }
                }
            }
        }
    }
}
