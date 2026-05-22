use embassy_futures::select::{select, select4, Either, Either4};
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    watch::{Sender, Watch},
};
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
    limit_encoder_task::{StallEvent, StallMonitorCmd},
    low_level::{
        low_level_task::{position_to_steps, STEPPER_ACTION},
        state_machine::SPS,
        StepperAction,
    },
    HomeStatus, LimitSwitchState, PositionModeStatus, LIMIT_EVENT,
};

/// Public: callers write MotorAction here
pub static KNIFE_MOTOR_SETPOINT: Watch<CriticalSectionRawMutex, MotorAction, 2> = Watch::new();
/// Public: anyone can read homing status
pub static KNIFE_MOTOR_HOME_STATUS: Watch<CriticalSectionRawMutex, HomeStatus, 2> = Watch::new();
/// Public: upstream position mode status
pub static KNIFE_MOTOR_POS_STATUS: Watch<CriticalSectionRawMutex, PositionModeStatus, 2> =
    Watch::new();
/// Public: live step-position (0 = home)
pub static KNIFE_MOTOR_POS: Watch<CriticalSectionRawMutex, Steps, 2> = Watch::new();
pub static KNIFE_MOTOR_POS_RESET: Watch<CriticalSectionRawMutex, (), 1> = Watch::new();

/// Homing speed in mm/s
pub const HOMING_SPEED_MM_PS: f32 = 10.0;
pub const HOMING_DIRECTION: MotorDirection = MotorDirection::Forward;
pub const OPERATION_SPEED_MM_PS: f32 = 1.0;
pub const VISION_MAX_SPEED_MM_PS: f32 = 0.2;
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
    let pos_tx = KNIFE_MOTOR_POS_STATUS.sender();
    let mut limit_rx = LIMIT_EVENT.receiver().unwrap();
    let mut action_rx = KNIFE_MOTOR_SETPOINT.receiver().unwrap();
    let mut pos_rx = KNIFE_MOTOR_POS.receiver().unwrap();
    let pos_reset_tx = KNIFE_MOTOR_POS_RESET.sender();

    // Start stopped and lost
    cmd_tx.send(StepperAction::new_stopped());
    home_tx.send(HomeStatus::Lost);
    pos_tx.send(PositionModeStatus::Reached);

    let mut current_action = MotorAction::default();
    let mut current_cmd = StepperAction::new_stopped();
    let mut home_status = HomeStatus::Lost;
    let mut current_pos_steps = pos_rx.get().await;
    let mut target_pos_steps = Steps(0);
    let mut current_stall_state = StallEvent::default();

    let mut tx_encoder_stall_cmd =
        crate::actuation::stepper::limit_encoder_task::START_STALL_MONITOR.sender();
    let mut rx_encoder_stall_event = crate::actuation::stepper::limit_encoder_task::STALL_EVENT
        .receiver()
        .unwrap();

    // Initialize encoder stall detection
    tx_encoder_stall_cmd.send(StallMonitorCmd::Start);

    // Main motor control loop
    // Transform target speed into appropriate step period and relay to stepper driver
    // Check for limit switch activation
    loop {
        match select4(
            action_rx.changed(),              // New MotorAction
            limit_rx.changed(),               // Limit switch event
            pos_rx.changed(),                 // Position Update event
            rx_encoder_stall_event.changed(), // Encoder stall event
        )
        .await
        {
            // New motor action received from upstream
            Either4::First(new_action) => {
                debug!(
                    "MOTOR: RX motor action: {:?} {} {:?} OLD motor action",
                    new_action,
                    if new_action != current_action {
                        "!="
                    } else {
                        "=="
                    },
                    current_action
                );

                // We must be homed before accepting new motor actions
                if let HomeStatus::Homed { position: _ } = home_status {
                    // Is this a new action?
                    // This state tracking is ergonomic to do here due to the nature of UART comms
                    // Note: custom implementation of MotorVelocitySetpoint/MotorPositionSetpoint PartialEq
                    if new_action != current_action {
                        let new_cmd = match new_action.clone() {
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
                        set_action(
                            new_action,
                            &mut current_action,
                            new_cmd,
                            &mut current_cmd,
                            &cmd_tx,
                        );
                    }
                } else {
                    // Not homed: attempt to home
                    if current_action != MotorAction::Home {
                        // Currently lost && not homing -> Home motor
                        info!("MOTOR: Homing status == LOST -> homing motor");
                        set_action(
                            MotorAction::Home,
                            &mut current_action,
                            StepperAction::new_homing(),
                            &mut current_cmd,
                            &cmd_tx,
                        );
                    }
                }
            }

            // limit_switch event
            Either4::Second(level) => {
                if level == LimitSwitchState::Active {
                    // Limit switch pressed
                    if let MotorAction::Home = current_action {
                        // Press during Homing, expected
                        info!("MOTOR: HOMING limit switch active detected, moving back");
                    } else {
                        // Whoops; We are not doing a homing action but we did hit the limit switch
                        warn!("MOTOR: Limit switch hit but NOT in homing mode");
                    }

                    // Move back in reverse homing direction regardless of cause
                    homing_start(
                        &cmd_tx,
                        &home_tx,
                        &mut current_action,
                        &mut current_cmd,
                        &mut home_status,
                    );
                } else {
                    // Limit switch disengaged again, we are now Homed!
                    homing_finished(
                        &cmd_tx,
                        &home_tx,
                        &pos_reset_tx,
                        &mut current_action,
                        &mut current_cmd,
                        &mut home_status,
                    );

                    // Stay stopped -> upstream controller must send next MotorAction
                }
            }

            // Track position information from stepper task
            Either4::Third(new_pos) => {
                // info!("MOTOR: new position information: {:?}", new_pos);

                // Track current pos
                current_pos_steps = new_pos;

                // Are we doing a position mode action?
                if let MotorAction::MovePosition(_) = current_action {
                    // Did we reach position target?
                    if current_pos_steps == target_pos_steps {
                        // We did!
                        info!("MOTOR: Reached target position: {:?}", target_pos_steps);
                        on_position_reached(&mut current_action, &mut current_cmd, &cmd_tx, &pos_tx)
                    }
                }
            }

            // Encoder stall event
            Either4::Fourth(new_stall_event) => {
                let new_stall = new_stall_event == StallEvent::Stalled
                    && current_stall_state == StallEvent::Resolved;
                let stall_resolved = new_stall_event == StallEvent::Resolved
                    && current_stall_state == StallEvent::Stalled;

                match current_action {
                    MotorAction::Home => {
                        if new_stall {
                            // Stalled during homing, indicates Either:
                            //  - Limit switch disabled OR failed
                            //  - Homing in wrong direction
                            #[cfg(feature = "home_encoder_stall")]
                            homing_move_away_from_limit(
                                &cmd_tx,
                                &mut current_action,
                                &mut current_cmd,
                            );
                        } else if stall_resolved {
                            // Stall resolved during homing, we are now homed
                            #[cfg(feature = "home_encoder_stall")]
                            homing_finished(
                                &cmd_tx,
                                &home_tx,
                                &pos_reset_tx,
                                &mut current_action,
                                &mut current_cmd,
                                &mut home_status,
                            );
                        }
                    }
                    // MotorAction::MoveVelocity(ref mut sp) => {
                    //     if new_stall {
                    //         // Stalled during velocity movement, move in reverse direction
                    //         sp.dir.flip();
                    //
                    //         set_action(
                    //             MotorAction::MoveVelocity(sp.clone()),
                    //             &mut current_action,
                    //             StepperAction::new_stopped(),
                    //             &mut current_cmd,
                    //             &cmd_tx,
                    //         );
                    //     } else if stall_resolved {
                    //         // Previous stall is now resolved
                    //         // Coast motors
                    //         set_action(
                    //             MotorAction::Coast,
                    //             &mut current_action,
                    //             StepperAction::new_stopped(),
                    //             &mut current_cmd,
                    //             &cmd_tx,
                    //         );
                    //     }
                    // }
                    // MotorAction::MovePosition(_) => {
                    //     if new_stall {
                    //         // Stalled during position movement, stop moving
                    //         // TODO: Is this right?
                    //         set_action(
                    //             MotorAction::Coast,
                    //             &mut current_action,
                    //             StepperAction::new_stopped(),
                    //             &mut current_cmd,
                    //             &cmd_tx,
                    //         );
                    //     }
                    // }
                    // MotorAction::Coast | MotorAction::Hold => {
                    //     // Stalled during Coast or Hold
                    //     // Expected; Purposly ignored
                    // }
                    _ => {
                        // Intentionally empty
                    }
                }

                // Track last stall event
                current_stall_state = new_stall_event;
            }
        }
    }
}

fn on_position_reached(
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    pos_tx: &Sender<'static, CriticalSectionRawMutex, PositionModeStatus, 2>,
) {
    // Inform upstream
    pos_tx.send(PositionModeStatus::Reached);

    // Hold motors and stay at target position until further notice; Transition to halt command
    set_action(
        MotorAction::Hold,
        current_action,
        StepperAction::new_stopped(),
        current_cmd,
        cmd_tx,
    );
}

fn set_action(
    new_action: MotorAction,
    current_action: &mut MotorAction,
    new_cmd: StepperAction,
    current_cmd: &mut StepperAction,
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
) {
    info!("MOTOR: send NEW steppercmd -> low lvl {:?}", new_cmd);

    *current_action = new_action;
    *current_cmd = new_cmd;
    cmd_tx.send(current_cmd.clone());
}

fn homing_start(
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    home_tx: &Sender<'static, CriticalSectionRawMutex, HomeStatus, 2>,
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
    home_status: &mut HomeStatus,
) {
    *home_status = HomeStatus::Lost;
    home_tx.send(home_status.clone());

    homing_move_towards_limit(cmd_tx, current_action, current_cmd);
}

fn homing_finished(
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    home_tx: &Sender<'static, CriticalSectionRawMutex, HomeStatus, 2>,
    pos_reset_tx: &Sender<'static, CriticalSectionRawMutex, (), 1>,
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
    home_status: &mut HomeStatus,
) {
    // Stop stepping
    set_action(
        MotorAction::Hold,
        current_action,
        StepperAction::new_stopped(),
        current_cmd,
        cmd_tx,
    );
    cmd_tx.send(StepperAction::new_stopped());
    *current_action = MotorAction::Hold;

    // Record home
    *home_status = HomeStatus::Homed { position: 0 };
    home_tx.send(home_status.clone());
    info!("MOTOR: HOME CONFIRMED");

    // Ask low level stepper task to reset position to zero
    pos_reset_tx.send(());
}

/// Move towards limit switch
fn homing_move_towards_limit(
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
) {
    homing_move_in_direction(cmd_tx, current_action, current_cmd, HOMING_DIRECTION);
}

/// Move away from limit switch
fn homing_move_away_from_limit(
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
) {
    homing_move_in_direction(
        cmd_tx,
        current_action,
        current_cmd,
        HOMING_DIRECTION.get_opposite(),
    );
}

fn homing_move_in_direction(
    cmd_tx: &Sender<'static, CriticalSectionRawMutex, StepperAction, 1>,
    current_action: &mut MotorAction,
    current_cmd: &mut StepperAction,
    dir: MotorDirection,
) {
    // Move back untill switch disengages
    // Notice we move in reverse homing direction
    let home_vel = match HOMING_DIRECTION {
        MotorDirection::Forward => -HOMING_SPEED_MM_PS,
        MotorDirection::Reverse => HOMING_SPEED_MM_PS,
    };
    *current_action = MotorAction::Home;
    *current_cmd = StepperAction::MoveVelocity(MotorVelocitySetpoint {
        dir,
        speed: Velocity::new::<millimeter_per_second>(home_vel),
    });

    cmd_tx.send(current_cmd.clone());
}
