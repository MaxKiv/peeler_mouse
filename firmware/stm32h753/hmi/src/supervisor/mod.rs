pub mod cmd;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::Either6;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex as Cs, watch::Watch};
use uom::si::f32::Velocity;
use uom::si::velocity::millimeter_per_second;

use crate::hmi::button::BUTTON_WATCH_SIZE;
use crate::hmi::encoder::data::EncoderData;
use crate::motor::MotorDirection;

pub static APPSTATE_WATCH: Watch<Cs, Appstate, 2> = Watch::new();

/// Different motors used in the project
#[derive(Debug, Clone, Default, defmt::Format)]
pub enum SelectedMotor {
    #[default]
    Translation,
    Rotation,
    Cut,
}

/// Setpoint for a single motor
#[derive(Debug, Clone, Default)]
pub struct MotorSetpoint {
    pub enabled: bool,
    pub speed: Velocity,
    pub dir: MotorDirection,
}

impl MotorSetpoint {
    pub fn safe() -> Self {
        Self {
            enabled: false,
            speed: Velocity::new::<millimeter_per_second>(0.0),
            dir: MotorDirection::Forward,
        }
    }
}

/// Application state, managed by the supervisor
/// Tracks the currently selected motor
/// And for each motor the previo
#[derive(Debug, Default, Clone)]
pub struct Appstate {
    pub selected_motor: SelectedMotor,
    pub translation_setpoint: MotorSetpoint,
    pub rotation_setpoint: MotorSetpoint,
    pub cut_setpoint: MotorSetpoint,
    pub last_encoder_pos: i16,
}

impl Appstate {
    fn select_motor(&mut self, to_select: SelectedMotor) {
        self.selected_motor = to_select;
    }

    fn get_selected_motor(&self) -> SelectedMotor {
        self.selected_motor.clone()
    }

    fn set_current_motor_setpoint(&mut self, setpoint: MotorSetpoint) {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint = setpoint,
            SelectedMotor::Rotation => self.rotation_setpoint = setpoint,
            SelectedMotor::Cut => self.cut_setpoint = setpoint,
        }
    }

    fn get_current_motor_setpoint(&self) -> MotorSetpoint {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint.clone(),
            SelectedMotor::Rotation => self.rotation_setpoint.clone(),
            SelectedMotor::Cut => self.cut_setpoint.clone(),
        }
    }

    fn stop_all(&mut self) {
        self.translation_setpoint = MotorSetpoint::safe();
        self.rotation_setpoint = MotorSetpoint::safe();
        self.cut_setpoint = MotorSetpoint::safe();
    }
}

pub static ROTATION_SELECTED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static TRANSLATION_SELECTED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static CUT_SELECTED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static STOP_ALL_SELECTED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static ENCODER_PRESSED: Watch<Cs, bool, { BUTTON_WATCH_SIZE }> = Watch::new();
pub static ENCODER_DATA: Watch<Cs, EncoderData, 2> = Watch::new();

pub const MOTOR_SPEED_STEPS: usize = 10;
pub const DEFAULT_ROTATION_VELOCITY_MM_PS: f32 = 0.0;
pub const DEFAULT_TRANSLATION_VELOCITY_MM_PS: f32 = 0.0;
pub const DEFAULT_CUT_VELOCITY_MM_PS: f32 = 0.0;
pub const MAX_ROTATION_VELOCITY_MM_PS: f32 = 0.1;
pub const MAX_TRANSLATION_VELOCITY_MM_PS: f32 = 1.0;
pub const MAX_CUT_VELOCITY_MM_PS: f32 = 1.4;

pub fn setup(spawner: &Spawner) {
    info!("Setting up Supervisor");

    spawner.spawn(supervise()).unwrap();
}

/// Main supervisor loop, manages appstate
#[embassy_executor::task]
async fn supervise() {
    let mut rotation_selected_rx = ROTATION_SELECTED
        .receiver()
        .expect("Increase rotation_selected N");
    let mut translation_selected_tx = TRANSLATION_SELECTED
        .receiver()
        .expect("Increase translation_selected N");
    let mut cut_selected_rx = CUT_SELECTED.receiver().expect("Increase cut_selected N");
    let mut stop_all_selected_rx = STOP_ALL_SELECTED
        .receiver()
        .expect("Increase stop_all_selected N");
    let mut encoder_pressed_rx = ENCODER_PRESSED
        .receiver()
        .expect("Increase encoder_pressed N");

    let mut encoder_data_rx = ENCODER_DATA.receiver().expect("Increase encoder_data N");

    let appstate_tx = APPSTATE_WATCH.sender();

    // Initialise appstate
    let mut app_state = Appstate::default();

    loop {
        // Wait for a HMI input that we need to process
        match embassy_futures::select::select6(
            rotation_selected_rx.changed(),
            translation_selected_tx.changed(),
            cut_selected_rx.changed(),
            stop_all_selected_rx.changed(),
            encoder_pressed_rx.changed(),
            encoder_data_rx.changed(),
        )
        .await
        {
            // Rotation Selected Pressed -> Select Rotation motor
            Either6::First(_) => {
                debug!("Supervisor - Select Rotation Motor");
                app_state.select_motor(SelectedMotor::Rotation);
            }
            // Translation Selected Pressed -> Select Translation motor
            Either6::Second(_) => {
                debug!("Supervisor - Select Translation Motor");
                app_state.select_motor(SelectedMotor::Translation);
            }
            // Cut Selected Pressed -> Select cut motor
            Either6::Third(_) => {
                debug!("Supervisor - Select Cut Motor");
                app_state.select_motor(SelectedMotor::Cut);
            }
            // Stop All Selected Pressed -> Stop all motors
            Either6::Fourth(_) => {
                debug!("Supervisor - STOP ALL");
                app_state.stop_all();
            }
            // Encoder button pressed -> Stop current motor + Reverse direction
            Either6::Fifth(_) => {
                let mut setpoint = app_state.get_current_motor_setpoint();
                setpoint.speed = Velocity::new::<millimeter_per_second>(0.0);
                setpoint.dir.reverse();

                debug!(
                    "Supervisor - Reversing {:?}",
                    app_state.get_selected_motor()
                );

                app_state.set_current_motor_setpoint(setpoint);
            }
            // Encoder count change -> Change current motor speed
            Either6::Sixth(encoder_data) => {
                // Calculate new speed
                let selected_motor = app_state.get_selected_motor();
                let mut setpoint = app_state.get_current_motor_setpoint();
                let encoder_delta = encoder_data.pos.saturating_sub(app_state.last_encoder_pos);

                setpoint.speed =
                    calculate_new_motor_speed(selected_motor, setpoint.speed, encoder_delta);

                // Log change in speed
                let speed = app_state
                    .get_current_motor_setpoint()
                    .speed
                    .get::<millimeter_per_second>();
                debug!(
                    "Supervisor - Setting speed of {:?} to {}mm/ps",
                    app_state.selected_motor, speed
                );

                app_state.last_encoder_pos = encoder_data.pos;
                app_state.set_current_motor_setpoint(setpoint);
            }
        }

        // Application state has changed, update downstream actuators & Display
        appstate_tx.send(app_state.clone());
    }
}

/// Calculates the new motor speed after a new encoder delta is received
/// This depends on the previous and maximum motor speed.
fn calculate_new_motor_speed(
    selected_motor: SelectedMotor,
    current_speed: Velocity,
    encoder_delta: i16,
) -> Velocity {
    let max_velocity = match selected_motor {
        SelectedMotor::Translation => MAX_TRANSLATION_VELOCITY_MM_PS,
        SelectedMotor::Rotation => MAX_ROTATION_VELOCITY_MM_PS,
        SelectedMotor::Cut => MAX_CUT_VELOCITY_MM_PS,
    };

    let step = max_velocity / MOTOR_SPEED_STEPS as f32;
    let current_speed = current_speed.get::<millimeter_per_second>();
    let new_speed = (current_speed + (step * encoder_delta as f32)).clamp(0.0, max_velocity);

    Velocity::new::<millimeter_per_second>(new_speed)
}
