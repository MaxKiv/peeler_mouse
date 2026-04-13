use defmt::warn;
use messenger_mouse::motor::{KnifeManager, MotorCommand, MotorVelocitySetpoint};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::{
    motor::controller::KNIFE_OPERATIONAL_SPEED_MM_PS,
    supervisor::{HmiState, SelectedMotor},
};

const MOTORS: [SelectedMotor; 3] = [
    SelectedMotor::Translation,
    SelectedMotor::Rotation,
    SelectedMotor::Cut,
];

/// Application state, managed by the supervisor
/// Tracks the currently selected motor
/// And for each motor the previo
#[derive(Debug, Clone, defmt::Format)]
pub struct Appstate {
    pub hmi_state: HmiState,
    pub selected_motor: SelectedMotor,
    pub translation_setpoint: MotorCommand,
    pub rotation_setpoint: MotorCommand,
    pub knife_setpoint: MotorCommand,
    pub knife_manager: KnifeManager,
    pub encoder_pos: i16,
    pub enable: bool,
}

impl Appstate {
    pub fn select_motor(&mut self, to_select: SelectedMotor) {
        self.selected_motor = to_select;
    }

    pub fn get_selected_motor(&self) -> SelectedMotor {
        self.selected_motor.clone()
    }

    pub fn selected_motor_idx(&mut self, idx: i16) {
        let len = MOTORS.len() as i16;
        let wrapped = idx.rem_euclid(len) as usize;
        self.selected_motor = MOTORS[wrapped].clone();
    }

    pub fn set_hmi_state(&mut self, hmi_state: HmiState) {
        self.hmi_state = hmi_state;
    }

    pub fn set_current_motor_setpoint(&mut self, setpoint: MotorCommand) {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint = setpoint,
            SelectedMotor::Rotation => self.rotation_setpoint = setpoint,
            SelectedMotor::Cut => self.knife_setpoint = setpoint,
        }
    }

    pub fn get_current_motor_setpoint(&self) -> MotorCommand {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint.clone(),
            SelectedMotor::Rotation => self.rotation_setpoint.clone(),
            SelectedMotor::Cut => self.knife_setpoint.clone(),
        }
    }

    pub fn set_knife_management(&mut self, manager: KnifeManager) {
        self.knife_manager = manager;
    }

    pub fn start_all(&mut self) {
        self.enable = true
    }

    pub fn stop_all(&mut self) {
        self.knife_manager = KnifeManager::Manual;
        self.enable = false;
    }

    pub fn reset_all(&mut self) {
        self.hmi_state = HmiState::default();
        self.translation_setpoint = MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe());
        self.rotation_setpoint = MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe());
        self.knife_setpoint = MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe());
        self.stop_all();
    }
}

impl Default for Appstate {
    fn default() -> Self {
        Self {
            translation_setpoint: MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe()),
            rotation_setpoint: MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe()),
            knife_setpoint: MotorCommand::MoveVelocity(MotorVelocitySetpoint::new_safe()),
            hmi_state: Default::default(),
            selected_motor: Default::default(),
            knife_manager: Default::default(),
            encoder_pos: Default::default(),
            enable: Default::default(),
        }
    }
}
