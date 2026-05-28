use messenger_mouse::motor::{MotorAction, MotorManager, MotorState};

use crate::supervisor::{HmiState, SelectedMotor};

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
    pub motor_state: MotorState,
    // pub translation_setpoint: MotorAction,
    // pub rotation_setpoint: MotorAction,
    // pub knife_setpoint: MotorAction,
    // pub knife_manager: MotorManager,
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
        const LEN: i16 = MOTORS.len() as i16;
        let wrapped = idx.rem_euclid(LEN) as usize;
        self.selected_motor = MOTORS[wrapped].clone();
    }

    pub fn set_hmi_state(&mut self, hmi_state: HmiState) {
        self.hmi_state = hmi_state;
    }

    pub fn set_current_motor_setpoint(&mut self, setpoint: MotorAction) {
        match self.selected_motor {
            SelectedMotor::Translation => self.motor_state.translation.setpoint = setpoint,
            SelectedMotor::Rotation => self.motor_state.rotation.setpoint = setpoint,
            SelectedMotor::Cut => self.motor_state.knife.setpoint = setpoint,
        }
    }

    pub fn get_current_motor_setpoint(&self) -> MotorAction {
        match self.selected_motor {
            SelectedMotor::Translation => self.motor_state.translation.setpoint.clone(),
            SelectedMotor::Rotation => self.motor_state.rotation.setpoint.clone(),
            SelectedMotor::Cut => self.motor_state.knife.setpoint.clone(),
        }
    }

    pub fn flip_management(&mut self, _manager: MotorManager) {
        self.motor_state.flip_management();
    }

    pub fn set_manager(&mut self, manager: MotorManager) {
        self.motor_state.set_manager(manager);
    }

    pub fn start_all(&mut self) {
        self.enable = true
    }

    pub fn stop_all(&mut self) {
        self.motor_state.set_manager(MotorManager::Manual);
        self.enable = false;
    }

    pub fn reset_all(&mut self) {
        self.hmi_state = HmiState::default();
        self.motor_state = Default::default();
        self.stop_all();
    }
}

impl Default for Appstate {
    fn default() -> Self {
        Self {
            hmi_state: Default::default(),
            selected_motor: Default::default(),
            motor_state: Default::default(),
            encoder_pos: Default::default(),
            enable: Default::default(),
        }
    }
}
