use crate::supervisor::{MotorSetpoint, SelectedMotor, task::HmiState};

const MOTORS: [SelectedMotor; 3] = [
    SelectedMotor::Translation,
    SelectedMotor::Rotation,
    SelectedMotor::Cut,
];

/// Application state, managed by the supervisor
/// Tracks the currently selected motor
/// And for each motor the previo
#[derive(Debug, Default, Clone)]
pub struct Appstate {
    pub hmi_state: HmiState,
    pub selected_motor: SelectedMotor,
    pub translation_setpoint: MotorSetpoint,
    pub rotation_setpoint: MotorSetpoint,
    pub cut_setpoint: MotorSetpoint,
    pub last_encoder_pos: i16,
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

    pub fn set_current_motor_setpoint(&mut self, setpoint: MotorSetpoint) {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint = setpoint,
            SelectedMotor::Rotation => self.rotation_setpoint = setpoint,
            SelectedMotor::Cut => self.cut_setpoint = setpoint,
        }
    }

    pub fn get_current_motor_setpoint(&self) -> MotorSetpoint {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint.clone(),
            SelectedMotor::Rotation => self.rotation_setpoint.clone(),
            SelectedMotor::Cut => self.cut_setpoint.clone(),
        }
    }

    fn set_motor_enable(&mut self, enable: bool) {
        self.translation_setpoint.enabled = enable;
        self.rotation_setpoint.enabled = enable;
        self.cut_setpoint.enabled = enable;
        self.enable = enable;
    }

    pub fn start_all(&mut self) {
        self.set_motor_enable(true);
    }

    pub fn stop_all(&mut self) {
        self.set_motor_enable(false);
    }

    pub fn reset_all(&mut self) {
        self.translation_setpoint = MotorSetpoint::safe();
        self.rotation_setpoint = MotorSetpoint::safe();
        self.cut_setpoint = MotorSetpoint::safe();
        self.enable = false;
    }
}
