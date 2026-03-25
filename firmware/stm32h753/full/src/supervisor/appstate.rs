use messenger_mouse::motor::{
    KnifeManagementState, KnifeManager, MotorCommand, MotorVelocitySetpoint,
};
use uom::si::{f32::Velocity, velocity::millimeter_per_second};

use crate::{
    motor::controller::KNIFE_OPERATIONAL_SPEED_MM_PS,
    supervisor::{HmiState, MotorSetpoint, SelectedMotor},
};

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
    pub translation_setpoint: MotorCommand,
    pub rotation_setpoint: MotorCommand,
    pub knife_setpoint: MotorCommand,
    pub knife_manager: KnifeManager,
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

    pub fn set_current_motor_setpoint(&mut self, setpoint: MotorCommand) {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint = setpoint,
            SelectedMotor::Rotation => self.rotation_setpoint = setpoint,
            SelectedMotor::Cut => {
                self.knife_setpoint = KnifeManagementState::Manual(MotorCommand::MoveVelocity(
                    self.construct_knife_setpoint_from_appstate(),
                ));
            }
        }
    }

    pub fn get_current_motor_setpoint(&self) -> MotorCommand {
        match self.selected_motor {
            SelectedMotor::Translation => self.translation_setpoint.clone(),
            SelectedMotor::Rotation => self.rotation_setpoint.clone(),
            SelectedMotor::Cut => self.cut_setpoint.clone(),
        }
    }

    fn set_motor_enable(&mut self, enable: bool) {
        self.translation_setpoint.enabled = enable;
        self.rotation_setpoint.enabled = enable;
        self.knife_setpoint.enabled = enable;
        self.enable = enable;
    }

    pub fn set_knife_management(&mut self, manager: KnifeManager) {
        self.knife_manager = manager;
    }

    pub fn start_all(&mut self) {
        self.set_motor_enable(true);
    }

    pub fn stop_all(&mut self) {
        self.knife_manager = KnifeManager::Manual;
        self.set_motor_enable(false);
    }

    pub fn reset_all(&mut self) {
        self.enable = false;
        self.translation_setpoint = MotorCommand::default();
        self.rotation_setpoint = MotorCommand::default();
        self.knife_setpoint = MotorCommand::default();
        self.knife_manager = KnifeManager::Manual;
        self.hmi_state = HmiState::default();
    }
}
