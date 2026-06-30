use messenger_mouse::motor::{ControlMode, MotorAction, MotorSetpoints};

use crate::supervisor::appstate::MOTORS;

pub mod appstate;
pub mod cmd;
pub mod hmi;
pub mod task;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum OverlayMode {
    Default,
    TransitionLine,
    CameraFPS,
    TearingDetection,
}

impl OverlayMode {
    pub fn next(&self) -> Self {
        match self {
            OverlayMode::Default => OverlayMode::TransitionLine,
            OverlayMode::TransitionLine => OverlayMode::CameraFPS,
            OverlayMode::CameraFPS => OverlayMode::TearingDetection,
            OverlayMode::TearingDetection => OverlayMode::Default,
        }
    }
}

#[derive(Debug, Clone, PartialEq, defmt::Format)]
/// Menu items / tabs the HMI can be in
pub struct HmiState {
    pub motor_selection_tab_state: MotorSelectionTab,
    pub selected_motor: &'static MotorTypes,
    pub control_mode: ControlMode,
    pub overlay_mode: OverlayMode,
    pub motor_setpoints: MotorSetpoints,
    pub encoder_pos: i16,
    pub enable: bool,
}

impl HmiState {
    pub fn get_selected_motor(&self) -> &MotorTypes {
        &self.selected_motor
    }

    pub fn set_control_mode(&mut self, control_mode: ControlMode) {
        self.control_mode = control_mode;
    }

    pub fn select_new_motor_idx(&mut self, idx: i16) {
        const LEN: i16 = MOTORS.len() as i16;
        let wrapped = idx.rem_euclid(LEN) as usize;
        self.selected_motor = &MOTORS[wrapped];
    }

    pub fn get_selected_motor_idx(&self) -> Option<usize> {
        MOTORS.iter().position(|m| m == self.get_selected_motor())
    }

    pub fn next_motor_selection_tab(&mut self) {
        self.motor_selection_tab_state.next();
    }

    pub fn get_current_motor_action(&self) -> MotorAction {
        match self.selected_motor {
            MotorTypes::Translation => self.motor_setpoints.translation.clone(),
            MotorTypes::Rotation => self.motor_setpoints.rotation.clone(),
            MotorTypes::Cut => self.motor_setpoints.knife.clone(),
        }
    }

    pub fn set_current_motor_action(&mut self, setpoint: MotorAction) {
        match self.selected_motor {
            MotorTypes::Translation => self.motor_setpoints.translation = setpoint,
            MotorTypes::Rotation => self.motor_setpoints.rotation = setpoint,
            MotorTypes::Cut => self.motor_setpoints.knife = setpoint,
        }
    }

    pub fn flip_control_mode(&mut self) {
        self.control_mode.flip();
    }

    pub fn start_all(&mut self) {
        self.enable = true
    }

    pub fn stop_all(&mut self) {
        self.control_mode = ControlMode::Manual;
        self.overlay_mode = OverlayMode::Default;
        self.enable = false;
    }

    pub fn reset_all(&mut self) {
        self.motor_selection_tab_state = MotorSelectionTab::default();
        self.motor_setpoints = MotorSetpoints::reset();
        self.overlay_mode = OverlayMode::Default;
        self.stop_all();
    }
}

impl Default for HmiState {
    fn default() -> Self {
        HmiState {
            selected_motor: &MOTORS[0],
            enable: false,
            motor_selection_tab_state: MotorSelectionTab::NoSelection,
            control_mode: ControlMode::Manual,
            overlay_mode: OverlayMode::Default,
            motor_setpoints: MotorSetpoints::reset(),
            encoder_pos: 0i16,
        }
    }
}

/// HMI motor selection menu state
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum MotorSelectionTab {
    #[default]
    NoSelection,
    MotorSelected,
}

impl MotorSelectionTab {
    pub fn next(&mut self) {
        *self = match self {
            MotorSelectionTab::NoSelection => MotorSelectionTab::MotorSelected,
            MotorSelectionTab::MotorSelected => MotorSelectionTab::NoSelection,
        };
    }
}

/// Different motors used in the project
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum MotorTypes {
    #[default]
    Translation,
    Rotation,
    Cut,
}
