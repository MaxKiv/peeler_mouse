use messenger_mouse::{
    FRAME_SIZE,
    control_params::ControlParams,
    motor::{ControlMode, MotorAction, MotorSetpoints},
};

use crate::supervisor::appstate::{MOTORS, PARAMS};

pub mod appstate;
pub mod cmd;
pub mod hmi;
pub mod task;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum OverlayMode {
    Default,
    TransitionLine,
    TransitionError,
    CameraFPS,
    TearingDetection,
}

impl OverlayMode {
    pub fn next(&self) -> Self {
        match self {
            OverlayMode::Default => OverlayMode::TransitionLine,
            OverlayMode::TransitionLine => OverlayMode::TransitionError,
            OverlayMode::TransitionError => OverlayMode::CameraFPS,
            OverlayMode::CameraFPS => OverlayMode::TearingDetection,
            OverlayMode::TearingDetection => OverlayMode::Default,
        }
    }
}

#[derive(Debug, Clone, PartialEq, defmt::Format)]
/// Menu items / tabs the HMI can be in
pub struct HmiState {
    pub motor_selection_state: SelectionState,
    pub parameter_selection_state: SelectionState,
    pub selected_motor: &'static MotorType,
    pub selected_parameter: &'static ControlParameterType,
    pub control_mode: ControlMode,
    pub overlay_mode: OverlayMode,
    pub motor_setpoints: MotorSetpoints,
    pub parameter_setpoints: ControlParams,
    pub encoder_pos: i16,
    pub enable: bool,
}

impl HmiState {
    pub fn get_selected_motor(&self) -> &MotorType {
        &self.selected_motor
    }

    pub fn get_selected_parameter(&self) -> &ControlParameterType {
        &self.selected_parameter
    }

    pub fn set_control_mode(&mut self, control_mode: ControlMode) {
        self.control_mode = control_mode;
    }

    pub fn set_param_zero_crossing(&mut self, new: u32) {
        const MAX_ZERO_CROSSING: u32 = FRAME_SIZE.get_dimensions().1 as u32;
        const MIN_ZERO_CROSSING: u32 = 0;

        self.parameter_setpoints.zero_line_px = new.clamp(MIN_ZERO_CROSSING, MAX_ZERO_CROSSING);
    }

    pub fn set_param_gain(&mut self, new: f32) {
        const MAX_GAIN: f32 = 100.0; // Max control effort = Max motor speed
        const MIN_GAIN: f32 = 0.0; // Controller disabled

        self.parameter_setpoints.gain = new.clamp(MIN_GAIN, MAX_GAIN);
    }

    pub fn set_param_lead(&mut self, new: f32) {
        const MIN_LEAD: f32 = 0.0; // no linear movement, just rotation

        self.parameter_setpoints.lead =
            new.clamp(MIN_LEAD, messenger_mouse::control_params::LEAD_MAX);
    }

    pub fn select_parameter_from_idx(&mut self, idx: i16) {
        const LEN: i16 = PARAMS.len() as i16;
        let wrapped = idx.rem_euclid(LEN) as usize;
        self.selected_parameter = &PARAMS[wrapped];
    }

    pub fn get_selected_parameter_idx(&self) -> Option<usize> {
        PARAMS
            .iter()
            .position(|m| m == self.get_selected_parameter())
    }

    pub fn select_motor_from_idx(&mut self, idx: i16) {
        const LEN: i16 = MOTORS.len() as i16;
        let wrapped = idx.rem_euclid(LEN) as usize;
        self.selected_motor = &MOTORS[wrapped];
    }

    pub fn get_selected_motor_idx(&self) -> Option<usize> {
        MOTORS.iter().position(|m| m == self.get_selected_motor())
    }

    pub fn select_next_motor(&mut self) {
        self.motor_selection_state.next();
    }

    pub fn select_next_parameter(&mut self) {
        self.parameter_selection_state.next();
    }

    pub fn get_current_motor_action(&self) -> MotorAction {
        match self.selected_motor {
            MotorType::Translation => self.motor_setpoints.translation.clone(),
            MotorType::Rotation => self.motor_setpoints.rotation.clone(),
            MotorType::Cut => self.motor_setpoints.knife.clone(),
        }
    }

    pub fn set_current_motor_action(&mut self, setpoint: MotorAction) {
        match self.selected_motor {
            MotorType::Translation => self.motor_setpoints.translation = setpoint,
            MotorType::Rotation => self.motor_setpoints.rotation = setpoint,
            MotorType::Cut => self.motor_setpoints.knife = setpoint,
        }
    }

    pub fn flip_control_mode(&mut self) {
        self.control_mode.flip();
    }

    pub fn start_all(&mut self) {
        self.enable = true
    }

    pub fn stop_all(&mut self) {
        self.overlay_mode = OverlayMode::Default;
        self.enable = false;
    }

    pub fn reset_all(&mut self) {
        self.motor_selection_state = SelectionState::default();
        self.motor_setpoints = MotorSetpoints::reset();
        self.parameter_selection_state = SelectionState::default();
        self.parameter_setpoints = ControlParams::reset();
        self.overlay_mode = OverlayMode::Default;
        self.control_mode = ControlMode::Manual;
        self.stop_all();
    }
}

impl Default for HmiState {
    fn default() -> Self {
        HmiState {
            enable: false,
            motor_selection_state: SelectionState::NoSelection,
            parameter_selection_state: SelectionState::NoSelection,
            control_mode: ControlMode::Manual,
            overlay_mode: OverlayMode::Default,
            encoder_pos: 0i16,
            selected_motor: &MOTORS[0],
            selected_parameter: &PARAMS[0],
            parameter_setpoints: ControlParams::reset(),
            motor_setpoints: MotorSetpoints::reset(),
        }
    }
}

/// HMI selection menu state
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum SelectionState {
    #[default]
    NoSelection,
    Selected,
}

impl SelectionState {
    pub fn next(&mut self) {
        *self = match self {
            SelectionState::NoSelection => SelectionState::Selected,
            SelectionState::Selected => SelectionState::NoSelection,
        };
    }
}

/// Different motors used in the project
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum MotorType {
    #[default]
    Translation,
    Rotation,
    Cut,
}

/// Different motors used in the project
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum ControlParameterType {
    #[default]
    ZeroLine,
    Gain,
    Lead,
}
