pub mod appstate;
pub mod cmd;
pub mod esp;
pub mod hmi;
pub mod task;

/// Menu items / tabs the HMI can be in
#[derive(Default, Debug, Clone, PartialEq, defmt::Format)]
pub enum HmiState {
    #[default]
    NoSelection,
    MotorSelected,
}

/// Different motors used in the project
#[derive(Debug, Clone, Default, defmt::Format)]
pub enum SelectedMotor {
    #[default]
    Translation,
    Rotation,
    Cut,
}
