pub mod command;
pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

#[derive(Clone)]
pub enum HomeStatus {
    Homed,
    Lost,
}
