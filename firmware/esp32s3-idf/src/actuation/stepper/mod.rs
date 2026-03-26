pub mod motor_task;
pub mod peripherals;
pub mod setpoint;

#[derive(Clone, PartialEq)]
pub enum HomeStatus {
    Homed,
    Lost,
}
