use embassy_executor::Spawner;
use embassy_time::Delay;
use esp_idf_hal::{
    gpio::*,
    ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
};
use l9110::L9110;

use crate::{comms::comms_task::SETPOINT_WATCH, control::control_loop::body::control_loop};

pub fn run(spawner: &Spawner, p: ControlPeripherals) -> anyhow::Result<()> {
    let setpoint_receiver = SETPOINT_WATCH
        .receiver()
        .expect("Increase SETPOINT_WATCH N");

    // Setup LED driver
    log::info!("initialising Control Loop - LED driver");
    let led_timer = LedcTimerDriver::new(p.led_timer, &TimerConfig::default())?;
    let led_pwm = LedcDriver::new(p.led_ch, led_timer, p.led_pin)?;

    // Setup motor controller
    let mut limit_switch = PinDriver::input(p.limit_switch)?;
    limit_switch.set_pull(Pull::Up);

    log::info!("initialising Control Loop - Knife Motor Task");
    spawner.spawn(manage_knife_motor(l9110))?;

    log::info!("Control Loop initialisation done, starting task");
    spawner.spawn(control_loop(setpoint_receiver, led_pwm))?;

    Ok(())
}
