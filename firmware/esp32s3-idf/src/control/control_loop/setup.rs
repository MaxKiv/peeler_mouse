use embassy_executor::Spawner;
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};

use crate::{
    comms::comms_task::SETPOINT_WATCH,
    control::control_loop::{body::control_loop, peripherals::ControlPeripherals},
};

pub fn run(spawner: &Spawner, p: ControlPeripherals) -> anyhow::Result<()> {
    let setpoint_receiver = SETPOINT_WATCH
        .receiver()
        .expect("Increase SETPOINT_WATCH N");

    // Setup LED driver
    log::info!("initialising Control Loop - LED driver");
    let led_timer = LedcTimerDriver::new(p.led_timer, &TimerConfig::default())?;
    let led_pwm = LedcDriver::new(p.led_ch, led_timer, p.led_pin)?;

    log::info!("Control Loop initialisation done, starting task");
    spawner.spawn(control_loop(setpoint_receiver, led_pwm))?;

    Ok(())
}
