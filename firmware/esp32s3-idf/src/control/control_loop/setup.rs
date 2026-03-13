use embassy_executor::Spawner;
use embassy_time::Delay;
use esp_idf_hal::{
    gpio::*,
    ledc::{
        config::TimerConfig, LedcDriver, LedcTimerDriver, CHANNEL0, CHANNEL1, CHANNEL2, TIMER0,
        TIMER1,
    },
};
use l9110::L9110;

use crate::{
    comms::comms_task::SETPOINT_WATCH,
    control::{actuation::l9110::manage_knife_motor, control_loop::body::control_loop},
};

pub struct ControlPeripherals {
    pub led_timer: TIMER0,
    pub led_ch: CHANNEL0,
    pub led_pin: Gpio48,
    pub motor_timer: TIMER1,
    pub motor_ch_a: CHANNEL1,
    pub motor_pin_a: Gpio42,
    pub motor_ch_b: CHANNEL2,
    pub motor_pin_b: Gpio41,
}

pub fn run(spawner: &Spawner, p: ControlPeripherals) -> anyhow::Result<()> {
    let setpoint_receiver = SETPOINT_WATCH
        .receiver()
        .expect("Increase SETPOINT_WATCH N");

    // Setup LED driver
    log::info!("initialising Control Loop - LED driver");
    let led_timer = LedcTimerDriver::new(p.led_timer, &TimerConfig::default())?;
    let led_pwm = LedcDriver::new(p.led_ch, led_timer, p.led_pin)?;

    let motor_timer = LedcTimerDriver::new(p.motor_timer, &TimerConfig::default())?;

    // Setup Knife Motor Driver
    log::info!("initialising Control Loop - Knife Motor Driver");
    let l9110 = L9110::try_new(
        "Knife",
        motor_timer,
        p.motor_ch_a,
        p.motor_pin_a,
        p.motor_ch_b,
        p.motor_pin_b,
        Delay,
    )?;

    log::info!("initialising Control Loop - Knife Motor Task");
    spawner.spawn(manage_knife_motor(l9110))?;

    log::info!("Control Loop initialisation done, starting task");
    spawner.spawn(control_loop(setpoint_receiver, led_pwm))?;

    Ok(())
}
