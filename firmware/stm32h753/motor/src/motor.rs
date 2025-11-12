use defmt::*;
use display_interface_i2c::I2CInterface;
use embassy_executor::Spawner;
use embassy_stm32::{
    Peri, bind_interrupts,
    gpio::{Level, Output, Speed},
    i2c::{self, I2c, Master},
    mode::Async,
    peripherals::*,
};
use embassy_time::{Duration, Ticker};
use embedded_graphics::{
    Drawable,
    image::{Image, ImageRawLE},
};
use embedded_graphics::{
    mono_font::{MonoTextStyleBuilder, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::Point,
    text::{Baseline, Text},
};
use ssd1309::{Builder, mode::GraphicsMode};
use tb6600::Tb6600;

const MOTOR_PERIOD: Duration = Duration::from_millis(100);

pub struct MotorPeripherals {
    pub motor_a_step: Peri<'static, PE3>,
    pub motor_a_dir: Peri<'static, PF8>,
}

pub fn setup(p: MotorPeripherals, spawner: &Spawner) {
    info!("Setting up motors");

    let step = Output::new(p.motor_a_step, Level::Low, Speed::Low);
    let dir = Output::new(p.motor_a_dir, Level::Low, Speed::Low);

    Tb6600::new(step, dir, embassy_time::Delay);

    spawner.spawn(manage_motors()).unwrap();
}

#[embassy_executor::task]
pub async fn manage_motors() {
    info!("Starting to manage motors");

    let mut ticker = Ticker::every(MOTOR_PERIOD);

    loop {
        ticker.next().await;
    }
}
