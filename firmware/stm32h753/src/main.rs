#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::rcc::AHBPrescaler;
use embassy_stm32::rcc::APBPrescaler;
use embassy_stm32::rcc::Hsi48Config;
use embassy_stm32::rcc::LsConfig;
use embassy_stm32::rcc::PllMul;
use embassy_stm32::rcc::PllPreDiv;
use embassy_stm32::rcc::PllSource;
use embassy_stm32::rcc::RtcClockSource;
use embassy_stm32::rcc::Sysclk;
use embassy_stm32::rcc::mux::Adcsel;
use embassy_stm32::rcc::mux::Rtcsel;
use embassy_stm32::{
    Config,
    exti::ExtiInput,
    gpio::{Level, Output, Pull, Speed},
};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting...");

    let config = configure_rcc();
    let p = embassy_stm32::init(config);
    info!("Default configuration applied, Hello world!");

    let mut led = Output::new(p.PE1, Level::High, Speed::Low);
    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down);

    info!("Press the USER button...");

    loop {
        button.wait_for_rising_edge().await;
        info!("Pressed!");
        led.set_high();

        button.wait_for_falling_edge().await;
        led.set_low();
        info!("Released!");
    }
}

// Configure reset and clock control
fn configure_rcc() -> Config {
    let mut config = Config::default();
    config
}
