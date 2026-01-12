#![no_std]
#![no_main]

pub mod clocks;
pub mod hmi;
pub mod motor;
pub mod supervisor;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::{Config, bind_interrupts, i2c};

use crate::hmi::lcd::setup::LcdPeripherals;
use crate::hmi::{
    button::{ButtonMode, ButtonPeripherals},
    encoder::QuadratureEncoderPeripherals,
};

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
});

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let config = clocks::setup_clocks(config);
    let p = embassy_stm32::init(config);
    info!("Clocks configured - Hello World!");

    // ---- HMI Peripheral declarations -----

    // I2C LCD screen
    let lcd_peri = LcdPeripherals {
        sda: p.PF0,
        scl: p.PF1,
        i2c: p.I2C2,
        tx_dma: p.DMA1_CH4,
        rx_dma: p.DMA1_CH5,
    };

    // LED buttons
    let green_button = ButtonPeripherals {
        pin: p.PD7,
        ch: p.EXTI7,
    };
    let blue_button = ButtonPeripherals {
        pin: p.PD6,
        ch: p.EXTI6,
    };
    let purple_button = ButtonPeripherals {
        pin: p.PD5,
        ch: p.EXTI5,
    };
    let gray_button = ButtonPeripherals {
        pin: p.PD4,
        ch: p.EXTI4,
    };

    // Encoder
    let encoder_button = ButtonPeripherals {
        pin: p.PD3,
        ch: p.EXTI3,
    };
    let encoder_peri = QuadratureEncoderPeripherals {
        ch1: p.PA6,
        ch2: p.PB5,
        timer: p.TIM3,
    };

    // ---- HMI Task Construction -----

    hmi::button::DebouncedButton::run(
        green_button,
        &supervisor::KNIFE_ENABLED,
        "green",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        blue_button,
        &supervisor::KNIFE_ENABLED,
        "blue",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        purple_button,
        &supervisor::KNIFE_ENABLED,
        "purple",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        gray_button,
        &supervisor::KNIFE_ENABLED,
        "gray",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        encoder_button,
        &supervisor::KNIFE_ENABLED,
        "encoder",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::encoder::QuadratureEncoder::run(encoder_peri, &spawner);

    hmi::lcd::setup::setup(lcd_peri, &spawner);

    // ---- Supervisor (routes HMI input to HMI & actuator output) -----
    supervisor::setup(&spawner);
}
