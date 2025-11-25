#![no_std]
#![no_main]

pub mod button;
pub mod linear_motor;
pub mod pot;
pub mod rotational_motor;

use crate::{
    button::ButtonPeripherals, linear_motor::LinearAxisMotorPeripherals, pot::PotPeripherals,
    rotational_motor::RotationalAxisMotorPeripherals,
};
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::{Config, adc::Adc};

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // SPI1 cksel defaults to pll1_q
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV8), // 100mhz
            divq: None,
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
        config.rcc.mux.adcsel = mux::Adcsel::PLL2_P;
    }
    let p = embassy_stm32::init(config);
    info!("Hello World!");

    let button_peri = ButtonPeripherals {
        b1: (p.PC13, p.EXTI13),
        b2: (p.PD7, p.EXTI7),
        b3: (p.PD6, p.EXTI6),
        b4: (p.PD5, p.EXTI5),
        b5: (p.PD4, p.EXTI4),
    };

    let linear_motor_peri = LinearAxisMotorPeripherals {
        step: p.PE3,
        dir: p.PF8,
    };

    let rotational_motor_peri = RotationalAxisMotorPeripherals {
        step: p.PF7,
        dir: p.PF9,
    };

    let pot_peri = PotPeripherals {
        pin: p.PA3,
        adc: p.ADC1,
        dma: *p.DMA1_CH1,
    };

    pot::setup(pot_peri, &spawner);
    button::setup(button_peri, &spawner);
    linear_motor::setup(linear_motor_peri, &spawner);
    rotational_motor::setup(rotational_motor_peri, &spawner);
}
