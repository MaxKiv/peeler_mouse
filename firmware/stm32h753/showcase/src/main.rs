#![no_std]
#![no_main]

pub mod button;
pub mod linear_motor;
pub mod rotational_motor;

use crate::{
    button::ButtonPeripherals, linear_motor::LinearAxisMotorPeripherals,
    rotational_motor::RotationalAxisMotorPeripherals,
};
use defmt::*;
use embassy_executor::Spawner;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
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

    button::setup(button_peri, &spawner);
    linear_motor::setup(linear_motor_peri, &spawner);
    rotational_motor::setup(rotational_motor_peri, &spawner);
}
