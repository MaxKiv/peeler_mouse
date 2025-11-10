#![no_std]
#![no_main]

pub mod button;

use crate::button::Button;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;

use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let button_1 = Button {
        input: ExtiInput::new(p.PE2, p.EXTI2, Pull::Down),
        name: "1",
    };
    let button_2 = Button {
        input: ExtiInput::new(p.PE4, p.EXTI4, Pull::Down),
        name: "2",
    };
    let button_3 = Button {
        input: ExtiInput::new(p.PE5, p.EXTI5, Pull::Down),
        name: "3",
    };
    let button_4 = Button {
        input: ExtiInput::new(p.PE6, p.EXTI6, Pull::Down),
        name: "4",
    };
    let button_5 = Button {
        input: ExtiInput::new(p.PE3, p.EXTI3, Pull::Down),
        name: "5",
    };

    p = button::setup(p, &spawner);
}
