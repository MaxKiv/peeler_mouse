use embassy_stm32::Peri;
use embassy_stm32::peripherals::*;

pub struct CommsPeripherals {
    pub uart: Peri<'static, USART2>,
    pub tx: Peri<'static, PA2>,
    pub rx: Peri<'static, PA3>,
}
