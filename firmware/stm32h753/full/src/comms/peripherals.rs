use embassy_stm32::Peri;
use embassy_stm32::peripherals::*;

pub struct CommsPeripherals {
    pub uart: Peri<'static, USART3>,
    pub tx: Peri<'static, PB10>,
    pub rx: Peri<'static, PB11>,
}
