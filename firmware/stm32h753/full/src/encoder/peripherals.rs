use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::*;

pub struct EncoderPeripherals {
    pub spi: SpiDevice<'static>,
    // pub uart: Peri<'static, USART3>,
    // pub tx: Peri<'static, PB10>,
    // pub rx: Peri<'static, PB11>,
}
