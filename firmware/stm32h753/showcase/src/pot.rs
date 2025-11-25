use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::adc::{AdcChannel, SampleTime};
use embassy_stm32::{Peri, adc::Adc, peripherals::*};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Timer};

// #[unsafe(link_section = ".ram_d3")]
// static mut DMA_BUF: [u16; 2] = [0; 2];

pub static WATCH_POT: Watch<CriticalSectionRawMutex, u16, 2> = Watch::new();

pub struct PotPeripherals {
    pub pin: Peri<'static, PA3>,
    pub adc: Peri<'static, ADC1>,
    pub dma: DMA1_CH1,
}

pub fn setup(p: PotPeripherals, spawner: &Spawner) {
    let adc = Adc::new(p.adc);

    let pot = PotAdc {
        pin: p.pin,
        dma: p.dma,
        adc,
    };

    spawner.spawn(manage_pot(pot)).unwrap();
}

pub struct PotAdc {
    pub pin: Peri<'static, PA3>,
    pub dma: DMA1_CH1,
    pub adc: Adc<'static, ADC1>,
}

#[embassy_executor::task]
pub async fn manage_pot(mut pot: PotAdc) {
    // let mut read_buffer = unsafe { &mut DMA_BUF[..] };
    // let pin = pot.pin.degrade_adc();

    let tx = WATCH_POT.sender();
    tx.send(5);

    loop {
        let measured = pot.adc.blocking_read(&mut pot.pin);

        info!("pot measured: {}", measured);

        tx.send(measured);

        // pot.adc
        //     .read(
        //         dma.reborrow(),
        //         [(&mut pin, SampleTime::CYCLES810_5)].into_iter(),
        //         &mut read_buffer,
        //     )
        //     .await;
        // // ?

        Timer::after(Duration::from_millis(100)).await;
    }
}
