use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::peripherals::*;

pub struct Button<'a> {
    pub input: ExtiInput<'a>,
    pub name: &'static str,
}

pub struct ButtonPeripherals {
    pub b1: (PE2, EXTI2),
    pub b2: (PE3, EXTI3),
    pub b3: (PE4, EXTI4),
    pub b4: (PE5, EXTI5),
    pub b5: (PE6, EXTI6),
}

impl ButtonPeripherals {
    pub fn new(p: embassy_stm32::Peripherals) -> (Self, embassy_stm32::Peripherals) {
        (
            ButtonPeripherals {
                b1: (*p.PE2, *p.EXTI2),
                b2: (*p.PE3, *p.EXTI3),
                b3: (*p.PE4, *p.EXTI4),
                b4: (*p.PE5, *p.EXTI5),
                b5: (*p.PE6, *p.EXTI6),
            },
            p,
        )
    }
}

pub fn setup(p: embassy_stm32::Peripherals, spawner: &Spawner) -> embassy_stm32::Peripherals {
    let (pins, p) = ButtonPeripherals::new(p);

    spawner.spawn(manage_button(pins)).unwrap();

    p
}

#[embassy_executor::task]
pub async fn manage_button(pins: ButtonPeripherals) {
    let handle_2 = handle_button(button_2);
    let handle_1 = handle_button(button_1);
    let handle_3 = handle_button(button_3);
    let handle_4 = handle_button(button_4);
    let handle_5 = handle_button(button_5);

    info!("Press a button...");

    embassy_futures::select::select5(handle_1, handle_2, handle_3, handle_4, handle_5).await;
}

async fn handle_button(mut button: Button<'_>) {
    loop {
        button.input.wait_for_rising_edge().await;
        info!("button {} pressed", button.name);
        button.input.wait_for_falling_edge().await;
        info!("Released!");
    }
}
