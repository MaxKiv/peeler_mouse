#![no_std]
#![no_main]

pub mod clocks;
pub mod comms;
pub mod hmi;
pub mod motor;
pub mod ringbuffer;
pub mod supervisor;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::I2C2;
use embassy_stm32::peripherals::USART3;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{Config, bind_interrupts, i2c, usart};

use crate::hmi::lcd::setup::LcdPeripherals;
use crate::hmi::{
    button::{ButtonMode, ButtonPeripherals},
    encoder::QuadratureEncoderPeripherals,
};
use crate::motor::rotation::RotationMotorPeripherals;
use crate::motor::translation::TranslationMotorPeripherals;

use embedded_alloc::LlffHeap;

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<I2C2>;
    USART3 => usart::BufferedInterruptHandler<USART3>;
});

use {defmt_rtt as _, panic_probe as _};

// ---- Heap -----
#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let config = Config::default();
    let config = clocks::setup_clocks(config);
    let p = embassy_stm32::init(config);
    info!("Clocks configured - Hello World!");

    // ---- Heap Setup for LCD -----
    setup_heap();

    // ---- Motor Peripheral declarations -----
    let linear_step_pwm_pin = p.PC6;
    let linear_step_pwm = PwmPin::new(linear_step_pwm_pin, OutputType::PushPull);
    let linear_step_timer = p.TIM8;
    let linear_pwm = SimplePwm::new(
        linear_step_timer,
        Some(linear_step_pwm),
        None,
        None,
        None,
        khz(10),
        Default::default(),
    );
    let translation_peri = TranslationMotorPeripherals {
        pwm: linear_pwm,
        dir: p.PF8,
    };

    let rotation_step_pwm_pin = p.PB6;
    let rotationstep_pwm = PwmPin::new(rotation_step_pwm_pin, OutputType::PushPull);
    let rotation_step_timer = p.TIM4;
    let rotation_pwm = SimplePwm::new(
        rotation_step_timer,
        Some(rotationstep_pwm),
        None,
        None,
        None,
        khz(10),
        Default::default(),
    );
    let rotation_peri = RotationMotorPeripherals {
        pwm: rotation_pwm,
        dir: p.PF9,
    };

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
    let button_d = ButtonPeripherals {
        pin: p.PD7,
        ch: p.EXTI7,
    };
    let button_a = ButtonPeripherals {
        pin: p.PD6,
        ch: p.EXTI6,
    };
    let button_b = ButtonPeripherals {
        pin: p.PD5,
        ch: p.EXTI5,
    };
    let button_c = ButtonPeripherals {
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

    // Uart
    let comms_peri = comms::peripherals::CommsPeripherals {
        uart: p.USART3,
        tx: p.PB10,
        rx: p.PB11,
    };

    // ---- HMI Task Construction -----
    hmi::button::DebouncedButton::run(
        button_d,
        &supervisor::task::BUTTON_D,
        "green",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        button_a,
        &supervisor::task::BUTTON_A,
        "blue",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        button_b,
        &supervisor::task::BUTTON_B,
        "purple",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        button_c,
        &supervisor::task::BUTTON_C,
        "gray",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::button::DebouncedButton::run(
        encoder_button,
        &supervisor::task::ENCODER_PRESSED,
        "encoder",
        ButtonMode::FallingEdge,
        &spawner,
    );

    hmi::encoder::QuadratureEncoder::run(encoder_peri, &spawner);
    hmi::lcd::setup::setup(lcd_peri, &spawner);

    // ---- Motor Task Construction -----
    motor::controller::setup(&spawner);
    motor::translation::setup(translation_peri, &spawner);
    motor::rotation::setup(rotation_peri, &spawner);

    // ---- Supervisor -----
    supervisor::task::setup(&spawner);

    // ---- UART Communication tasks -----
    comms::task::setup(&spawner, comms_peri);
}

// Set up a 128KiB heap in AXI SRAM
fn setup_heap() {
    use core::mem::MaybeUninit;
    const HEAP_SIZE: usize = 1024 * 128;

    #[unsafe(link_section = ".axi_heap")]
    static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
    unsafe { HEAP.init(&raw mut HEAP_MEM as usize, HEAP_SIZE) }
}
