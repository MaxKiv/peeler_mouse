export RUST_BACKTRACE := "1"

openocd:
    openocd -f interface/stlink.cfg -f target/stm32g4x.cfg

build:
    cargo build

clean:
    cargo clean

check:
    cargo check

run CRATE:
    cargo run --bin {{ CRATE }}

attach TARGET:
    probe-rs attach --chip STM32H753ZITx ./target/thumbv7em-none-eabihf/debug/{{ TARGET }}
