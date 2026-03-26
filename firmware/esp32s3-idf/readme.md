# Esp32 firmware using the esp-idf (freertos) framework

Firmware for the esp32s3 in the rotating head of the cable peeler project.

# Development Setup (Nix + ESP32-S3)

This project uses [Nix](https://nixos.org/) with flakes to provide a fully reproducible development environment for **ESP32-S3 + esp-idf + Rust** development. All required toolchains, SDKs, and utilities are preconfigured.

---

## Installing Nix

### Windows (via WSL2 – recommended)

```bash
# Install WSL2 first, then inside WSL:
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
```

### macOS / Linux

```bash
curl --proto '=https' --tlsv1.2 -sSf -L https://install.determinate.systems/nix | sh -s -- install
```

After installation:

```bash
source ~/.nix-profile/etc/profile.d/nix.sh
```

---

## Enable Flakes

```bash
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
```

---

## Enter the Development Shell

```bash
git clone <repository-url>
cd peeler_mouse

# Enter ESP32-S3 development environment
nix develop
```

Optional but recommended (requires `direnv`):

```bash
echo "use flake" > .envrc
direnv allow
```

---

## What the Dev Shell Provides

The Nix flake sets up a complete ESP development environment using [`nixpkgs-esp-dev`](https://github.com/mirrexagon/nixpkgs-esp-dev):

- **Rust toolchain (via rust-overlay)**
- **esp-idf (Espressif SDK)**
- **ESP32-S3 target support**
- **espflash** for flashing firmware
- **OpenOCD** (optional debugging)
- **Serial monitor tools**
- **C toolchain required by esp-idf**
- **cargo + rust-analyzer**

---

## Building the Project

Inside the dev shell:

### Build firmware

```bash
cargo build
```

### Build optimized

```bash
cargo build --release
```

### Run on device

```bash
cargo run
```

---

## Flashing & Monitoring

### Flash firmware

```bash
espflash flash target/xtensa-esp32s3-espidf/debug/esp32s3
```

### Monitor serial output

```bash
just monitor
```

Or directly:

```bash
espflash monitor
```

---

## Project Features

Cargo features can be toggled depending on functionality:

```bash
cargo build --features webserver
cargo build --features sd
cargo build --features streaming
```

- `webserver` → enables HTTP server functionality
- `sd` → SD card support
- `streaming` → enables streaming (depends on webserver)
- `experimental` → enables experimental esp-idf features

---

## Code Quality

```bash
cargo check
cargo fmt
cargo clippy
```

---

## Notes on the Setup

- The flake uses:
  - `nixos-unstable` for up-to-date packages
  - [`rust-overlay`] for flexible Rust toolchains
  - [`nixpkgs-esp-dev`] for ESP-IDF integration

- The development shell is defined as:
  - `devShells.esp32s3` (default)

---

## Hardware Target

This project targets:

- **ESP32-S3**
- Uses **esp-idf** (via `esp-idf-sys` + `esp-idf-svc`)
- Supports peripherals like:
  - Camera (via ESP-IDF component)
  - Stepper drivers (RMT + simple drivers)
  - Encoders (AS5048A)

---

# Pin Mapping

## UART (external)

| Signal | GPIO |
| ------ | ---- |
| TX     | 43   |
| RX     | 44   |

---

## Encoder (SPI)

| Signal            | GPIO |
| ----------------- | ---- |
| SCLK              | 1    |
| MISO (serial_out) | 2    |
| MOSI (serial_in)  | 21   |
| CS                | 47   |

---

## Stepper Motor

| Signal       | GPIO |
| ------------ | ---- |
| STEP (RMT)   | 41   |
| DIR          | 42   |
| LIMIT SWITCH | 45   |

---

## Camera (internal)

| Signal | GPIO |
| ------ | ---- |
| XCLK   | 15   |
| D0     | 11   |
| D1     | 9    |
| D2     | 8    |
| D3     | 10   |
| D4     | 12   |
| D5     | 18   |
| D6     | 17   |
| D7     | 16   |
| VSYNC  | 6    |
| HREF   | 7    |
| PCLK   | 13   |
| SDA    | 4    |
| SCL    | 5    |

---

## SD Card _(feature = "sd")_

| Signal | GPIO |
| ------ | ---- |
| CMD    | 38   |
| CLK    | 39   |
| D0     | 40   |

---

## Status LED

| Signal | GPIO |
| ------ | ---- |
| LED    | 48   |

---
