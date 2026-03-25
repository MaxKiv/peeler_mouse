# Esp32 firmware using the esp-idf (freertos) framework

Note:

- SD and Webserver are optional via features.

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
