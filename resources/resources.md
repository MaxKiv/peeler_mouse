# How to flash esp32cam

1. Connect some USB - UART converter to esp32 (Rx: GPIO3, Tx: GPIO1)
2. Turn off power
3. Hold IO0 low
4. Turn on power
5. CAM LED should now flash indicating bootloader is ready

# Building a Wifi controlled car using esp32cam and Rust

Excellent source containing very similar code to what I need
https://jamesmcm.github.io/blog/esp32-wifi-tank/

# Plant remote management and automation

Excellent source for the nix flake esp32 toolchain setup, and some commmon
functionality like wifi.
https://github.com/wyatt-avilla/rainworld/issues
