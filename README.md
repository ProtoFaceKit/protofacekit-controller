<h1>
    <img src="assets/banner-controller.svg" height="50">
</h1>

Powered by the [Adafruit Matrix Portal S3](https://www.adafruit.com/product/5778) ([Pins here](https://learn.adafruit.com/assets/140086)) and the [Rust](https://rust-lang.org/) language

## Features

- [x] Proximity/Touch detection (GY-9960-3.3 APDS-9960)
- [x] Sound detection (MAX9814)
- [x] Bluetooth control
- [x] Drives HUB75 RGB matrix panels (2x 64x32 panels)

## Pins

### GY-9960-3.3 APDS-9960

- SCL - GPIO10 / A3
- SDA - GPIO9 / A2
- VCC - Board 3.3V
- GND - Board Ground
- VL - Board 3.3V

### MAX9814

- GND - Board Ground
- VDD - Board 3.3V
- OUT - GPIO11 / A4
