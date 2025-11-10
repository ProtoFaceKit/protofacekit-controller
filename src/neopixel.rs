use blinksy::{driver::ClocklessDriver, leds::Ws2812};
use blinksy_esp::{ClocklessRmt, ClocklessRmtBuilder, rmt::rmt_buffer_size};
use esp_hal::{
    peripherals::{GPIO4, RMT},
    rmt::Rmt,
    time::Rate,
};

pub const PIXEL_COUNT: usize = 1;
pub const FRAME_BUFFER_SIZE: usize = Ws2812::frame_buffer_size(PIXEL_COUNT);
pub const RMT_BUFFER_SIZE: usize = rmt_buffer_size::<Ws2812>(PIXEL_COUNT);

pub type NeoPixelDriver<'a> = ClocklessDriver<
    Ws2812,
    ClocklessRmt<
        RMT_BUFFER_SIZE,
        Ws2812,
        esp_hal::rmt::Channel<'a, esp_hal::Async, esp_hal::rmt::Tx>,
    >,
>;

/// Create the [ClocklessDriver] for diving the NeoPixel LED
pub fn create_neopixel_driver(
    data_pin: GPIO4<'static>,
    rmt: RMT<'static>,
) -> NeoPixelDriver<'static> {
    // Initialize RMT peripheral (typical base clock 80 MHz).
    let rmt_clk_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(rmt, rmt_clk_freq).unwrap().into_async();
    let rmt_channel = rmt.channel0;

    blinksy::driver::ClocklessDriver::default()
        .with_led::<Ws2812>()
        .with_writer(
            ClocklessRmtBuilder::default()
                .with_rmt_buffer_size::<RMT_BUFFER_SIZE>()
                .with_led::<Ws2812>()
                .with_channel(rmt_channel)
                .with_pin(data_pin)
                .build(),
        )
}
