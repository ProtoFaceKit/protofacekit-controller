use blinksy::{
    Control, ControlBuilder, color::LinearSrgb, driver::ClocklessDriver, layout::Layout1d,
    layout1d, leds::Ws2812, markers::Dim1d, pattern::Pattern,
};
use blinksy_esp::{ClocklessRmt, ClocklessRmtBuilder, rmt::rmt_buffer_size};
use esp_hal::{
    peripherals::{GPIO4, RMT},
    rmt::Rmt,
    time::Rate,
};

const PIXEL_COUNT: usize = 1;
const RMT_BUFFER_SIZE: usize = rmt_buffer_size::<Ws2812>(PIXEL_COUNT);
const FRAME_BUFFER_SIZE: usize = Ws2812::frame_buffer_size(PIXEL_COUNT);

layout1d!(pub Layout, PIXEL_COUNT);

pub type NeoPixelDriver<'a> = ClocklessDriver<
    Ws2812,
    ClocklessRmt<
        RMT_BUFFER_SIZE,
        Ws2812,
        esp_hal::rmt::Channel<'a, esp_hal::Async, esp_hal::rmt::Tx>,
    >,
>;

pub type NeoPixelControl<'a, S> = Control<
    PIXEL_COUNT,
    3,
    Dim1d,
    blinksy::markers::Async,
    Layout,
    StatePattern<S>,
    NeoPixelDriver<'a>,
>;

pub trait NeoPixelStateColor: Send + 'static {
    fn state_color(&self) -> LinearSrgb;
}

pub struct StatePattern<S: NeoPixelStateColor> {
    state: S,
}

impl<S, Layout> Pattern<Dim1d, Layout> for StatePattern<S>
where
    S: NeoPixelStateColor,
    Layout: Layout1d,
{
    type Params = S;
    type Color = LinearSrgb;

    fn new(state: Self::Params) -> Self {
        Self { state }
    }

    fn tick(&self, _time_in_ms: u64) -> impl Iterator<Item = Self::Color> {
        let color = self.state.state_color();
        Layout::points().map(move |_| color)
    }
}

/// Create the [ClocklessDriver] for diving the NeoPixel LED
fn create_neopixel_driver(data_pin: GPIO4<'static>, rmt: RMT<'static>) -> NeoPixelDriver<'static> {
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

/// Create a new neopixel state control
pub fn create_neopixel_state_control<S: NeoPixelStateColor>(
    data_pin: GPIO4<'static>,
    rmt: RMT<'static>,
    state: S,
) -> NeoPixelControl<'static, S> {
    let driver = create_neopixel_driver(data_pin, rmt);
    ControlBuilder::new_1d_async()
        .with_layout::<Layout, PIXEL_COUNT>()
        .with_pattern::<StatePattern<S>>(state)
        .with_driver(driver)
        .with_frame_buffer_size::<FRAME_BUFFER_SIZE>()
        .build()
}
