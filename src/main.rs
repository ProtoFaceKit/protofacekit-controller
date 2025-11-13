#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(try_with_capacity)]

use crate::bluetooth::ble_server;
use crate::data::{Expression, Face};
use crate::face_control::{FaceConsumer, FaceController, FaceExpressionController};
use crate::hub75::{Hub75Peripherals, setup_hub75_dedicated_core};
use crate::neopixel::{create_neopixel_driver, show_neopixel_color};
use blinksy::color::LinearSrgb;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::signal::Signal;
use esp_hal::clock::CpuClock;
use esp_hal::efuse::Efuse;
use esp_hal::gpio::Pin;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let msg = info.message();
    let msg = msg.as_str().unwrap_or("unknown panic message");
    defmt::error!("panic: {}", msg);
    loop {}
}

extern crate alloc;

mod bluetooth;
mod data;
mod face_control;
mod hub75;
mod macros;
mod neopixel;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// Get the device serial number (Just the device mac address padded to 16 bytes)
#[allow(unused)]
fn device_serial_number() -> [u8; 16] {
    let mac = Efuse::read_base_mac_address();
    let mut serial = [0u8; 16];
    serial[..6].copy_from_slice(&mac);
    serial
}

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());

    let peripherals = esp_hal::init(config);

    // Setup the heap allocator
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);

    // Add PSRAM to the heap
    esp_alloc::psram_allocator!(&peripherals.PSRAM, esp_hal::psram);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    defmt::info!("runtime initialized");

    let mut driver = create_neopixel_driver(peripherals.GPIO4, peripherals.RMT);
    show_neopixel_color(&mut driver, LinearSrgb::new(1.0, 0.0, 0.0), 0.01).await;

    let radio_init = esp_radio::init().expect("failed to initialize Wi-Fi/BLE controller");

    let face = &*mk_static!(Mutex<CriticalSectionRawMutex, Face>, Mutex::new(Face::default()));
    let expression_signal =
        mk_static!(Signal::<CriticalSectionRawMutex, Expression>, Signal::new());

    let face_expression_controller = FaceExpressionController::new(expression_signal);
    let face_consumer = FaceConsumer::new(face, expression_signal);
    let face_controller = FaceController::new(face);

    // TODO: Spawn task to listen to microphone and provide expressions
    face_expression_controller.signal(Expression::IDLE);

    // Run the Hub75
    let hub75_peripherals = Hub75Peripherals {
        lcd_cam: peripherals.LCD_CAM,
        dma_channel: peripherals.DMA_CH0,
        red1: peripherals.GPIO42.degrade(),
        grn1: peripherals.GPIO41.degrade(),
        blu1: peripherals.GPIO40.degrade(),
        red2: peripherals.GPIO38.degrade(),
        grn2: peripherals.GPIO39.degrade(),
        blu2: peripherals.GPIO37.degrade(),
        addr0: peripherals.GPIO45.degrade(),
        addr1: peripherals.GPIO36.degrade(),
        addr2: peripherals.GPIO48.degrade(),
        addr3: peripherals.GPIO35.degrade(),
        addr4: peripherals.GPIO21.degrade(),
        blank: peripherals.GPIO14.degrade(),
        clock: peripherals.GPIO2.degrade(),
        latch: peripherals.GPIO47.degrade(),
    };
    setup_hub75_dedicated_core(
        peripherals.CPU_CTRL,
        peripherals.SW_INTERRUPT,
        hub75_peripherals,
        face_consumer,
    );

    // Green light for startup complete
    show_neopixel_color(&mut driver, LinearSrgb::new(0.0, 1.0, 0.0), 0.01).await;

    // Run the bluetooth GATT service
    ble_server(radio_init, peripherals.BT, face_controller).await;
}
