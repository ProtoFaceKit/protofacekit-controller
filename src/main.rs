#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(try_with_capacity)]

use crate::data::{Expression, FRAME_WIDTH, Face, RLEPixelProducerIterator, RLESlicePixelIterator};
use crate::neopixel::{FRAME_BUFFER_SIZE, PIXEL_COUNT, create_neopixel_driver};
use alloc::boxed::Box;
use blinksy::color::{ColorCorrection, LinearSrgb};
use blinksy::driver::DriverAsync;
use bt_hci::controller::ExternalController;
use core::iter::{self, Once};
use defmt::{Debug2Format, info};
use embassy_executor::{Spawner, task};
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use embedded_graphics::prelude::Point;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::AnyPin;
use esp_hal::gpio::Pin;
use esp_hal::interrupt::Priority;
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::peripherals::LCD_CAM;
use esp_hal::system::Stack;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_hub75::framebuffer::plain::DmaFrameBuffer;
use esp_hub75::framebuffer::tiling::{ChainTopRightDown, TiledFrameBuffer, compute_tiled_cols};
use esp_hub75::framebuffer::{FrameBufferOperations, compute_frame_count, compute_rows};
use esp_hub75::{Color, Hub75, Hub75Pins16};
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use esp_rtos::embassy::{Executor, InterruptExecutor};
use trouble_host::prelude::*;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let msg = info.message();
    let msg = msg.as_str().unwrap_or("unknown panic message");
    defmt::error!("panic: {}", msg);
    loop {}
}

extern crate alloc;

mod data;
mod neopixel;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

// GATT Server definition
#[gatt_server]
struct Server {
    face_service: ProtoFaceService,
}

/// Service for controlling the ProtoFace display
#[gatt_service(uuid = "bd6f7967-023c-4f0b-aad4-16a8a116f62c")]
struct ProtoFaceService {
    /// Begin face upload
    #[characteristic(uuid = "2a3f5ae2-c3bd-4561-945b-ea5da0787576", write)]
    begin_face: bool,

    /// Display the stored face
    #[characteristic(uuid = "e66d5e7a-7458-4d71-b625-331062166d74", write)]
    display_face: bool,

    /// Begin the provided expression type
    #[characteristic(uuid = "d82855fb-c9ae-4322-9839-89d23839c569", write)]
    begin_expression: u8,

    /// Begin writing a frame, specifies the duration of the frame in milliseconds
    #[characteristic(uuid = "21bced55-0b96-4711-a0f5-cd9653aca013", write)]
    begin_frame: u8,

    /// Frame data chunks
    #[characteristic(uuid = "05940bf3-cc0f-4349-8ae4-e2bb89385540", write)]
    frame_chunk: heapless::Vec<u8, 248>,
}

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const BITS: u8 = 4;

const TILED_COLS: usize = 2;
const TILED_ROWS: usize = 1;

const PANEL_ROWS: usize = 32;
const PANEL_COLS: usize = 64;

const FB_COLS: usize = compute_tiled_cols(PANEL_COLS, TILED_ROWS, TILED_COLS);
const NROWS: usize = compute_rows(PANEL_ROWS);
const FRAME_COUNT: usize = compute_frame_count(BITS);

type Hub75Type = Hub75<'static, esp_hal::Async>;

type FBType = DmaFrameBuffer<PANEL_ROWS, FB_COLS, NROWS, BITS, FRAME_COUNT>;
type TiledFBType = TiledFrameBuffer<
    FBType,
    ChainTopRightDown<PANEL_ROWS, PANEL_COLS, TILED_ROWS, TILED_COLS>,
    PANEL_ROWS,
    PANEL_COLS,
    NROWS,
    BITS,
    FRAME_COUNT,
    TILED_ROWS,
    TILED_COLS,
    FB_COLS,
>;

type FrameBufferExchange = Signal<CriticalSectionRawMutex, &'static mut TiledFBType>;

pub struct Hub75Peripherals<'d> {
    pub lcd_cam: LCD_CAM<'d>,
    pub dma_channel: esp_hal::peripherals::DMA_CH0<'d>,
    pub red1: AnyPin<'d>,
    pub grn1: AnyPin<'d>,
    pub blu1: AnyPin<'d>,
    pub red2: AnyPin<'d>,
    pub grn2: AnyPin<'d>,
    pub blu2: AnyPin<'d>,
    pub addr0: AnyPin<'d>,
    pub addr1: AnyPin<'d>,
    pub addr2: AnyPin<'d>,
    pub addr3: AnyPin<'d>,
    pub addr4: AnyPin<'d>,
    pub blank: AnyPin<'d>,
    pub clock: AnyPin<'d>,
    pub latch: AnyPin<'d>,
}

unsafe extern "C" {
    static _stack_end_cpu0: u32;
    static _stack_start_cpu0: u32;
}
#[esp_rtos::main]
async fn main(_spawner: Spawner) {
    info!("main: stack size:  {}", unsafe {
        core::ptr::addr_of!(_stack_start_cpu0).offset_from(core::ptr::addr_of!(_stack_end_cpu0))
    });

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Setup the heap allocator
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 73744);

    // Add PSRAM to the heap.
    esp_alloc::psram_allocator!(&peripherals.PSRAM, esp_hal::psram);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    let mut driver = create_neopixel_driver(peripherals.GPIO4, peripherals.RMT);
    driver
        .show::<PIXEL_COUNT, FRAME_BUFFER_SIZE, Once<LinearSrgb>, LinearSrgb>(
            iter::once(LinearSrgb::new(1.0, 0.0, 0.0)),
            0.01,
            ColorCorrection::default(),
        )
        .await
        .unwrap();

    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&radio_init, peripherals.BT, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    let address_fmt = defmt::Debug2Format(&address);
    defmt::info!("Our address = {:?}", address_fmt);

    let stack = trouble_host::new(ble_controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    defmt::info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "ProtoFaceKit",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    driver
        .show::<PIXEL_COUNT, FRAME_BUFFER_SIZE, Once<LinearSrgb>, LinearSrgb>(
            iter::once(LinearSrgb::new(0.0, 1.0, 0.0)),
            0.01,
            ColorCorrection::default(),
        )
        .await
        .unwrap();

    static TX: FrameBufferExchange = FrameBufferExchange::new();
    static RX: FrameBufferExchange = FrameBufferExchange::new();

    defmt::info!("initializing frame buffers");
    defmt::info!("size of buffer: {}", core::mem::size_of::<TiledFBType>());

    let fb0 = mk_static!(TiledFBType, TiledFBType::new());
    let fb1 = mk_static!(TiledFBType, TiledFBType::new());

    defmt::info!("initialized frame buffers");
    defmt::info!("init face data");

    let face = &*mk_static!(Mutex<CriticalSectionRawMutex, Face>, Mutex::new(Face::default()));

    defmt::info!("inited face data");

    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    let software_interrupt = sw_ints.software_interrupt2;

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

    // run hub75 and display on second core
    let cpu1_fnctn = {
        move || {
            let hp_executor = mk_static!(
                InterruptExecutor<2>,
                InterruptExecutor::new(software_interrupt)
            );
            let high_pri_spawner = hp_executor.start(Priority::Priority3);

            // hub75 runs as high priority task
            high_pri_spawner
                .spawn(matrix_driver(hub75_peripherals, &RX, &TX, fb1))
                .ok();

            let lp_executor = mk_static!(Executor, Executor::new());
            // display task runs as low priority task
            lp_executor.run(|spawner| {
                spawner.spawn(render_task(face, &TX, &RX, fb0)).ok();
            });
        }
    };

    const DISPLAY_STACK_SIZE: usize = 4096;
    let app_core_stack = mk_static!(Stack<DISPLAY_STACK_SIZE>, Stack::new());

    esp_rtos::start_second_core(
        peripherals.CPU_CTRL,
        sw_ints.software_interrupt0,
        sw_ints.software_interrupt1,
        app_core_stack,
        cpu1_fnctn,
    );

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("ProtoFaceKit", &mut peripheral, &server).await {
                Ok(conn) => {
                    _ = gatt_events_task(&server, &conn, face).await;
                }
                Err(e) => {
                    let e = defmt::Debug2Format(&e);
                    defmt::panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

#[task]
async fn matrix_driver(
    peripherals: Hub75Peripherals<'static>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    fb: &'static mut TiledFBType,
) {
    info!("hub75_task: starting!");
    let channel = peripherals.dma_channel;
    let (_, tx_descriptors) = esp_hal::dma_descriptors!(0, FBType::dma_buffer_size_bytes());

    let pins = Hub75Pins16 {
        red1: peripherals.red1,
        grn1: peripherals.grn1,
        blu1: peripherals.blu1,
        red2: peripherals.red2,
        grn2: peripherals.grn2,
        blu2: peripherals.blu2,
        addr0: peripherals.addr0,
        addr1: peripherals.addr1,
        addr2: peripherals.addr2,
        addr3: peripherals.addr3,
        addr4: peripherals.addr4,
        blank: peripherals.blank,
        clock: peripherals.clock,
        latch: peripherals.latch,
    };

    let mut hub75 = Hub75Type::new_async(
        peripherals.lcd_cam,
        pins,
        channel,
        tx_descriptors,
        Rate::from_mhz(20),
    )
    .expect("failed to create Hub75!");

    // keep the frame buffer in an option so we can swap it
    let mut fb = fb;

    loop {
        // if there is a new buffer available, swap it and send the old one
        if rx.signaled() {
            let new_fb = rx.wait().await;
            tx.signal(fb);
            fb = new_fb;
        }
        let mut xfer = hub75
            .render(fb)
            .map_err(|(e, _hub75)| e)
            .expect("failed to start render!");
        xfer.wait_for_done()
            .await
            .expect("render DMA transfer failed");
        let (result, new_hub75) = xfer.wait();
        hub75 = new_hub75;
        result.expect("transfer failed");
    }
}

/// Task to render the face
#[task]
async fn render_task(
    face: &'static Mutex<CriticalSectionRawMutex, Face>,
    rx: &'static FrameBufferExchange,
    tx: &'static FrameBufferExchange,
    mut fb: &'static mut TiledFBType,
) {
    let mut expression = Expression::IDLE;
    let mut frame_index = 0;

    loop {
        let face = face.lock().await;
        let frames = match face.get_expression_frames(expression) {
            Some(value) => value,
            // Nothing to render
            None => {
                drop(face);
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
        };

        // No frames to render
        if frames.is_empty() {
            drop(face);
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }

        if frame_index >= frames.len() {
            frame_index = 0;
        }

        let frame = &frames[frame_index];

        let pixels = match face.get_frame_pixels(frame) {
            Some(value) => value,
            // Nothing to render
            None => {
                drop(face);
                Timer::after(Duration::from_millis(100)).await;
                continue;
            }
        };

        let iterator = RLEPixelProducerIterator::new(pixels);
        fb.erase();

        for (index, (r, g, b)) in iterator.enumerate() {
            let row = index / FRAME_WIDTH;
            let column = index % FRAME_WIDTH;

            fb.set_pixel(Point::new(column as i32, row as i32), Color::new(r, g, b));
        }

        tx.signal(fb);

        fb = rx.wait().await;

        let duration = frame.duration as u64;
        drop(face);
        Timer::after_millis(duration).await;

        frame_index += 1;
        Timer::after(Duration::from_millis(100)).await;
    }

    // TODO: Render face to hub75
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(_err) = runner.run().await {}
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<'m, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    face: &'m Mutex<CriticalSectionRawMutex, Face>,
) -> Result<(), Error> {
    let begin_face = server.face_service.begin_face;
    let display_face = server.face_service.display_face;
    let begin_expression = server.face_service.begin_expression;
    let begin_frame = server.face_service.begin_frame;
    let frame_chunk = &server.face_service.frame_chunk;

    let mut mutex_guard: Option<MutexGuard<'m, CriticalSectionRawMutex, Face>> = None;

    // let level = server.battery_service.level;
    let _reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(_event) => {
                        // if event.handle() == level.handle {
                        //     // This is how the client reads changes
                        //     let value = server.get(&level);
                        //     let value = defmt::Debug2Format(&value);
                        //     defmt::info!("[gatt] Read Event to Level Characteristic: {:?}", value);
                        // }
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == begin_face.handle {
                            let face = match mutex_guard.as_mut() {
                                Some(value) => value,
                                None => {
                                    let guard = face.lock().await;
                                    mutex_guard.insert(guard)
                                }
                            };

                            defmt::info!("beginning face");

                            face.reset();
                        } else if event.handle() == display_face.handle {
                            if let Some(face) = mutex_guard.take() {
                                let pixels_fmt = Debug2Format(&face.pixels);
                                defmt::info!("data at {}", pixels_fmt);
                            } else {
                                todo!("err mutex guard was never held")
                            }

                            // Mutex has been dropped, control has returned to the renderer
                            defmt::info!("finished face");
                        } else if event.handle() == begin_expression.handle {
                            let face = match mutex_guard.as_mut() {
                                Some(value) => value,
                                None => {
                                    todo!("error render task owns the face, didn't begin face");
                                }
                            };

                            let data = event.data();
                            let value = *data.first().expect("expression must provided the value");
                            let expression =
                                Expression::from_value(value).expect("invalid expression value");

                            defmt::info!("beginning expression {}", expression);

                            if let Err(_error) = face.begin_expression(expression) {
                                todo!("handle begin expression error");
                            }
                        } else if event.handle() == begin_frame.handle {
                            let face = match mutex_guard.as_mut() {
                                Some(value) => value,
                                None => {
                                    todo!("error render task owns the face, didn't begin face");
                                }
                            };

                            let data = event.data();
                            let duration =
                                *data.first().expect("begin frame must provided the value");

                            defmt::info!("beginning frame {}", duration);

                            if let Err(_error) = face.begin_frame(duration) {
                                todo!("handle begin frame error");
                            }
                        } else if event.handle() == frame_chunk.handle {
                            let face = match mutex_guard.as_mut() {
                                Some(value) => value,
                                None => {
                                    todo!("error render task owns the face, didn't begin face");
                                }
                            };

                            let data = event.data();
                            let pixels = RLESlicePixelIterator::new(data);

                            defmt::info!("writing pixel data");

                            if let Err(_error) = face.push_pixels(pixels) {
                                todo!("handle begin frame error");
                            }
                        }
                    }
                    _ => {}
                };
                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => {
                        let e = defmt::Debug2Format(&e);
                        defmt::warn!("[gatt] error sending response: {:?}", e)
                    }
                };
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };

    // disconnected
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];

    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x0f, 0x18]]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    defmt::info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    defmt::info!("[adv] connection established");
    Ok(conn)
}
