#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![feature(try_with_capacity)]

use crate::data::{Expression, Face, RLESlicePixelIterator};
use crate::neopixel::{FRAME_BUFFER_SIZE, PIXEL_COUNT, create_neopixel_driver};
use blinksy::color::{ColorCorrection, LinearSrgb};
use blinksy::driver::DriverAsync;
use bt_hci::controller::ExternalController;
use core::iter::{self, Once};
use defmt::info;
use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::{Mutex, MutexGuard};
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use esp_radio::ble::controller::BleConnector;
use trouble_host::prelude::*;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    defmt::error!("panic");
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

#[esp_rtos::main]
async fn main(_spawner: Spawner) {
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

    let face: Mutex<CriticalSectionRawMutex, Face> = Mutex::new(Face::default());

    defmt::info!("allocate face data");

    let _ = join3(ble_task(runner), render_task(&face), async {
        loop {
            match advertise("ProtoFaceKit", &mut peripheral, &server).await {
                Ok(conn) => {
                    _ = gatt_events_task(&server, &conn, &face).await;
                }
                Err(e) => {
                    let e = defmt::Debug2Format(&e);
                    defmt::panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0/examples/src/bin
}

/// Task to render the face
async fn render_task<'m>(face: &'m Mutex<CriticalSectionRawMutex, Face>) {
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
                            if mutex_guard.take().is_none() {
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

                            defmt::info!("beginning expression");

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

                            defmt::info!("beginning frame");

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
