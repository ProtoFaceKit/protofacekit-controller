use core::ops::Range;

use crate::data::Expression;
use crate::face_control::FaceController;
use embassy_futures::join::join;
use embedded_storage_async::nor_flash::NorFlash;
use esp_hal::efuse::Efuse;
use esp_hal::peripherals::BT;
use esp_hal::rng::Trng;
use esp_radio::ble::controller::BleConnector;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{Key, SerializationError, Value};
use trouble_host::prelude::*;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 1;

type PacketPool = DefaultPacketPool;

// GATT Server definition
#[gatt_server]
pub struct Server {
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

#[derive(Debug, Clone, PartialEq, Eq)]
struct StoredAddr(BdAddr);

impl Key for StoredAddr {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 6 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..6].copy_from_slice(self.0.raw());
        Ok(6)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() < 6 {
            Err(SerializationError::BufferTooSmall)
        } else {
            Ok((StoredAddr(BdAddr::new(buffer[0..6].try_into().unwrap())), 6))
        }
    }
}

struct StoredBondInformation {
    ltk: LongTermKey,
    security_level: SecurityLevel,
}

impl<'a> Value<'a> for StoredBondInformation {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < 17 {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0..16].copy_from_slice(self.ltk.to_le_bytes().as_slice());
        buffer[16] = match self.security_level {
            SecurityLevel::NoEncryption => 0,
            SecurityLevel::Encrypted => 1,
            SecurityLevel::EncryptedAuthenticated => 2,
        };
        Ok(17)
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<(Self, usize), SerializationError>
    where
        Self: Sized,
    {
        if buffer.len() < 17 {
            return Err(SerializationError::BufferTooSmall);
        }

        let ltk = LongTermKey::from_le_bytes(buffer[0..16].try_into().unwrap());
        let security_level = match buffer[16] {
            0 => SecurityLevel::NoEncryption,
            1 => SecurityLevel::Encrypted,
            2 => SecurityLevel::EncryptedAuthenticated,
            _ => return Err(SerializationError::InvalidData),
        };
        Ok((
            StoredBondInformation {
                ltk,
                security_level,
            },
            17,
        ))
    }
}

fn flash_range<S: NorFlash>() -> Range<u32> {
    let start_addr = 0x7D0000;
    let data_size = 2 * S::ERASE_SIZE as u32;

    start_addr..(start_addr + data_size)
}

async fn store_bonding_info<S: NorFlash>(
    storage: &mut S,
    info: &BondInformation,
) -> Result<(), sequential_storage::Error<S::Error>> {
    let storage_range = flash_range::<S>();
    sequential_storage::erase_all(storage, storage_range.clone()).await?;
    let mut buffer = [0; 32];
    let key = StoredAddr(info.identity.bd_addr);
    let value = StoredBondInformation {
        ltk: info.ltk,
        security_level: info.security_level,
    };
    sequential_storage::map::store_item(
        storage,
        storage_range,
        &mut NoCache::new(),
        &mut buffer,
        &key,
        &value,
    )
    .await?;
    Ok(())
}

async fn load_bonding_info<S: NorFlash>(storage: &mut S) -> Option<BondInformation> {
    let mut buffer = [0; 32];
    let mut cache = NoCache::new();
    let mut iter = sequential_storage::map::fetch_all_items::<StoredAddr, _, _>(
        storage,
        flash_range::<S>(),
        &mut cache,
        &mut buffer,
    )
    .await
    .ok()?;

    iter.next::<StoredBondInformation>(&mut buffer)
        .await
        .ok()
        .and_then(|value| value)
        .map(|(key, value)| BondInformation {
            identity: Identity {
                bd_addr: key.0,
                irk: None,
            },
            security_level: value.security_level,
            is_bonded: true,
            ltk: value.ltk,
        })
}

/// Execute a BLE GATT server
pub async fn ble_server<S: NorFlash>(
    radio_init: esp_radio::Controller<'_>,
    device: BT<'static>,
    mut rand: Trng,
    face: FaceController,
    mut storage: S,
) {
    let transport = BleConnector::new(&radio_init, device, Default::default()).unwrap();
    let ble_controller = ExternalController::<_, 20>::new(transport);
    let mut resources: HostResources<PacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();

    let address: Address = Address::random(Efuse::read_base_mac_address());
    defmt::info!("device MAC address = {:?}", defmt::Debug2Format(&address));

    let stack = trouble_host::new(ble_controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(&mut rand)
        .set_io_capabilities(IoCapabilities::DisplayYesNo);

    if let Some(bond_info) = load_bonding_info(&mut storage).await {
        defmt::info!("[ble] loaded bond information");
        stack.add_bond_information(bond_info).unwrap();
        true
    } else {
        defmt::info!("no bond information found");
        false
    };

    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    defmt::info!("starting GATT service");

    // Name must be short to fit
    let name = "ProtoFaceKit";

    // Short name is required for advertising in order to fit the
    // service identifier
    let short_name = "PFK";

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name,
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .expect("failed to create bluetooth server");

    join(ble_task(runner), async {
        loop {
            let connection = match advertise(short_name, &mut peripheral, &server).await {
                Ok(value) => value,
                Err(err) => {
                    defmt::panic!("[adv] error: {}", err);
                }
            };

            if let Err(err) =
                gatt_events_task(&mut storage, &server, connection, face.clone()).await
            {
                defmt::error!("[gatt] error handling connection: {}", err);
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
pub async fn ble_task<C: Controller>(mut runner: Runner<'_, C, PacketPool>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            defmt::error!("[ble] controller error: {}", e);
        }
    }
}

/// Handle GATT events for a connection
pub async fn gatt_events_task<S: NorFlash>(
    storage: &mut S,
    server: &Server<'_>,
    conn: GattConnection<'_, '_, PacketPool>,
    mut face: FaceController,
) -> Result<(), Error> {
    let begin_face = server.face_service.begin_face;
    let display_face = server.face_service.display_face;
    let begin_expression = server.face_service.begin_expression;
    let begin_frame = server.face_service.begin_frame;
    let frame_chunk = &server.face_service.frame_chunk;

    // Mark connection as bondable
    conn.raw().set_bondable(true)?;

    if !conn.raw().security_level()?.encrypted() {
        // Request security on the connection
        conn.raw().request_security()?;
    }

    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::PassKeyDisplay(key) => {
                defmt::info!("[gatt] passkey display: {}", key);
            }
            GattConnectionEvent::PassKeyConfirm(key) => {
                defmt::info!(
                    "Press the yes or no button to confirm pairing with key = {}",
                    key
                );
                // match select(yes.wait_for_low(), no.wait_for_low()).await {
                //     Either::First(_) => {
                //         info!("[gatt] confirming pairing");
                //         conn.pass_key_confirm()?
                //     }
                //     Either::Second(_) => {
                //         info!("[gatt] denying pairing");
                //         conn.pass_key_cancel()?
                //     }
                // }
                conn.pass_key_confirm()?
            }
            GattConnectionEvent::PairingComplete {
                security_level,
                bond,
            } => {
                defmt::info!("[gatt] pairing complete: {:?}", security_level);
                if let Some(bond) = bond {
                    if let Err(err) = store_bonding_info(storage, &bond).await {
                        let err = defmt::Debug2Format(&err);
                        defmt::panic!("failed to store bonding info {}", err);
                    }
                    defmt::info!("Bond information stored");
                }
                defmt::info!("[gatt] pair complete finished")
            }
            GattConnectionEvent::PairingFailed(err) => {
                defmt::error!("[gatt] pairing error: {:?}", err);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(_event) => {
                        if conn.raw().security_level()?.encrypted() {
                            Ok(())
                        } else {
                            defmt::warn!("unencrypted read attempted");
                            // User tried to write without encryption
                            Err(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                        //
                    }
                    GattEvent::Write(event) => {
                        if conn.raw().security_level()?.encrypted() {
                            match event.handle() {
                                h if h == begin_face.handle => {
                                    face.begin_face().await;
                                }

                                h if h == display_face.handle => {
                                    face.display_face();
                                }

                                h if h == begin_expression.handle => {
                                    let data = event.data();
                                    let value =
                                        *data.first().expect("expression must provided the value");
                                    let expression = Expression::from_value(value)
                                        .expect("invalid expression value");
                                    if let Err(err) = face.begin_expression(expression) {
                                        defmt::error!("failed to begin_expression: {}", err);
                                    }
                                }

                                h if h == begin_frame.handle => {
                                    let data = event.data();
                                    let duration =
                                        *data.first().expect("begin frame must provided the value");

                                    if let Err(err) = face.begin_frame(duration) {
                                        defmt::error!("failed to begin_frame: {}", err);
                                    }
                                }

                                h if h == frame_chunk.handle => {
                                    let data = event.data();
                                    if let Err(err) = face.push_pixels(data) {
                                        defmt::error!("failed to begin_frame: {}", err);
                                    }
                                }

                                _ => {}
                            }

                            Ok(())
                        } else {
                            defmt::warn!("unencrypted read attempted");
                            // User tried to write without encryption
                            Err(AttErrorCode::INSUFFICIENT_ENCRYPTION)
                        }
                    }
                    GattEvent::Other(_) => {
                        defmt::info!("[gatt] got other event");

                        Ok(())
                    }
                };

                let result = if let Err(code) = result {
                    event.reject(code)
                } else {
                    event.accept()
                };

                // This step is also performed at drop(), but writing it explicitly is necessary
                // in order to ensure reply is sent.
                match result {
                    Ok(reply) => reply.send().await,
                    Err(e) => {
                        defmt::warn!("[gatt] error sending response: {:?}", e)
                    }
                };
            }
            _ => {}
        }
    };

    defmt::info!("[gatt] disconnected: {:?}", reason);

    // disconnected
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
pub async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];

    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[[
                44u8, 246u8, 22u8, 161u8, 168u8, 22u8, 212u8, 170u8, 11u8, 79u8, 60u8, 2u8, 103u8,
                121u8, 111u8, 189u8,
            ]]),
            AdStructure::ShortenedLocalName(name.as_bytes()),
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
