//! # Bluetooth Storage
//!
//! Storage related logic for the BLE bonding storage in flash

use core::ops::Range;
use embassy_embedded_hal::adapter::BlockingAsync;
use embedded_storage_async::nor_flash::NorFlash;
use esp_storage::FlashStorage;
use sequential_storage::cache::NoCache;
use sequential_storage::map::{Key, SerializationError, Value};
use trouble_host::prelude::*;

/// Identity stored as the key for bonding details
#[derive(Clone, PartialEq, Eq)]
struct StoredIdentity(Identity);

impl Key for StoredIdentity {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        let identity = &self.0;
        let is_irk = identity.irk.is_some();

        let length = if is_irk { 23 } else { 7 };
        if buffer.len() < length {
            return Err(SerializationError::BufferTooSmall);
        }

        // Bluetooth address
        buffer[0] = is_irk as u8;
        buffer[1..7].copy_from_slice(identity.bd_addr.raw());

        // Identity resolving key
        if let Some(irk) = identity.irk.as_ref() {
            let le_bytes = irk.to_le_bytes();
            buffer[7..23].copy_from_slice(&le_bytes);
        }

        Ok(length)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }

        let is_irk = buffer[0] == 1;
        let length = if is_irk { 23 } else { 7 };

        if buffer.len() < length {
            return Err(SerializationError::BufferTooSmall);
        }

        let bd_addr = BdAddr::new(buffer[1..7].try_into().unwrap());
        let irk = if is_irk {
            let le_bytes: [u8; 16] = buffer[7..23]
                .try_into()
                .expect("16 bytes from buffer should be convertible");
            let irk = IdentityResolvingKey::from_le_bytes(le_bytes);

            Some(irk)
        } else {
            None
        };

        Ok((StoredIdentity(Identity { bd_addr, irk }), length))
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

pub type FlashBluetoothStorage = BluetoothStorage<BlockingAsync<FlashStorage<'static>>>;

pub struct BluetoothStorage<S: NorFlash> {
    storage: S,
}

impl<S: NorFlash> BluetoothStorage<S> {
    pub fn new(storage: S) -> Self {
        Self { storage }
    }

    /// Read the bonding information
    pub async fn read(&mut self) -> Option<BondInformation> {
        let mut buffer = [0; 48];
        let mut cache = NoCache::new();

        let mut iter = sequential_storage::map::fetch_all_items::<StoredIdentity, _, _>(
            &mut self.storage,
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
            .map(|(StoredIdentity(identity), value)| BondInformation {
                identity,
                security_level: value.security_level,
                is_bonded: true,
                ltk: value.ltk,
            })
    }

    /// Erase the flash
    pub async fn reset(&mut self) -> Result<(), sequential_storage::Error<S::Error>> {
        sequential_storage::erase_all(&mut self.storage, flash_range::<S>()).await?;
        Ok(())
    }

    /// Write the known bond information
    pub async fn write(
        &mut self,
        info: &BondInformation,
    ) -> Result<(), sequential_storage::Error<S::Error>> {
        self.reset().await?;

        let mut buffer = [0; 48];
        let mut cache = NoCache::new();

        let key = StoredIdentity(info.identity);
        let value = StoredBondInformation {
            ltk: info.ltk,
            security_level: info.security_level,
        };
        sequential_storage::map::store_item(
            &mut self.storage,
            flash_range::<S>(),
            &mut cache,
            &mut buffer,
            &key,
            &value,
        )
        .await?;
        Ok(())
    }
}
