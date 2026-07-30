//! Backend startup and fixed-representation contracts.

use std::{convert::Infallible, fmt, marker::PhantomData, num::NonZeroU32, str::FromStr};

use bytemuck::Pod;

use crate::ContractError;

/// Maximum UTF-8 byte length of a namespace-safe channel name.
pub const MAX_CHANNEL_NAME_BYTES: usize = 96;
/// Maximum number of slots in layout v1.
pub const MAX_CHANNEL_CAPACITY: u32 = 65_536;
/// Maximum bytes in one fixed shared representation.
pub const MAX_PAYLOAD_BYTES: usize = 65_536;
/// Maximum alignment accepted by layout v1.
pub const MAX_PAYLOAD_ALIGNMENT: usize = 64;

/// Explicit fixed representation allowed in a shared-memory slot.
///
/// `Pod` owns the unsafe bit-validity/layout promise. The constants add stable
/// semantic compatibility identity; matching Rust type names, size or
/// `needs_drop` are deliberately irrelevant.
pub trait ShmPayload: Pod + Send + Sync + 'static {
    /// Stable nonzero schema identity.
    const SCHEMA_ID: [u8; 16];
    /// Nonzero schema revision for this exact representation.
    const SCHEMA_VERSION: u32;
}

/// Bounded conversion between a semantic port value and its fixed SHM form.
///
/// Encoding a value that already passed its semantic constructor is infallible.
/// Decoding may reject corrupt bytes before they become an application value.
pub trait ShmCodec<T>: Send + 'static {
    /// Fixed shared representation.
    type Repr: ShmPayload;
    /// Semantic decode failure.
    type Error;

    /// Converts valid semantic state without allocation or I/O.
    fn encode(&mut self, value: &T) -> Self::Repr;
    /// Validates a complete shared representation.
    fn decode(&mut self, value: Self::Repr) -> Result<T, Self::Error>;
}

/// Identity conversion for an explicitly approved shared payload.
#[derive(Clone, Copy, Debug)]
pub struct IdentityCodec<T>(PhantomData<fn() -> T>);

impl<T> IdentityCodec<T> {
    /// Creates a stateless identity codec.
    #[must_use]
    pub const fn new() -> Self {
        Self(PhantomData)
    }
}

impl<T> Default for IdentityCodec<T> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T: ShmPayload> ShmCodec<T> for IdentityCodec<T> {
    type Repr = T;
    type Error = Infallible;

    fn encode(&mut self, value: &T) -> Self::Repr {
        *value
    }

    fn decode(&mut self, value: Self::Repr) -> Result<T, Self::Error> {
        Ok(value)
    }
}

/// Strict, fixed-storage channel filename component.
#[derive(Clone, Copy, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ChannelName {
    bytes: [u8; MAX_CHANNEL_NAME_BYTES],
    len: u8,
}

impl ChannelName {
    /// Validates the exact lowercase filename-safe grammar.
    pub fn new(value: &str) -> Result<Self, ContractError> {
        let source = value.as_bytes();
        if source.is_empty() {
            return Err(ContractError::EmptyName);
        }
        if source.len() > MAX_CHANNEL_NAME_BYTES {
            return Err(ContractError::NameTooLong {
                actual: source.len(),
                maximum: MAX_CHANNEL_NAME_BYTES,
            });
        }
        if source[0] == b'.' {
            return Err(ContractError::LeadingDot);
        }
        if source.windows(2).any(|window| window == b"..") {
            return Err(ContractError::Traversal);
        }
        for (index, byte) in source.iter().copied().enumerate() {
            if !(byte.is_ascii_lowercase()
                || byte.is_ascii_digit()
                || matches!(byte, b'.' | b'_' | b'-'))
            {
                return Err(ContractError::InvalidNameByte { index, byte });
            }
        }

        let mut bytes = [0; MAX_CHANNEL_NAME_BYTES];
        bytes[..source.len()].copy_from_slice(source);
        Ok(Self {
            bytes,
            len: source.len() as u8,
        })
    }

    /// Returns the already-validated name.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // Every accepted byte is ASCII.
        std::str::from_utf8(&self.bytes[..usize::from(self.len)])
            .expect("ChannelName stores validated ASCII")
    }
}

impl fmt::Debug for ChannelName {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_tuple("ChannelName")
            .field(&self.as_str())
            .finish()
    }
}

impl fmt::Display for ChannelName {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl FromStr for ChannelName {
    type Err = ContractError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Self::new(value)
    }
}

/// Startup policy for RT-visible mapped pages.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum MemoryPolicy {
    /// Populate and explicitly touch all pages before returning an endpoint.
    #[default]
    Prefault,
    /// Additionally require the complete mapping to be locked in RAM.
    LockRequired,
}

/// Validated bounded channel construction settings.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ShmChannelConfig {
    capacity: NonZeroU32,
    memory_policy: MemoryPolicy,
}

impl ShmChannelConfig {
    /// Validates nonzero bounded capacity.
    pub fn new(capacity: u32, memory_policy: MemoryPolicy) -> Result<Self, ContractError> {
        let capacity = NonZeroU32::new(capacity).ok_or(ContractError::ZeroCapacity)?;
        if capacity.get() > MAX_CHANNEL_CAPACITY {
            return Err(ContractError::CapacityTooLarge {
                actual: capacity.get(),
                maximum: MAX_CHANNEL_CAPACITY,
            });
        }
        Ok(Self {
            capacity,
            memory_policy,
        })
    }

    /// Fixed slot count.
    #[must_use]
    pub const fn capacity(self) -> NonZeroU32 {
        self.capacity
    }

    /// Startup prefault/lock policy.
    #[must_use]
    pub const fn memory_policy(self) -> MemoryPolicy {
        self.memory_policy
    }
}

/// Validated representation identity used by layout construction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct PayloadContract {
    pub(crate) schema_id: [u8; 16],
    pub(crate) schema_version: u32,
    pub(crate) size: usize,
    pub(crate) alignment: usize,
}

impl PayloadContract {
    pub(crate) fn of<T: ShmPayload>() -> Result<Self, ContractError> {
        if T::SCHEMA_ID == [0; 16] {
            return Err(ContractError::ZeroSchemaId);
        }
        if T::SCHEMA_VERSION == 0 {
            return Err(ContractError::ZeroSchemaVersion);
        }
        let size = std::mem::size_of::<T>();
        if size == 0 {
            return Err(ContractError::ZeroSizedPayload);
        }
        if size > MAX_PAYLOAD_BYTES {
            return Err(ContractError::PayloadTooLarge {
                actual: size,
                maximum: MAX_PAYLOAD_BYTES,
            });
        }
        let alignment = std::mem::align_of::<T>();
        if alignment > MAX_PAYLOAD_ALIGNMENT {
            return Err(ContractError::PayloadAlignmentTooLarge {
                actual: alignment,
                maximum: MAX_PAYLOAD_ALIGNMENT,
            });
        }
        Ok(Self {
            schema_id: T::SCHEMA_ID,
            schema_version: T::SCHEMA_VERSION,
            size,
            alignment,
        })
    }
}

#[cfg(test)]
mod tests {
    use bytemuck::{Pod, Zeroable};

    use super::*;

    #[repr(C)]
    #[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
    struct TestPayload {
        generation: u64,
        value: u64,
    }

    impl ShmPayload for TestPayload {
        const SCHEMA_ID: [u8; 16] = *b"rs-shm-test-v001";
        const SCHEMA_VERSION: u32 = 1;
    }

    #[test]
    fn channel_name_round_trips_exact_ascii() {
        let name = ChannelName::new("motion.chassis_state-1").unwrap();
        assert_eq!(name.as_str(), "motion.chassis_state-1");
        assert_eq!(name.to_string(), "motion.chassis_state-1");
    }

    #[test]
    fn channel_name_rejects_empty_traversal_separators_and_uppercase() {
        assert_eq!(ChannelName::new(""), Err(ContractError::EmptyName));
        assert_eq!(ChannelName::new(".hidden"), Err(ContractError::LeadingDot));
        assert_eq!(ChannelName::new("a..b"), Err(ContractError::Traversal));
        assert!(matches!(
            ChannelName::new("a/b"),
            Err(ContractError::InvalidNameByte {
                index: 1,
                byte: b'/'
            })
        ));
        assert!(matches!(
            ChannelName::new("Robot"),
            Err(ContractError::InvalidNameByte {
                index: 0,
                byte: b'R'
            })
        ));
    }

    #[test]
    fn channel_config_rejects_zero_and_excessive_capacity() {
        assert_eq!(
            ShmChannelConfig::new(0, MemoryPolicy::Prefault),
            Err(ContractError::ZeroCapacity)
        );
        assert!(matches!(
            ShmChannelConfig::new(MAX_CHANNEL_CAPACITY + 1, MemoryPolicy::Prefault),
            Err(ContractError::CapacityTooLarge { .. })
        ));
    }

    #[test]
    fn explicit_payload_identity_and_identity_codec_round_trip() {
        let contract = PayloadContract::of::<TestPayload>().unwrap();
        assert_eq!(contract.schema_id, *b"rs-shm-test-v001");
        assert_eq!(contract.schema_version, 1);
        assert_eq!(contract.size, 16);

        let value = TestPayload {
            generation: 7,
            value: 42,
        };
        let mut codec = IdentityCodec::<TestPayload>::new();
        let encoded = codec.encode(&value);
        assert_eq!(codec.decode(encoded), Ok(value));
    }
}
