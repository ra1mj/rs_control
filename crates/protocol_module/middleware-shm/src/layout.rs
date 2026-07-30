//! Pure checked construction and validation of on-memory layout v1.

use crate::{
    ShmChannelConfig, ShmPayload,
    contract::PayloadContract,
    error::{LayoutError, LayoutField},
};

pub(crate) const CACHE_LINE_BYTES: usize = 64;
pub(crate) const HEADER_BYTES: usize = CACHE_LINE_BYTES * 4;
pub(crate) const LAYOUT_MAGIC: u64 = u64::from_le_bytes(*b"RSHMv001");
pub(crate) const LAYOUT_VERSION: u32 = 1;
pub(crate) const ENDIAN_LITTLE: u32 = 1;
pub(crate) const ATOMIC_WIDTH_BITS: u32 = 64;
pub(crate) const LATEST_MIN_CAPACITY: u32 = 3;

/// Delivery algorithm encoded into the immutable header.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub(crate) enum ChannelKind {
    Latest = 1,
    Ordered = 2,
}

/// Exact immutable identity read from or written to layout v1.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct LayoutSnapshot {
    pub(crate) magic: u64,
    pub(crate) version: u32,
    pub(crate) endian: u32,
    pub(crate) atomic_width: u32,
    pub(crate) channel_kind: u32,
    pub(crate) capacity: u32,
    pub(crate) schema_id: [u8; 16],
    pub(crate) schema_version: u32,
    pub(crate) payload_size: u32,
    pub(crate) payload_alignment: u32,
    pub(crate) slot_stride: u32,
    pub(crate) slots_offset: u64,
    pub(crate) versions_offset: u64,
    pub(crate) total_length: u64,
}

/// Fully checked layout used before any raw mapped-memory view is created.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct LayoutDescriptor {
    snapshot: LayoutSnapshot,
}

impl LayoutDescriptor {
    pub(crate) fn for_payload<T: ShmPayload>(
        kind: ChannelKind,
        config: ShmChannelConfig,
    ) -> Result<Self, LayoutError> {
        let payload = PayloadContract::of::<T>()?;
        let capacity = config.capacity().get();
        if kind == ChannelKind::Latest && capacity < LATEST_MIN_CAPACITY {
            return Err(LayoutError::LatestCapacityTooSmall {
                actual: capacity,
                minimum: LATEST_MIN_CAPACITY,
            });
        }

        let slot_alignment = payload.alignment.max(std::mem::align_of::<u64>());
        let slot_stride = align_up(payload.size, slot_alignment)?;
        let slots_bytes = slot_stride
            .checked_mul(capacity as usize)
            .ok_or(LayoutError::ArithmeticOverflow)?;
        let versions_offset = align_up(
            HEADER_BYTES
                .checked_add(slots_bytes)
                .ok_or(LayoutError::ArithmeticOverflow)?,
            CACHE_LINE_BYTES,
        )?;
        let version_bytes = std::mem::size_of::<u64>()
            .checked_mul(capacity as usize)
            .ok_or(LayoutError::ArithmeticOverflow)?;
        let total_length = align_up(
            versions_offset
                .checked_add(version_bytes)
                .ok_or(LayoutError::ArithmeticOverflow)?,
            CACHE_LINE_BYTES,
        )?;

        Ok(Self {
            snapshot: LayoutSnapshot {
                magic: LAYOUT_MAGIC,
                version: LAYOUT_VERSION,
                endian: ENDIAN_LITTLE,
                atomic_width: ATOMIC_WIDTH_BITS,
                channel_kind: kind as u32,
                capacity,
                schema_id: payload.schema_id,
                schema_version: payload.schema_version,
                payload_size: to_u32(payload.size)?,
                payload_alignment: to_u32(payload.alignment)?,
                slot_stride: to_u32(slot_stride)?,
                slots_offset: HEADER_BYTES as u64,
                versions_offset: to_u64(versions_offset)?,
                total_length: to_u64(total_length)?,
            },
        })
    }

    pub(crate) const fn snapshot(self) -> LayoutSnapshot {
        self.snapshot
    }

    pub(crate) fn validate_observed(self, observed: LayoutSnapshot) -> Result<(), LayoutError> {
        macro_rules! require_equal {
            ($field:ident, $kind:ident) => {
                if observed.$field != self.snapshot.$field {
                    return Err(LayoutError::Incompatible {
                        field: LayoutField::$kind,
                    });
                }
            };
        }

        require_equal!(magic, Magic);
        require_equal!(version, Version);
        require_equal!(endian, Endian);
        require_equal!(atomic_width, AtomicWidth);
        require_equal!(channel_kind, ChannelKind);
        require_equal!(capacity, Capacity);
        require_equal!(schema_id, SchemaId);
        require_equal!(schema_version, SchemaVersion);
        require_equal!(payload_size, PayloadSize);
        require_equal!(payload_alignment, PayloadAlignment);
        require_equal!(slot_stride, SlotStride);
        require_equal!(slots_offset, SlotsOffset);
        require_equal!(versions_offset, VersionsOffset);
        require_equal!(total_length, TotalLength);
        Ok(())
    }
}

fn align_up(value: usize, alignment: usize) -> Result<usize, LayoutError> {
    debug_assert!(alignment.is_power_of_two());
    let mask = alignment - 1;
    value
        .checked_add(mask)
        .map(|sum| sum & !mask)
        .ok_or(LayoutError::ArithmeticOverflow)
}

fn to_u32(value: usize) -> Result<u32, LayoutError> {
    value
        .try_into()
        .map_err(|_| LayoutError::ArithmeticOverflow)
}

fn to_u64(value: usize) -> Result<u64, LayoutError> {
    value
        .try_into()
        .map_err(|_| LayoutError::ArithmeticOverflow)
}

#[cfg(test)]
mod tests {
    use bytemuck::{Pod, Zeroable};

    use super::*;
    use crate::{MemoryPolicy, ShmChannelConfig, error::ContractError};

    #[repr(C)]
    #[derive(Clone, Copy, Pod, Zeroable)]
    struct Payload([u64; 4]);

    impl ShmPayload for Payload {
        const SCHEMA_ID: [u8; 16] = *b"layout-test-v001";
        const SCHEMA_VERSION: u32 = 3;
    }

    fn config(capacity: u32) -> ShmChannelConfig {
        ShmChannelConfig::new(capacity, MemoryPolicy::Prefault).unwrap()
    }

    #[test]
    fn layout_is_cache_aligned_bounded_and_deterministic() {
        let first = LayoutDescriptor::for_payload::<Payload>(ChannelKind::Ordered, config(8))
            .unwrap()
            .snapshot();
        let second = LayoutDescriptor::for_payload::<Payload>(ChannelKind::Ordered, config(8))
            .unwrap()
            .snapshot();
        assert_eq!(first, second);
        assert_eq!(first.slots_offset as usize % CACHE_LINE_BYTES, 0);
        assert_eq!(first.versions_offset as usize % CACHE_LINE_BYTES, 0);
        assert_eq!(first.total_length as usize % CACHE_LINE_BYTES, 0);
        assert_eq!(first.slot_stride, 32);
    }

    #[test]
    fn latest_requires_a_multi_slot_window() {
        assert_eq!(
            LayoutDescriptor::for_payload::<Payload>(ChannelKind::Latest, config(2)),
            Err(LayoutError::LatestCapacityTooSmall {
                actual: 2,
                minimum: 3,
            })
        );
    }

    #[test]
    fn payload_identity_rejects_zero_schema_components() {
        #[repr(transparent)]
        #[derive(Clone, Copy, Pod, Zeroable)]
        struct ZeroId(u64);
        impl ShmPayload for ZeroId {
            const SCHEMA_ID: [u8; 16] = [0; 16];
            const SCHEMA_VERSION: u32 = 1;
        }

        assert_eq!(
            LayoutDescriptor::for_payload::<ZeroId>(ChannelKind::Ordered, config(1)),
            Err(LayoutError::Contract(ContractError::ZeroSchemaId))
        );
    }

    #[test]
    fn alignment_arithmetic_rejects_usize_overflow() {
        assert_eq!(
            align_up(usize::MAX, CACHE_LINE_BYTES),
            Err(LayoutError::ArithmeticOverflow)
        );
    }

    #[test]
    fn every_observed_identity_dimension_is_checked() {
        let descriptor =
            LayoutDescriptor::for_payload::<Payload>(ChannelKind::Ordered, config(8)).unwrap();
        let baseline = descriptor.snapshot();
        let cases = [
            (
                {
                    let mut v = baseline;
                    v.magic ^= 1;
                    v
                },
                LayoutField::Magic,
            ),
            (
                {
                    let mut v = baseline;
                    v.version += 1;
                    v
                },
                LayoutField::Version,
            ),
            (
                {
                    let mut v = baseline;
                    v.endian += 1;
                    v
                },
                LayoutField::Endian,
            ),
            (
                {
                    let mut v = baseline;
                    v.atomic_width += 1;
                    v
                },
                LayoutField::AtomicWidth,
            ),
            (
                {
                    let mut v = baseline;
                    v.channel_kind += 1;
                    v
                },
                LayoutField::ChannelKind,
            ),
            (
                {
                    let mut v = baseline;
                    v.capacity += 1;
                    v
                },
                LayoutField::Capacity,
            ),
            (
                {
                    let mut v = baseline;
                    v.schema_id[0] ^= 1;
                    v
                },
                LayoutField::SchemaId,
            ),
            (
                {
                    let mut v = baseline;
                    v.schema_version += 1;
                    v
                },
                LayoutField::SchemaVersion,
            ),
            (
                {
                    let mut v = baseline;
                    v.payload_size += 1;
                    v
                },
                LayoutField::PayloadSize,
            ),
            (
                {
                    let mut v = baseline;
                    v.payload_alignment += 1;
                    v
                },
                LayoutField::PayloadAlignment,
            ),
            (
                {
                    let mut v = baseline;
                    v.slot_stride += 1;
                    v
                },
                LayoutField::SlotStride,
            ),
            (
                {
                    let mut v = baseline;
                    v.slots_offset += 1;
                    v
                },
                LayoutField::SlotsOffset,
            ),
            (
                {
                    let mut v = baseline;
                    v.versions_offset += 1;
                    v
                },
                LayoutField::VersionsOffset,
            ),
            (
                {
                    let mut v = baseline;
                    v.total_length += 1;
                    v
                },
                LayoutField::TotalLength,
            ),
        ];

        for (observed, field) in cases {
            assert_eq!(
                descriptor.validate_observed(observed),
                Err(LayoutError::Incompatible { field })
            );
        }
        assert_eq!(descriptor.validate_observed(baseline), Ok(()));
    }
}
