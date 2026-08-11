//! Fixed local transport diagnostics for operator observation.
//!
//! The snapshot is diagnostic only. It carries no authority and its absence,
//! freshness or delivery must never affect navigation execution.

use arrayvec::ArrayVec;
use bytemuck::{Pod, Zeroable};
use message_model::{AsciiId, Header, Id128, MAX_ID_LEN};
use middleware_api::ChannelCounters;
use middleware_shm::{ShmCodec, ShmDiagnostics, ShmPayload};

use crate::{
    NavigationIpcError,
    wire::{NowFn, WireAsciiId, WireHeader, monotonic_now_ns, validate_header_at},
};

/// Maximum channel counter records in one navigation transport snapshot.
pub const MAX_NAVIGATION_TRANSPORT_CHANNELS: usize = 16;

/// SHM-specific observations retained next to backend-neutral counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NavigationShmDiagnostics {
    /// Attach attempts rejected by a live endpoint lock.
    pub attach_rejected: u64,
    /// Reads abandoned after bounded seqlock validation failed.
    pub read_races: u64,
    /// Complete fixed representations rejected by the semantic codec.
    pub decode_rejected: u64,
    /// Stale active metadata repaired after kernel-lock acquisition.
    pub reclaimed_endpoints: u64,
    /// Ordered generations abandoned when a subscriber lease was claimed.
    pub skipped_backlog: u64,
}

impl From<ShmDiagnostics> for NavigationShmDiagnostics {
    fn from(value: ShmDiagnostics) -> Self {
        Self {
            attach_rejected: value.attach_rejected,
            read_races: value.read_races,
            decode_rejected: value.decode_rejected,
            reclaimed_endpoints: value.reclaimed_endpoints,
            skipped_backlog: value.skipped_backlog,
        }
    }
}

/// One bounded channel counter record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NavigationTransportChannelSnapshot {
    channel_id: AsciiId<MAX_ID_LEN>,
    counters: ChannelCounters,
    shm_diagnostics: NavigationShmDiagnostics,
}

impl NavigationTransportChannelSnapshot {
    /// Constructs one record from a stable channel identity and counter snapshots.
    #[must_use]
    pub const fn new(
        channel_id: AsciiId<MAX_ID_LEN>,
        counters: ChannelCounters,
        shm_diagnostics: NavigationShmDiagnostics,
    ) -> Self {
        Self {
            channel_id,
            counters,
            shm_diagnostics,
        }
    }

    /// Returns the stable local channel identity.
    #[must_use]
    pub const fn channel_id(self) -> AsciiId<MAX_ID_LEN> {
        self.channel_id
    }

    /// Returns backend-neutral delivery counters.
    #[must_use]
    pub const fn counters(self) -> ChannelCounters {
        self.counters
    }

    /// Returns SHM-only diagnostic counters.
    #[must_use]
    pub const fn shm_diagnostics(self) -> NavigationShmDiagnostics {
        self.shm_diagnostics
    }
}

/// Fixed-capacity, fresh transport diagnostic snapshot.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NavigationTransportDiagnosticsSnapshot {
    header: Header,
    process_epoch: u64,
    command_id: Option<Id128>,
    channels: ArrayVec<NavigationTransportChannelSnapshot, MAX_NAVIGATION_TRANSPORT_CHANNELS>,
}

impl NavigationTransportDiagnosticsSnapshot {
    /// Constructs a snapshot after checking process identity and unique channels.
    pub fn new(
        header: Header,
        process_epoch: u64,
        command_id: Option<Id128>,
        channels: ArrayVec<NavigationTransportChannelSnapshot, MAX_NAVIGATION_TRANSPORT_CHANNELS>,
    ) -> Result<Self, NavigationIpcError> {
        if process_epoch == 0 {
            return Err(NavigationIpcError::TransportDiagnostics(
                "zero process epoch",
            ));
        }
        if channels.is_empty() {
            return Err(NavigationIpcError::TransportDiagnostics(
                "empty channel set",
            ));
        }
        for (index, channel) in channels.iter().enumerate() {
            if channels[..index]
                .iter()
                .any(|previous| previous.channel_id == channel.channel_id)
            {
                return Err(NavigationIpcError::TransportDiagnostics(
                    "duplicate channel ID",
                ));
            }
        }
        Ok(Self {
            header,
            process_epoch,
            command_id,
            channels,
        })
    }

    /// Returns source, sequence and freshness metadata.
    #[must_use]
    pub const fn header(&self) -> Header {
        self.header
    }

    /// Returns the supervisor-owned process epoch.
    #[must_use]
    pub const fn process_epoch(&self) -> u64 {
        self.process_epoch
    }

    /// Returns the canonical command identity when one is active.
    #[must_use]
    pub const fn command_id(&self) -> Option<Id128> {
        self.command_id
    }

    /// Returns channel records in deterministic publisher order.
    #[must_use]
    pub fn channels(&self) -> &[NavigationTransportChannelSnapshot] {
        self.channels.as_slice()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
struct NavigationTransportChannelRepr {
    channel_id: WireAsciiId,
    published: u64,
    consumed: u64,
    overwritten: u64,
    full: u64,
    disconnected: u64,
    attach_rejected: u64,
    read_races: u64,
    decode_rejected: u64,
    reclaimed_endpoints: u64,
    skipped_backlog: u64,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationTransportDiagnosticsRepr {
    header: WireHeader,
    process_epoch: u64,
    command_id: [u8; 16],
    channels: [NavigationTransportChannelRepr; MAX_NAVIGATION_TRANSPORT_CHANNELS],
    channel_count: u8,
    command_present: u8,
    reserved: [u8; 6],
}

impl ShmPayload for NavigationTransportDiagnosticsRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-diag-v001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Transport diagnostic codec using receive-time freshness validation.
#[derive(Clone, Copy)]
pub struct NavigationTransportDiagnosticsCodec {
    now: NowFn,
}

impl NavigationTransportDiagnosticsCodec {
    /// Uses a caller-supplied local monotonic clock function.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationTransportDiagnosticsCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationTransportDiagnosticsSnapshot> for NavigationTransportDiagnosticsCodec {
    type Repr = NavigationTransportDiagnosticsRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationTransportDiagnosticsSnapshot) -> Self::Repr {
        let mut channels =
            [NavigationTransportChannelRepr::zeroed(); MAX_NAVIGATION_TRANSPORT_CHANNELS];
        for (index, channel) in value.channels().iter().enumerate() {
            let counters = channel.counters();
            let diagnostics = channel.shm_diagnostics();
            channels[index] = NavigationTransportChannelRepr {
                channel_id: WireAsciiId::encode(channel.channel_id()),
                published: counters.published,
                consumed: counters.consumed,
                overwritten: counters.overwritten,
                full: counters.full,
                disconnected: counters.disconnected,
                attach_rejected: diagnostics.attach_rejected,
                read_races: diagnostics.read_races,
                decode_rejected: diagnostics.decode_rejected,
                reclaimed_endpoints: diagnostics.reclaimed_endpoints,
                skipped_backlog: diagnostics.skipped_backlog,
            };
        }
        NavigationTransportDiagnosticsRepr {
            header: WireHeader::encode(value.header()),
            process_epoch: value.process_epoch(),
            command_id: value.command_id().map_or([0; 16], |id| *id.as_bytes()),
            channels,
            channel_count: value.channels().len() as u8,
            command_present: u8::from(value.command_id().is_some()),
            reserved: [0; 6],
        }
    }

    fn decode(
        &mut self,
        value: Self::Repr,
    ) -> Result<NavigationTransportDiagnosticsSnapshot, Self::Error> {
        let channel_count = usize::from(value.channel_count);
        if value.reserved != [0; 6]
            || value.command_present > 1
            || channel_count == 0
            || channel_count > MAX_NAVIGATION_TRANSPORT_CHANNELS
            || value.channels[channel_count..]
                .iter()
                .any(|channel| *channel != NavigationTransportChannelRepr::zeroed())
        {
            return Err(NavigationIpcError::TransportDiagnostics(
                "channel count or padding",
            ));
        }
        let command_id = if value.command_present == 1 {
            Some(Id128::new(value.command_id)?)
        } else {
            if value.command_id != [0; 16] {
                return Err(NavigationIpcError::TransportDiagnostics(
                    "absent command contains identity",
                ));
            }
            None
        };
        let mut channels = ArrayVec::new();
        for channel in &value.channels[..channel_count] {
            channels
                .try_push(NavigationTransportChannelSnapshot::new(
                    channel.channel_id.decode::<MAX_ID_LEN>()?,
                    ChannelCounters {
                        published: channel.published,
                        consumed: channel.consumed,
                        overwritten: channel.overwritten,
                        full: channel.full,
                        disconnected: channel.disconnected,
                    },
                    NavigationShmDiagnostics {
                        attach_rejected: channel.attach_rejected,
                        read_races: channel.read_races,
                        decode_rejected: channel.decode_rejected,
                        reclaimed_endpoints: channel.reclaimed_endpoints,
                        skipped_backlog: channel.skipped_backlog,
                    },
                ))
                .map_err(|_| NavigationIpcError::TransportDiagnostics("channel capacity"))?;
        }
        NavigationTransportDiagnosticsSnapshot::new(
            validate_header_at(value.header.decode()?, (self.now)())?,
            value.process_epoch,
            command_id,
            channels,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use message_model::{ClockState, ClockStatus, DataQuality, HeaderFields, Id128};

    const NOW: u64 = 100;

    fn header() -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("rs-navigation-e2e").unwrap(),
            boot_id: Id128::new([7; 16]).unwrap(),
            sequence: 3,
            source_monotonic_ns: NOW,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 50,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    fn snapshot() -> NavigationTransportDiagnosticsSnapshot {
        let mut channels = ArrayVec::new();
        channels.push(NavigationTransportChannelSnapshot::new(
            AsciiId::parse("navigation.authority.v1").unwrap(),
            ChannelCounters {
                published: 10,
                consumed: 9,
                overwritten: 0,
                full: 1,
                disconnected: 2,
            },
            NavigationShmDiagnostics {
                attach_rejected: 3,
                read_races: 4,
                decode_rejected: 5,
                reclaimed_endpoints: 6,
                skipped_backlog: 7,
            },
        ));
        NavigationTransportDiagnosticsSnapshot::new(
            header(),
            8,
            Some(Id128::new([9; 16]).unwrap()),
            channels,
        )
        .unwrap()
    }

    #[test]
    fn codec_round_trips_and_rejects_stale_or_padded_values() {
        let snapshot = snapshot();
        let mut codec = NavigationTransportDiagnosticsCodec::new(|| NOW + 1);
        let repr = codec.encode(&snapshot);
        assert_eq!(codec.decode(repr).unwrap(), snapshot);
        assert!(
            NavigationTransportDiagnosticsCodec::new(|| NOW + 50)
                .decode(repr)
                .is_err()
        );

        let mut padded = repr;
        padded.reserved[0] = 1;
        assert!(codec.decode(padded).is_err());
    }

    #[test]
    fn constructor_rejects_empty_and_duplicate_channel_sets() {
        assert!(
            NavigationTransportDiagnosticsSnapshot::new(header(), 8, None, ArrayVec::new())
                .is_err()
        );
        let channel = snapshot().channels()[0];
        let mut duplicate = ArrayVec::new();
        duplicate.push(channel);
        duplicate.push(channel);
        assert!(NavigationTransportDiagnosticsSnapshot::new(header(), 8, None, duplicate).is_err());
    }
}
