use bytemuck::{Pod, Zeroable};
use message_model::{
    AcquisitionTimestamp, AsciiId, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    DataQuality, Header, HeaderFields, Id128, MAX_ID_LEN, Pose3, ResourceLease,
    ResourceLeaseFields, TimestampDomain, Twist3, UnitQuaternion, Vector3,
};
use rustix::time::{ClockId, clock_gettime};

use crate::NavigationIpcError;

pub(crate) type NowFn = fn() -> u64;

pub(crate) fn monotonic_now_ns() -> u64 {
    let time = clock_gettime(ClockId::Monotonic);
    u64::try_from(time.tv_sec)
        .unwrap_or(u64::MAX)
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::try_from(time.tv_nsec).unwrap_or(0))
}

pub(crate) fn validate_header_at(
    header: Header,
    now_ns: u64,
) -> Result<Header, NavigationIpcError> {
    let deadline_ns =
        header
            .source_expiration_ns()
            .ok_or(NavigationIpcError::InvalidRepresentation(
                "header validity overflow",
            ))?;
    if now_ns >= deadline_ns {
        return Err(NavigationIpcError::Expired {
            deadline_ns,
            now_ns,
        });
    }
    Ok(header)
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
pub(crate) struct WireAsciiId {
    bytes: [u8; MAX_ID_LEN],
    len: u8,
    reserved: [u8; 7],
}

impl WireAsciiId {
    pub(crate) fn encode<const N: usize>(value: AsciiId<N>) -> Self {
        let source = value.as_str().as_bytes();
        let mut bytes = [0; MAX_ID_LEN];
        bytes[..source.len()].copy_from_slice(source);
        Self {
            bytes,
            len: source.len() as u8,
            reserved: [0; 7],
        }
    }

    pub(crate) fn decode<const N: usize>(self) -> Result<AsciiId<N>, NavigationIpcError> {
        let len = usize::from(self.len);
        if len == 0
            || len > N
            || len > MAX_ID_LEN
            || self.reserved != [0; 7]
            || self.bytes[len..].iter().any(|byte| *byte != 0)
        {
            return Err(NavigationIpcError::InvalidRepresentation(
                "identifier length or padding",
            ));
        }
        let value = std::str::from_utf8(&self.bytes[..len])
            .map_err(|_| NavigationIpcError::InvalidRepresentation("identifier is not ASCII"))?;
        Ok(AsciiId::parse(value)?)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
pub(crate) struct WireHeader {
    source_id: WireAsciiId,
    frame_id: WireAsciiId,
    boot_id: [u8; 16],
    sequence: u64,
    source_monotonic_ns: u64,
    publish_unix_ns: u64,
    valid_for_ns: u64,
    acquisition_timestamp_ns: u64,
    acquisition_uncertainty_ns: u64,
    clock_uncertainty_ns: u64,
    flags: u32,
    acquisition_domain: u8,
    clock_state: u8,
    quality: u8,
    reserved: u8,
}

impl WireHeader {
    const HAS_PUBLISH_UNIX: u32 = 1 << 0;
    const HAS_ACQUISITION: u32 = 1 << 1;
    const HAS_FRAME: u32 = 1 << 2;
    const KNOWN_FLAGS: u32 = Self::HAS_PUBLISH_UNIX | Self::HAS_ACQUISITION | Self::HAS_FRAME;

    pub(crate) fn encode(value: Header) -> Self {
        let mut flags = 0;
        let publish_unix_ns = value.publish_unix_ns().map_or(0, |timestamp| {
            flags |= Self::HAS_PUBLISH_UNIX;
            timestamp
        });
        let (acquisition_timestamp_ns, acquisition_domain, acquisition_uncertainty_ns) = value
            .acquisition_timestamp()
            .map_or((0, 0, 0), |timestamp| {
                flags |= Self::HAS_ACQUISITION;
                (
                    timestamp.timestamp_ns,
                    encode_timestamp_domain(timestamp.domain),
                    timestamp.uncertainty_ns,
                )
            });
        let frame_id = value.frame_id().map_or_else(WireAsciiId::zeroed, |frame| {
            flags |= Self::HAS_FRAME;
            WireAsciiId::encode(frame)
        });
        Self {
            source_id: WireAsciiId::encode(value.source_id()),
            frame_id,
            boot_id: *value.boot_id().as_bytes(),
            sequence: value.sequence(),
            source_monotonic_ns: value.source_monotonic_ns(),
            publish_unix_ns,
            valid_for_ns: value.valid_for_ns(),
            acquisition_timestamp_ns,
            acquisition_uncertainty_ns,
            clock_uncertainty_ns: value.clock_status().uncertainty_ns,
            flags,
            acquisition_domain,
            clock_state: encode_clock_state(value.clock_status().state),
            quality: encode_quality(value.quality()),
            reserved: 0,
        }
    }

    pub(crate) fn decode(self) -> Result<Header, NavigationIpcError> {
        if self.flags & !Self::KNOWN_FLAGS != 0 || self.reserved != 0 {
            return Err(NavigationIpcError::InvalidRepresentation(
                "header flags or padding",
            ));
        }
        let publish_unix_ns = optional_u64(
            self.flags & Self::HAS_PUBLISH_UNIX != 0,
            self.publish_unix_ns,
            "publish timestamp",
        )?;
        let acquisition_timestamp = if self.flags & Self::HAS_ACQUISITION != 0 {
            Some(AcquisitionTimestamp {
                timestamp_ns: self.acquisition_timestamp_ns,
                domain: decode_timestamp_domain(self.acquisition_domain)?,
                uncertainty_ns: self.acquisition_uncertainty_ns,
            })
        } else {
            if self.acquisition_timestamp_ns != 0
                || self.acquisition_domain != 0
                || self.acquisition_uncertainty_ns != 0
            {
                return Err(NavigationIpcError::InvalidRepresentation(
                    "absent acquisition timestamp contains data",
                ));
            }
            None
        };
        let frame_id = if self.flags & Self::HAS_FRAME != 0 {
            Some(self.frame_id.decode::<MAX_ID_LEN>()?)
        } else {
            if self.frame_id != WireAsciiId::zeroed() {
                return Err(NavigationIpcError::InvalidRepresentation(
                    "absent frame contains data",
                ));
            }
            None
        };
        Ok(Header::new(HeaderFields {
            source_id: self.source_id.decode::<MAX_ID_LEN>()?,
            boot_id: Id128::new(self.boot_id)?,
            sequence: self.sequence,
            source_monotonic_ns: self.source_monotonic_ns,
            publish_unix_ns,
            acquisition_timestamp,
            valid_for_ns: self.valid_for_ns,
            frame_id,
            clock_status: ClockStatus {
                state: decode_clock_state(self.clock_state)?,
                uncertainty_ns: self.clock_uncertainty_ns,
            },
            quality: decode_quality(self.quality)?,
        })?)
    }
}

fn optional_u64(
    present: bool,
    value: u64,
    name: &'static str,
) -> Result<Option<u64>, NavigationIpcError> {
    if present {
        Ok(Some(value))
    } else if value == 0 {
        Ok(None)
    } else {
        Err(NavigationIpcError::InvalidRepresentation(name))
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
pub(crate) struct WirePose3 {
    values: [f64; 7],
}

impl WirePose3 {
    pub(crate) fn encode(value: Pose3) -> Self {
        let mut values = [0.0; 7];
        values[..3].copy_from_slice(&[
            value.position_m.x(),
            value.position_m.y(),
            value.position_m.z(),
        ]);
        values[3..].copy_from_slice(&value.orientation.components());
        Self { values }
    }

    pub(crate) fn decode(self) -> Result<Pose3, NavigationIpcError> {
        Ok(Pose3 {
            position_m: Vector3::new(self.values[0], self.values[1], self.values[2])?,
            orientation: UnitQuaternion::new(
                self.values[3],
                self.values[4],
                self.values[5],
                self.values[6],
            )?,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
pub(crate) struct WireTwist3 {
    values: [f64; 6],
}

impl WireTwist3 {
    pub(crate) fn encode(value: Twist3) -> Self {
        Self {
            values: [
                value.linear_mps.x(),
                value.linear_mps.y(),
                value.linear_mps.z(),
                value.angular_radps.x(),
                value.angular_radps.y(),
                value.angular_radps.z(),
            ],
        }
    }

    pub(crate) fn decode(self) -> Result<Twist3, NavigationIpcError> {
        Ok(Twist3 {
            linear_mps: Vector3::new(self.values[0], self.values[1], self.values[2])?,
            angular_radps: Vector3::new(self.values[3], self.values[4], self.values[5])?,
        })
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
pub(crate) struct WireLease {
    resource_id: WireAsciiId,
    owner_id: WireAsciiId,
    lease_id: [u8; 16],
    session_epoch: u64,
    expires_monotonic_ns: u64,
}

impl WireLease {
    pub(crate) fn encode(value: ResourceLease) -> Self {
        Self {
            resource_id: WireAsciiId::encode(value.resource_id()),
            owner_id: WireAsciiId::encode(value.owner_id()),
            lease_id: *value.lease_id().as_bytes(),
            session_epoch: value.session_epoch(),
            expires_monotonic_ns: value.expires_monotonic_ns(),
        }
    }

    pub(crate) fn decode(self) -> Result<ResourceLease, NavigationIpcError> {
        Ok(ResourceLease::new(ResourceLeaseFields {
            resource_id: self.resource_id.decode::<MAX_ID_LEN>()?,
            owner_id: self.owner_id.decode::<MAX_ID_LEN>()?,
            lease_id: Id128::new(self.lease_id)?,
            session_epoch: self.session_epoch,
            expires_monotonic_ns: self.expires_monotonic_ns,
        })?)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
pub(crate) struct WireCommandIntent {
    header: WireHeader,
    resource_id: WireAsciiId,
    owner_id: WireAsciiId,
    lease_id: [u8; 16],
    command_id: [u8; 16],
    session_epoch: u64,
    requested_ttl_ns: u64,
}

impl WireCommandIntent {
    pub(crate) fn encode(value: CommandIntent) -> Self {
        Self {
            header: WireHeader::encode(value.header()),
            resource_id: WireAsciiId::encode(value.resource_id()),
            owner_id: WireAsciiId::encode(value.owner_id()),
            lease_id: *value.lease_id().as_bytes(),
            command_id: *value.command_id().as_bytes(),
            session_epoch: value.session_epoch(),
            requested_ttl_ns: value.requested_ttl_ns(),
        }
    }

    pub(crate) fn decode(self) -> Result<CommandIntent, NavigationIpcError> {
        Ok(CommandIntent::new(CommandIntentFields {
            header: self.header.decode()?,
            resource_id: self.resource_id.decode::<MAX_ID_LEN>()?,
            owner_id: self.owner_id.decode::<MAX_ID_LEN>()?,
            lease_id: Id128::new(self.lease_id)?,
            session_epoch: self.session_epoch,
            command_id: Id128::new(self.command_id)?,
            requested_ttl_ns: self.requested_ttl_ns,
        })?)
    }
}

fn encode_clock_state(value: ClockState) -> u8 {
    match value {
        ClockState::Local => 1,
        ClockState::Synchronized => 2,
        ClockState::Unsynchronized => 3,
    }
}

fn decode_clock_state(value: u8) -> Result<ClockState, NavigationIpcError> {
    match value {
        1 => Ok(ClockState::Local),
        2 => Ok(ClockState::Synchronized),
        3 => Ok(ClockState::Unsynchronized),
        _ => Err(NavigationIpcError::InvalidRepresentation("clock state")),
    }
}

fn encode_timestamp_domain(value: TimestampDomain) -> u8 {
    match value {
        TimestampDomain::Unix => 1,
        TimestampDomain::Ros => 2,
    }
}

fn decode_timestamp_domain(value: u8) -> Result<TimestampDomain, NavigationIpcError> {
    match value {
        1 => Ok(TimestampDomain::Unix),
        2 => Ok(TimestampDomain::Ros),
        _ => Err(NavigationIpcError::InvalidRepresentation(
            "acquisition timestamp domain",
        )),
    }
}

fn encode_quality(value: DataQuality) -> u8 {
    match value {
        DataQuality::Valid => 1,
        DataQuality::Degraded => 2,
        DataQuality::Stale => 3,
        DataQuality::Calibrating => 4,
        DataQuality::Faulted => 5,
    }
}

fn decode_quality(value: u8) -> Result<DataQuality, NavigationIpcError> {
    match value {
        1 => Ok(DataQuality::Valid),
        2 => Ok(DataQuality::Degraded),
        3 => Ok(DataQuality::Stale),
        4 => Ok(DataQuality::Calibrating),
        5 => Ok(DataQuality::Faulted),
        _ => Err(NavigationIpcError::InvalidRepresentation("data quality")),
    }
}

pub(crate) fn encode_bool(value: bool) -> u8 {
    u8::from(value)
}

pub(crate) fn decode_bool(value: u8) -> Result<bool, NavigationIpcError> {
    match value {
        0 => Ok(false),
        1 => Ok(true),
        _ => Err(NavigationIpcError::InvalidRepresentation("boolean")),
    }
}
