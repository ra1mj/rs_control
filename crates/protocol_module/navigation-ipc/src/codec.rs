use arrayvec::ArrayVec;
use bytemuck::{Pod, Zeroable};
use message_model::{
    ChassisSetpointIntent, Covariance6, Id128, MAX_NAVIGATION_FAULTS, NavigationAuthority,
    NavigationAuthorityState, NavigationAuthorityUnavailable, NavigationComponentState,
    NavigationCounters, NavigationHealth, NavigationReadiness, OdometryState, OdometryStateFields,
    Sha256Identity, StampedTransform, StampedTransformFields, TransformBatch, Twist2,
    UnitQuaternion, Vector3,
};
use middleware_shm::{ShmCodec, ShmPayload};

use crate::{
    NavigationIpcError,
    wire::{
        NowFn, WireAsciiId, WireCommandIntent, WireHeader, WireLease, WirePose3, WireTwist3,
        decode_bool, encode_bool, monotonic_now_ns, validate_header_at,
    },
};

/// Authority codec using the host monotonic clock by default.
#[derive(Clone, Copy)]
pub struct NavigationAuthorityCodec {
    now: NowFn,
}

impl NavigationAuthorityCodec {
    /// Uses a caller-supplied local monotonic time source.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationAuthorityCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationAuthorityRepr {
    header: WireHeader,
    lease: WireLease,
    active_command_id: [u8; 16],
    tag: u8,
    unavailable_reason: u8,
    active_command_present: u8,
    reserved: [u8; 5],
}

impl ShmPayload for NavigationAuthorityRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-auth-v001";
    const SCHEMA_VERSION: u32 = 1;
}

impl ShmCodec<NavigationAuthorityState> for NavigationAuthorityCodec {
    type Repr = NavigationAuthorityRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationAuthorityState) -> Self::Repr {
        let (lease, active_command_id, tag, unavailable_reason, active_command_present) =
            match value.authority {
                NavigationAuthority::Unavailable(reason) => (
                    WireLease::zeroed(),
                    [0; 16],
                    1,
                    encode_unavailable(reason),
                    0,
                ),
                NavigationAuthority::Available {
                    lease,
                    active_command_id,
                } => (
                    WireLease::encode(lease),
                    active_command_id.map_or([0; 16], |id| *id.as_bytes()),
                    2,
                    0,
                    u8::from(active_command_id.is_some()),
                ),
            };
        NavigationAuthorityRepr {
            header: WireHeader::encode(value.header),
            lease,
            active_command_id,
            tag,
            unavailable_reason,
            active_command_present,
            reserved: [0; 5],
        }
    }

    fn decode(&mut self, value: Self::Repr) -> Result<NavigationAuthorityState, Self::Error> {
        if value.reserved != [0; 5] {
            return Err(NavigationIpcError::InvalidRepresentation(
                "authority padding",
            ));
        }
        let now = (self.now)();
        let header = validate_header_at(value.header.decode()?, now)?;
        let authority = match value.tag {
            1 => {
                if value.lease != WireLease::zeroed()
                    || value.active_command_id != [0; 16]
                    || value.active_command_present != 0
                {
                    return Err(NavigationIpcError::InvalidRepresentation(
                        "unavailable authority contains lease state",
                    ));
                }
                NavigationAuthority::Unavailable(decode_unavailable(value.unavailable_reason)?)
            }
            2 => {
                if value.unavailable_reason != 0 || value.active_command_present > 1 {
                    return Err(NavigationIpcError::InvalidRepresentation(
                        "available authority flags",
                    ));
                }
                let lease = value.lease.decode()?;
                lease.validate_at(now)?;
                let active_command_id = if value.active_command_present == 1 {
                    Some(Id128::new(value.active_command_id)?)
                } else {
                    if value.active_command_id != [0; 16] {
                        return Err(NavigationIpcError::InvalidRepresentation(
                            "absent active command contains identity",
                        ));
                    }
                    None
                };
                NavigationAuthority::Available {
                    lease,
                    active_command_id,
                }
            }
            _ => {
                return Err(NavigationIpcError::InvalidRepresentation(
                    "authority state tag",
                ));
            }
        };
        NavigationAuthorityState { header, authority }
            .validate_at(now)
            .map_err(Into::into)
    }
}

/// Allocation-free fixed representation of a raw controller proposal.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationProposalRepr {
    intent: WireCommandIntent,
    target: [f64; 3],
}

impl ShmPayload for NavigationProposalRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-prop-v001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Raw proposal codec using receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationProposalCodec {
    now: NowFn,
}

impl NavigationProposalCodec {
    /// Uses a caller-supplied local monotonic time source.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationProposalCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<ChassisSetpointIntent> for NavigationProposalCodec {
    type Repr = NavigationProposalRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &ChassisSetpointIntent) -> Self::Repr {
        NavigationProposalRepr {
            intent: WireCommandIntent::encode(value.intent()),
            target: value.target().components(),
        }
    }

    fn decode(&mut self, value: Self::Repr) -> Result<ChassisSetpointIntent, Self::Error> {
        let intent = value.intent.decode()?;
        let _ = validate_header_at(intent.header(), (self.now)())?;
        Ok(ChassisSetpointIntent::new(
            intent,
            Twist2::new(value.target[0], value.target[1], value.target[2])?,
        ))
    }
}

/// Allocation-free fixed representation of semantic odometry.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationOdometryRepr {
    header: WireHeader,
    parent_frame_id: WireAsciiId,
    child_frame_id: WireAsciiId,
    pose: WirePose3,
    twist: WireTwist3,
    pose_covariance: [f64; 36],
    twist_covariance: [f64; 36],
}

impl ShmPayload for NavigationOdometryRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-odom-v001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Semantic odometry codec using receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationOdometryCodec {
    now: NowFn,
}

impl NavigationOdometryCodec {
    /// Uses a caller-supplied local monotonic time source.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationOdometryCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<OdometryState> for NavigationOdometryCodec {
    type Repr = NavigationOdometryRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &OdometryState) -> Self::Repr {
        NavigationOdometryRepr {
            header: WireHeader::encode(value.header()),
            parent_frame_id: WireAsciiId::encode(value.parent_frame_id()),
            child_frame_id: WireAsciiId::encode(value.child_frame_id()),
            pose: WirePose3::encode(value.pose()),
            twist: WireTwist3::encode(value.twist()),
            pose_covariance: *value.pose_covariance().values(),
            twist_covariance: *value.twist_covariance().values(),
        }
    }

    fn decode(&mut self, value: Self::Repr) -> Result<OdometryState, Self::Error> {
        let header = validate_header_at(value.header.decode()?, (self.now)())?;
        Ok(OdometryState::new(OdometryStateFields {
            header,
            parent_frame_id: value.parent_frame_id.decode()?,
            child_frame_id: value.child_frame_id.decode()?,
            pose: value.pose.decode()?,
            twist: value.twist.decode()?,
            pose_covariance: Covariance6::new(value.pose_covariance)?,
            twist_covariance: Covariance6::new(value.twist_covariance)?,
        })?)
    }
}

const NAVIGATION_TF_COUNT: usize = 2;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct WireTransform {
    header: WireHeader,
    parent_frame_id: WireAsciiId,
    child_frame_id: WireAsciiId,
    translation_m: [f64; 3],
    rotation: [f64; 4],
    is_static: u8,
    reserved: [u8; 7],
}

impl WireTransform {
    fn encode(value: StampedTransform) -> Self {
        Self {
            header: WireHeader::encode(value.header()),
            parent_frame_id: WireAsciiId::encode(value.parent_frame_id()),
            child_frame_id: WireAsciiId::encode(value.child_frame_id()),
            translation_m: [
                value.translation_m().x(),
                value.translation_m().y(),
                value.translation_m().z(),
            ],
            rotation: value.rotation().components(),
            is_static: encode_bool(value.is_static()),
            reserved: [0; 7],
        }
    }

    fn decode(self, now: u64) -> Result<StampedTransform, NavigationIpcError> {
        if self.reserved != [0; 7] {
            return Err(NavigationIpcError::InvalidRepresentation("TF padding"));
        }
        Ok(StampedTransform::new(StampedTransformFields {
            header: validate_header_at(self.header.decode()?, now)?,
            parent_frame_id: self.parent_frame_id.decode()?,
            child_frame_id: self.child_frame_id.decode()?,
            translation_m: Vector3::new(
                self.translation_m[0],
                self.translation_m[1],
                self.translation_m[2],
            )?,
            rotation: UnitQuaternion::new(
                self.rotation[0],
                self.rotation[1],
                self.rotation[2],
                self.rotation[3],
            )?,
            is_static: decode_bool(self.is_static)?,
        })?)
    }
}

/// Exact two-edge dynamic navigation TF batch.
#[derive(Clone, Debug, PartialEq)]
pub struct NavigationTfBatch(TransformBatch);

impl NavigationTfBatch {
    /// Requires exactly `map -> odom` then `odom -> base_link`, both dynamic.
    pub fn new(value: TransformBatch) -> Result<Self, NavigationIpcError> {
        let transforms = value.transforms();
        if transforms.len() != NAVIGATION_TF_COUNT {
            return Err(NavigationIpcError::TransformOwnership(
                "expected exactly two transforms",
            ));
        }
        validate_edge(transforms[0], "map", "odom")?;
        validate_edge(transforms[1], "odom", "base_link")?;
        Ok(Self(value))
    }

    /// Returns the backend-neutral batch.
    #[must_use]
    pub const fn batch(&self) -> &TransformBatch {
        &self.0
    }

    /// Consumes the wrapper.
    #[must_use]
    pub fn into_batch(self) -> TransformBatch {
        self.0
    }
}

fn validate_edge(
    transform: StampedTransform,
    parent: &str,
    child: &str,
) -> Result<(), NavigationIpcError> {
    if transform.is_static()
        || transform.parent_frame_id().as_str() != parent
        || transform.child_frame_id().as_str() != child
    {
        return Err(NavigationIpcError::TransformOwnership(
            "edge or authority does not match manifest",
        ));
    }
    Ok(())
}

/// Fixed representation of the exact navigation TF batch.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationTfRepr {
    transforms: [WireTransform; NAVIGATION_TF_COUNT],
}

impl ShmPayload for NavigationTfRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-tf---v001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Dynamic TF codec using receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationTfCodec {
    now: NowFn,
}

impl NavigationTfCodec {
    /// Uses a caller-supplied local monotonic time source.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationTfCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationTfBatch> for NavigationTfCodec {
    type Repr = NavigationTfRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationTfBatch) -> Self::Repr {
        let transforms = value.batch().transforms();
        NavigationTfRepr {
            transforms: [
                WireTransform::encode(transforms[0]),
                WireTransform::encode(transforms[1]),
            ],
        }
    }

    fn decode(&mut self, value: Self::Repr) -> Result<NavigationTfBatch, Self::Error> {
        let now = (self.now)();
        let mut transforms = ArrayVec::new();
        for transform in value.transforms {
            transforms.push(transform.decode(now)?);
        }
        NavigationTfBatch::new(TransformBatch::new(transforms)?)
    }
}

/// Fixed bridge-health representation.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationHealthRepr {
    header: WireHeader,
    process_epoch: u64,
    manifest_sha256: [u8; 32],
    counters: [u64; 6],
    fault_codes: [u32; MAX_NAVIGATION_FAULTS],
    component_state: u8,
    readiness: u8,
    fault_count: u8,
    reserved: [u8; 5],
}

impl ShmPayload for NavigationHealthRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-health001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Health codec using receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationHealthCodec {
    now: NowFn,
}

impl NavigationHealthCodec {
    /// Uses a caller-supplied local monotonic time source.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationHealthCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationHealth> for NavigationHealthCodec {
    type Repr = NavigationHealthRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationHealth) -> Self::Repr {
        let mut fault_codes = [0; MAX_NAVIGATION_FAULTS];
        fault_codes[..value.fault_codes.len()].copy_from_slice(&value.fault_codes);
        NavigationHealthRepr {
            header: WireHeader::encode(value.header),
            process_epoch: value.process_epoch,
            manifest_sha256: value.manifest_sha256.as_bytes(),
            counters: [
                value.counters.accepted_proposals,
                value.counters.authority_rejections,
                value.counters.freshness_rejections,
                value.counters.overwritten_proposals,
                value.counters.accepted_goals,
                value.counters.terminal_results,
            ],
            fault_codes,
            component_state: encode_component_state(value.component_state),
            readiness: encode_readiness(value.readiness),
            fault_count: value.fault_codes.len() as u8,
            reserved: [0; 5],
        }
    }

    fn decode(&mut self, value: Self::Repr) -> Result<NavigationHealth, Self::Error> {
        let fault_count = usize::from(value.fault_count);
        if value.reserved != [0; 5]
            || fault_count > MAX_NAVIGATION_FAULTS
            || value.fault_codes[fault_count..]
                .iter()
                .any(|code| *code != 0)
        {
            return Err(NavigationIpcError::InvalidRepresentation(
                "health fault count or padding",
            ));
        }
        let mut fault_codes = ArrayVec::new();
        fault_codes
            .try_extend_from_slice(&value.fault_codes[..fault_count])
            .map_err(|_| NavigationIpcError::InvalidRepresentation("health fault bound"))?;
        Ok(NavigationHealth::new(
            validate_header_at(value.header.decode()?, (self.now)())?,
            value.process_epoch,
            Sha256Identity::new(value.manifest_sha256, "navigation_health.manifest_sha256")?,
            decode_component_state(value.component_state)?,
            decode_readiness(value.readiness)?,
            fault_codes,
            NavigationCounters {
                accepted_proposals: value.counters[0],
                authority_rejections: value.counters[1],
                freshness_rejections: value.counters[2],
                overwritten_proposals: value.counters[3],
                accepted_goals: value.counters[4],
                terminal_results: value.counters[5],
            },
        )?)
    }
}

fn encode_unavailable(value: NavigationAuthorityUnavailable) -> u8 {
    match value {
        NavigationAuthorityUnavailable::NoLease => 1,
        NavigationAuthorityUnavailable::LeaseExpired => 2,
        NavigationAuthorityUnavailable::MissionInactive => 3,
        NavigationAuthorityUnavailable::PluginUnavailable => 4,
    }
}

fn decode_unavailable(value: u8) -> Result<NavigationAuthorityUnavailable, NavigationIpcError> {
    match value {
        1 => Ok(NavigationAuthorityUnavailable::NoLease),
        2 => Ok(NavigationAuthorityUnavailable::LeaseExpired),
        3 => Ok(NavigationAuthorityUnavailable::MissionInactive),
        4 => Ok(NavigationAuthorityUnavailable::PluginUnavailable),
        _ => Err(NavigationIpcError::InvalidRepresentation(
            "authority unavailable reason",
        )),
    }
}

fn encode_component_state(value: NavigationComponentState) -> u8 {
    match value {
        NavigationComponentState::Stopped => 1,
        NavigationComponentState::Starting => 2,
        NavigationComponentState::Ready => 3,
        NavigationComponentState::Failed => 4,
    }
}

fn decode_component_state(value: u8) -> Result<NavigationComponentState, NavigationIpcError> {
    match value {
        1 => Ok(NavigationComponentState::Stopped),
        2 => Ok(NavigationComponentState::Starting),
        3 => Ok(NavigationComponentState::Ready),
        4 => Ok(NavigationComponentState::Failed),
        _ => Err(NavigationIpcError::InvalidRepresentation(
            "navigation component state",
        )),
    }
}

fn encode_readiness(value: NavigationReadiness) -> u8 {
    encode_bool(value.manifest)
        | (encode_bool(value.action_server) << 1)
        | (encode_bool(value.nav2_lifecycle) << 2)
        | (encode_bool(value.map) << 3)
        | (encode_bool(value.localization) << 4)
        | (encode_bool(value.ipc) << 5)
}

fn decode_readiness(value: u8) -> Result<NavigationReadiness, NavigationIpcError> {
    if value & !0x3f != 0 {
        return Err(NavigationIpcError::InvalidRepresentation(
            "navigation readiness bits",
        ));
    }
    Ok(NavigationReadiness {
        manifest: value & 1 != 0,
        action_server: value & 2 != 0,
        nav2_lifecycle: value & 4 != 0,
        map: value & 8 != 0,
        localization: value & 16 != 0,
        ipc: value & 32 != 0,
    })
}
