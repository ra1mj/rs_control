//! Validated message values used by the first ROS mapping profile.

use arrayvec::ArrayVec;

use crate::{
    AsciiId, Covariance3, Covariance6, FrameId, Header, Id128, MAX_ID_LEN, Pose3, SemanticError,
    Twist3, UnitQuaternion, Vector3, numeric::require_finite,
};

/// Maximum joints in one time-aligned aggregation.
pub const MAX_JOINTS: usize = 64;
/// Maximum transforms in one batch.
pub const MAX_TRANSFORMS: usize = 64;
/// Maximum faults attached to one component snapshot.
pub const MAX_COMPONENT_FAULTS: usize = 32;
/// Maximum components in one health query/response.
pub const MAX_HEALTH_COMPONENTS: usize = 64;

/// Covariance availability for a present IMU measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CovarianceStatus {
    /// Measurement exists but covariance is unknown; projects to all zeros.
    Unknown,
    /// Validated covariance is available.
    Known(Covariance3),
}

/// Independent IMU measurement availability.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ImuMeasurement<T> {
    /// Measurement is absent; projects to the ROS first-element-minus-one sentinel.
    Unavailable,
    /// Measurement and covariance status are present.
    Present {
        /// Validated measured value.
        value: T,
        /// Known or unknown covariance.
        covariance: CovarianceStatus,
    },
}

/// Validated IMU sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuState {
    header: Header,
    orientation: ImuMeasurement<UnitQuaternion>,
    angular_velocity: ImuMeasurement<Vector3>,
    linear_acceleration: ImuMeasurement<Vector3>,
}

impl ImuState {
    /// Constructs an IMU sample and requires an explicit portable frame.
    pub fn new(
        header: Header,
        orientation: ImuMeasurement<UnitQuaternion>,
        angular_velocity: ImuMeasurement<Vector3>,
        linear_acceleration: ImuMeasurement<Vector3>,
    ) -> Result<Self, SemanticError> {
        require_frame(header, "imu.header.frame_id")?;
        Ok(Self {
            header,
            orientation,
            angular_velocity,
            linear_acceleration,
        })
    }

    /// Returns common metadata.
    #[must_use]
    pub const fn header(self) -> Header {
        self.header
    }

    /// Returns orientation availability/value.
    #[must_use]
    pub const fn orientation(self) -> ImuMeasurement<UnitQuaternion> {
        self.orientation
    }

    /// Returns angular-velocity availability/value.
    #[must_use]
    pub const fn angular_velocity(self) -> ImuMeasurement<Vector3> {
        self.angular_velocity
    }

    /// Returns linear-acceleration availability/value.
    #[must_use]
    pub const fn linear_acceleration(self) -> ImuMeasurement<Vector3> {
        self.linear_acceleration
    }
}

/// Validated joint sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointState {
    resource_id: AsciiId<MAX_ID_LEN>,
    position_rad: f64,
    velocity_radps: f64,
    effort_nm: f64,
}

impl JointState {
    /// Constructs a finite joint sample.
    pub fn new(
        resource_id: AsciiId<MAX_ID_LEN>,
        position_rad: f64,
        velocity_radps: f64,
        effort_nm: f64,
    ) -> Result<Self, SemanticError> {
        require_finite(position_rad, "joint.position_rad")?;
        require_finite(velocity_radps, "joint.velocity_radps")?;
        require_finite(effort_nm, "joint.effort_nm")?;
        Ok(Self {
            resource_id,
            position_rad,
            velocity_radps,
            effort_nm,
        })
    }

    /// Returns the logical joint identifier.
    #[must_use]
    pub const fn resource_id(self) -> AsciiId<MAX_ID_LEN> {
        self.resource_id
    }

    /// Returns position in radians.
    #[must_use]
    pub const fn position_rad(self) -> f64 {
        self.position_rad
    }

    /// Returns velocity in radians per second.
    #[must_use]
    pub const fn velocity_radps(self) -> f64 {
        self.velocity_radps
    }

    /// Returns effort in newton metres.
    #[must_use]
    pub const fn effort_nm(self) -> f64 {
        self.effort_nm
    }
}

/// Time-aligned joint aggregation.
#[derive(Clone, Debug, PartialEq)]
pub struct JointStateAggregate {
    header: Header,
    joints: ArrayVec<JointState, MAX_JOINTS>,
}

impl JointStateAggregate {
    /// Constructs a nonempty, duplicate-free bounded aggregation.
    pub fn new(
        header: Header,
        joints: ArrayVec<JointState, MAX_JOINTS>,
    ) -> Result<Self, SemanticError> {
        if joints.is_empty() {
            return Err(SemanticError::EmptyCollection { field: "joints" });
        }
        for (index, joint) in joints.iter().enumerate() {
            if joints[..index]
                .iter()
                .any(|existing| existing.resource_id == joint.resource_id)
            {
                return Err(SemanticError::DuplicateIdentifier { field: "joints" });
            }
        }
        Ok(Self { header, joints })
    }

    /// Returns common metadata.
    #[must_use]
    pub const fn header(&self) -> Header {
        self.header
    }

    /// Returns the bounded joint sequence.
    #[must_use]
    pub fn joints(&self) -> &[JointState] {
        &self.joints
    }
}

/// Canonical 3D odometry estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OdometryState {
    header: Header,
    parent_frame_id: FrameId,
    child_frame_id: FrameId,
    pose: Pose3,
    twist: Twist3,
    pose_covariance: Covariance6,
    twist_covariance: Covariance6,
}

/// Fields supplied to [`OdometryState::new`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OdometryStateFields {
    /// Common sample metadata.
    pub header: Header,
    /// Parent frame, normally odom.
    pub parent_frame_id: FrameId,
    /// Child frame, normally base_link.
    pub child_frame_id: FrameId,
    /// Child pose in the parent frame.
    pub pose: Pose3,
    /// Child velocity.
    pub twist: Twist3,
    /// Pose covariance.
    pub pose_covariance: Covariance6,
    /// Twist covariance.
    pub twist_covariance: Covariance6,
}

impl OdometryState {
    /// Constructs odometry with explicit, non-self frames.
    pub fn new(fields: OdometryStateFields) -> Result<Self, SemanticError> {
        require_header_parent(fields.header, fields.parent_frame_id, "odometry")?;
        require_distinct_frames(fields.parent_frame_id, fields.child_frame_id)?;
        Ok(Self {
            header: fields.header,
            parent_frame_id: fields.parent_frame_id,
            child_frame_id: fields.child_frame_id,
            pose: fields.pose,
            twist: fields.twist,
            pose_covariance: fields.pose_covariance,
            twist_covariance: fields.twist_covariance,
        })
    }

    /// Returns common metadata.
    #[must_use]
    pub const fn header(self) -> Header {
        self.header
    }

    /// Returns the parent frame.
    #[must_use]
    pub const fn parent_frame_id(self) -> FrameId {
        self.parent_frame_id
    }

    /// Returns the child frame.
    #[must_use]
    pub const fn child_frame_id(self) -> FrameId {
        self.child_frame_id
    }

    /// Returns the pose.
    #[must_use]
    pub const fn pose(self) -> Pose3 {
        self.pose
    }

    /// Returns the twist.
    #[must_use]
    pub const fn twist(self) -> Twist3 {
        self.twist
    }

    /// Returns pose covariance.
    #[must_use]
    pub const fn pose_covariance(self) -> Covariance6 {
        self.pose_covariance
    }

    /// Returns twist covariance.
    #[must_use]
    pub const fn twist_covariance(self) -> Covariance6 {
        self.twist_covariance
    }
}

/// Validated stamped transform.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StampedTransform {
    header: Header,
    parent_frame_id: FrameId,
    child_frame_id: FrameId,
    translation_m: Vector3,
    rotation: UnitQuaternion,
    is_static: bool,
}

/// Fields supplied to [`StampedTransform::new`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StampedTransformFields {
    /// Common timestamp and provenance.
    pub header: Header,
    /// Parent coordinate frame.
    pub parent_frame_id: FrameId,
    /// Child coordinate frame.
    pub child_frame_id: FrameId,
    /// Translation in metres.
    pub translation_m: Vector3,
    /// Unit rotation.
    pub rotation: UnitQuaternion,
    /// Static/dynamic authority class.
    pub is_static: bool,
}

impl StampedTransform {
    /// Constructs a transform with consistent header/parent frames.
    pub fn new(fields: StampedTransformFields) -> Result<Self, SemanticError> {
        require_header_parent(fields.header, fields.parent_frame_id, "transform")?;
        require_distinct_frames(fields.parent_frame_id, fields.child_frame_id)?;
        Ok(Self {
            header: fields.header,
            parent_frame_id: fields.parent_frame_id,
            child_frame_id: fields.child_frame_id,
            translation_m: fields.translation_m,
            rotation: fields.rotation,
            is_static: fields.is_static,
        })
    }

    /// Returns common metadata.
    #[must_use]
    pub const fn header(self) -> Header {
        self.header
    }

    /// Returns the parent frame.
    #[must_use]
    pub const fn parent_frame_id(self) -> FrameId {
        self.parent_frame_id
    }

    /// Returns the child frame.
    #[must_use]
    pub const fn child_frame_id(self) -> FrameId {
        self.child_frame_id
    }

    /// Returns translation in metres.
    #[must_use]
    pub const fn translation_m(self) -> Vector3 {
        self.translation_m
    }

    /// Returns the unit rotation.
    #[must_use]
    pub const fn rotation(self) -> UnitQuaternion {
        self.rotation
    }

    /// Returns whether this transform is static.
    #[must_use]
    pub const fn is_static(self) -> bool {
        self.is_static
    }
}

/// Bounded static or dynamic transform aggregation.
#[derive(Clone, Debug, PartialEq)]
pub struct TransformBatch(ArrayVec<StampedTransform, MAX_TRANSFORMS>);

impl TransformBatch {
    /// Constructs a nonempty batch with one authority class and unique children.
    pub fn new(
        transforms: ArrayVec<StampedTransform, MAX_TRANSFORMS>,
    ) -> Result<Self, SemanticError> {
        let Some(first) = transforms.first() else {
            return Err(SemanticError::EmptyCollection {
                field: "transforms",
            });
        };
        for (index, transform) in transforms.iter().enumerate() {
            if transform.is_static != first.is_static {
                return Err(SemanticError::MixedTransformAuthority);
            }
            if transforms[..index]
                .iter()
                .any(|existing| existing.child_frame_id == transform.child_frame_id)
            {
                return Err(SemanticError::DuplicateIdentifier {
                    field: "transform.child_frame_id",
                });
            }
        }
        Ok(Self(transforms))
    }

    /// Returns transforms in deterministic input order.
    #[must_use]
    pub fn transforms(&self) -> &[StampedTransform] {
        &self.0
    }
}

/// Component lifecycle exposed to health consumers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleState {
    /// Starting but not ready.
    Starting,
    /// Ready but inactive.
    Ready,
    /// Actively serving its contract.
    Active,
    /// Serving under an explicit degraded policy.
    Degraded,
    /// Faulted and unavailable for normal use.
    Faulted,
    /// Stopping.
    Stopping,
}

/// Safety significance of a fault.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaultSeverity {
    /// Informational condition.
    Info,
    /// Warning condition.
    Warning,
    /// Error condition.
    Error,
    /// Critical safety condition.
    Critical,
}

/// Recovery policy for a fault.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FaultRecoverability {
    /// Automatic recovery is permitted.
    Automatic,
    /// Operator action is required.
    Operator,
    /// Power cycling is required.
    PowerCycle,
    /// Recovery is not supported.
    NotRecoverable,
}

/// Validated typed component fault.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Fault {
    /// Stable component-owned code.
    pub code: u32,
    /// Safety significance.
    pub severity: FaultSeverity,
    /// Whether the fault latches.
    pub latched: bool,
    /// Recovery classification.
    pub recoverability: FaultRecoverability,
    /// First observation in source monotonic time.
    pub first_seen_monotonic_ns: u64,
    /// Latest observation in source monotonic time.
    pub last_seen_monotonic_ns: u64,
    /// Observation count.
    pub count: u64,
    /// Faulting component/subcomponent.
    pub fault_source: AsciiId<MAX_ID_LEN>,
    /// Optional related command.
    pub related_command_id: Option<Id128>,
    /// Stable recommended-action code.
    pub recommended_action: u32,
}

impl Fault {
    /// Validates time ordering and a nonzero occurrence count.
    pub fn validate(self) -> Result<Self, SemanticError> {
        if self.count == 0 {
            return Err(SemanticError::ZeroCount {
                field: "fault.count",
            });
        }
        if self.last_seen_monotonic_ns < self.first_seen_monotonic_ns {
            return Err(SemanticError::InvalidTimeOrder { field: "fault" });
        }
        Ok(self)
    }
}

/// Validated component health snapshot.
#[derive(Clone, Debug, PartialEq)]
pub struct ComponentHealth {
    /// Common metadata.
    pub header: Header,
    /// Stable component identifier.
    pub component_id: AsciiId<MAX_ID_LEN>,
    /// Current lifecycle.
    pub lifecycle: LifecycleState,
    /// Bounded active fault set.
    pub faults: ArrayVec<Fault, MAX_COMPONENT_FAULTS>,
    /// Last successful I/O time in source monotonic time.
    pub last_success_monotonic_ns: u64,
}

/// Bounded health snapshot request.
#[derive(Clone, Debug, PartialEq)]
pub struct HealthSnapshotRequest {
    /// Common request metadata.
    pub header: Header,
    /// Empty means all components within the bound.
    pub component_ids: ArrayVec<AsciiId<MAX_ID_LEN>, MAX_HEALTH_COMPONENTS>,
    /// Nonzero requested response bound.
    pub max_components: usize,
}

impl HealthSnapshotRequest {
    /// Validates the response bound and duplicate-free component allowlist.
    pub fn validate(self) -> Result<Self, SemanticError> {
        if self.max_components == 0 || self.max_components > MAX_HEALTH_COMPONENTS {
            return Err(SemanticError::InvalidBound {
                field: "max_components",
                maximum: MAX_HEALTH_COMPONENTS,
            });
        }
        for (index, component) in self.component_ids.iter().enumerate() {
            if self.component_ids[..index].contains(component) {
                return Err(SemanticError::DuplicateIdentifier {
                    field: "component_ids",
                });
            }
        }
        Ok(self)
    }
}

/// Bounded health snapshot response.
#[derive(Clone, Debug, PartialEq)]
pub struct HealthSnapshotResponse {
    /// Common response metadata.
    pub header: Header,
    /// Component snapshots.
    pub components: ArrayVec<ComponentHealth, MAX_HEALTH_COMPONENTS>,
    /// Whether more components existed than the request bound.
    pub truncated: bool,
}

fn require_frame(header: Header, field: &'static str) -> Result<FrameId, SemanticError> {
    header
        .frame_id()
        .ok_or(SemanticError::MissingFrame { field })
}

fn require_header_parent(
    header: Header,
    parent: FrameId,
    field: &'static str,
) -> Result<(), SemanticError> {
    let header_frame = require_frame(header, field)?;
    if header_frame != parent {
        return Err(SemanticError::FrameMismatch { field });
    }
    Ok(())
}

fn require_distinct_frames(parent: FrameId, child: FrameId) -> Result<(), SemanticError> {
    if parent == child {
        Err(SemanticError::SelfFrame)
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use arrayvec::ArrayVec;

    use super::{CovarianceStatus, ImuMeasurement, ImuState, JointState, JointStateAggregate};
    use crate::{
        AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128, UnitQuaternion,
        Vector3,
    };

    fn header(frame: Option<&str>) -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("sensor").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 1,
            source_monotonic_ns: 10,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 100,
            frame_id: frame.map(|value| AsciiId::parse(value).unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    #[test]
    fn imu_requires_frame_and_preserves_independent_availability() {
        let present = ImuMeasurement::Present {
            value: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
            covariance: CovarianceStatus::Unknown,
        };
        assert!(
            ImuState::new(
                header(None),
                present,
                ImuMeasurement::Unavailable,
                ImuMeasurement::Unavailable
            )
            .is_err()
        );
        assert!(
            ImuState::new(
                header(Some("imu_link")),
                present,
                ImuMeasurement::Unavailable,
                ImuMeasurement::Unavailable
            )
            .is_ok()
        );
    }

    #[test]
    fn joint_aggregation_rejects_duplicates() {
        let joint = JointState::new(AsciiId::parse("wheel").unwrap(), 0.0, 0.0, 0.0).unwrap();
        let mut joints = ArrayVec::new();
        joints.push(joint);
        joints.push(joint);
        assert!(JointStateAggregate::new(header(None), joints).is_err());
        assert!(Vector3::new(0.0, 0.0, 0.0).is_ok());
    }
}
