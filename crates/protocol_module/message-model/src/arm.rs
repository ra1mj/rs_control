//! Project-owned semantic contracts for multi-morphology mechanical arms.
//!
//! These types describe SI-valued observations and control proposals. They do
//! not contain a transport, motor address, protocol opcode, or wire encoding.

use arrayvec::ArrayVec;
use thiserror::Error;

use crate::{
    AsciiId, Header, MAX_ID_LEN, MAX_JOINTS, MAX_TRAJECTORY_POINTS, SemanticError,
    numeric::require_finite,
};

/// Stable kinematic-group identifier.
pub type KinematicGroupId = AsciiId<MAX_ID_LEN>;
/// Stable project-owned joint identifier.
pub type ArmJointId = AsciiId<MAX_ID_LEN>;
/// Stable logical rotary-actuator identifier.
pub type ArmActuatorId = AsciiId<MAX_ID_LEN>;

/// Non-zero identity of one compiled arm profile.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ArmProfileId([u8; 32]);

impl ArmProfileId {
    /// Constructs an identity from a non-zero SHA-256 digest.
    pub fn new(bytes: [u8; 32]) -> Result<Self, ArmProfileIdError> {
        if bytes == [0; 32] {
            return Err(ArmProfileIdError::AllZero);
        }
        Ok(Self(bytes))
    }

    /// Parses an identity from its exact external representation.
    pub fn from_slice(bytes: &[u8]) -> Result<Self, ArmProfileIdError> {
        let bytes: [u8; 32] = bytes
            .try_into()
            .map_err(|_| ArmProfileIdError::InvalidLength {
                actual: bytes.len(),
            })?;
        Self::new(bytes)
    }

    /// Returns the canonical digest bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

/// Invalid compiled arm-profile identity.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum ArmProfileIdError {
    /// External representation was not exactly 32 bytes.
    #[error("arm profile identity must contain exactly 32 bytes, got {actual}")]
    InvalidLength {
        /// Observed byte length.
        actual: usize,
    },
    /// The all-zero value is reserved for an unspecified profile.
    #[error("arm profile identity cannot be all zero")]
    AllZero,
}

/// Coordinate family used by an active joint.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ArmCoordinateKind {
    /// Rotary coordinate expressed in radians.
    Angular,
    /// Translational coordinate expressed in metres.
    Linear,
}

/// Effort family used by an active joint.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ArmEffortKind {
    /// Rotary effort expressed in newton metres.
    Torque,
    /// Translational effort expressed in newtons.
    Force,
}

/// Supported active joint kinds.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ArmJointKind {
    /// Bounded rotary joint.
    Revolute,
    /// Unbounded-position rotary joint.
    Continuous,
    /// Bounded translational joint.
    Prismatic,
}

impl ArmJointKind {
    /// Returns the required coordinate unit family.
    #[must_use]
    pub const fn coordinate_kind(self) -> ArmCoordinateKind {
        match self {
            Self::Revolute | Self::Continuous => ArmCoordinateKind::Angular,
            Self::Prismatic => ArmCoordinateKind::Linear,
        }
    }

    /// Returns the required effort unit family.
    #[must_use]
    pub const fn effort_kind(self) -> ArmEffortKind {
        match self {
            Self::Revolute | Self::Continuous => ArmEffortKind::Torque,
            Self::Prismatic => ArmEffortKind::Force,
        }
    }
}

/// Finite tagged joint position in radians or metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmPosition {
    kind: ArmCoordinateKind,
    value_si: f64,
}

impl ArmPosition {
    /// Constructs an angular position in radians.
    pub fn angular_rad(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Angular, value, "arm.position_rad")
    }

    /// Constructs a linear position in metres.
    pub fn linear_m(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Linear, value, "arm.position_m")
    }

    fn new(
        kind: ArmCoordinateKind,
        value_si: f64,
        field: &'static str,
    ) -> Result<Self, SemanticError> {
        require_finite(value_si, field)?;
        Ok(Self { kind, value_si })
    }

    /// Returns the coordinate unit family.
    #[must_use]
    pub const fn kind(self) -> ArmCoordinateKind {
        self.kind
    }

    /// Returns radians for angular values or metres for linear values.
    #[must_use]
    pub const fn value_si(self) -> f64 {
        self.value_si
    }
}

/// Finite tagged joint velocity in radians/second or metres/second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmVelocity {
    kind: ArmCoordinateKind,
    value_si: f64,
}

impl ArmVelocity {
    /// Constructs an angular velocity in radians per second.
    pub fn angular_radps(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Angular, value, "arm.velocity_radps")
    }

    /// Constructs a linear velocity in metres per second.
    pub fn linear_mps(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Linear, value, "arm.velocity_mps")
    }

    fn new(
        kind: ArmCoordinateKind,
        value_si: f64,
        field: &'static str,
    ) -> Result<Self, SemanticError> {
        require_finite(value_si, field)?;
        Ok(Self { kind, value_si })
    }

    /// Returns the coordinate unit family.
    #[must_use]
    pub const fn kind(self) -> ArmCoordinateKind {
        self.kind
    }

    /// Returns radians/second or metres/second according to [`Self::kind`].
    #[must_use]
    pub const fn value_si(self) -> f64 {
        self.value_si
    }
}

/// Finite tagged joint acceleration in radians/second² or metres/second².
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmAcceleration {
    kind: ArmCoordinateKind,
    value_si: f64,
}

impl ArmAcceleration {
    /// Constructs an angular acceleration in radians per second squared.
    pub fn angular_radps2(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Angular, value, "arm.acceleration_radps2")
    }

    /// Constructs a linear acceleration in metres per second squared.
    pub fn linear_mps2(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmCoordinateKind::Linear, value, "arm.acceleration_mps2")
    }

    fn new(
        kind: ArmCoordinateKind,
        value_si: f64,
        field: &'static str,
    ) -> Result<Self, SemanticError> {
        require_finite(value_si, field)?;
        Ok(Self { kind, value_si })
    }

    /// Returns the coordinate unit family.
    #[must_use]
    pub const fn kind(self) -> ArmCoordinateKind {
        self.kind
    }

    /// Returns the finite SI value according to [`Self::kind`].
    #[must_use]
    pub const fn value_si(self) -> f64 {
        self.value_si
    }
}

/// Finite tagged joint effort in newton metres or newtons.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmEffort {
    kind: ArmEffortKind,
    value_si: f64,
}

impl ArmEffort {
    /// Constructs rotary effort in newton metres.
    pub fn torque_nm(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmEffortKind::Torque, value, "arm.effort_nm")
    }

    /// Constructs translational effort in newtons.
    pub fn force_n(value: f64) -> Result<Self, SemanticError> {
        Self::new(ArmEffortKind::Force, value, "arm.effort_n")
    }

    fn new(kind: ArmEffortKind, value_si: f64, field: &'static str) -> Result<Self, SemanticError> {
        require_finite(value_si, field)?;
        Ok(Self { kind, value_si })
    }

    /// Returns the effort unit family.
    #[must_use]
    pub const fn kind(self) -> ArmEffortKind {
        self.kind
    }

    /// Returns newton metres or newtons according to [`Self::kind`].
    #[must_use]
    pub const fn value_si(self) -> f64 {
        self.value_si
    }
}

/// Finite direct rotary-actuator torque in newton metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotaryTorqueNm(f64);

impl RotaryTorqueNm {
    /// Constructs a finite actuator torque.
    pub fn new(value: f64) -> Result<Self, SemanticError> {
        require_finite(value, "arm.actuator_torque_nm")?;
        Ok(Self(value))
    }

    /// Returns torque in newton metres.
    #[must_use]
    pub const fn get(self) -> f64 {
        self.0
    }
}

/// One entry in a stable active-joint order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmJointDescriptor {
    joint_id: ArmJointId,
    kind: ArmJointKind,
}

impl ArmJointDescriptor {
    /// Constructs a joint descriptor.
    #[must_use]
    pub const fn new(joint_id: ArmJointId, kind: ArmJointKind) -> Self {
        Self { joint_id, kind }
    }

    /// Returns the stable joint identifier.
    #[must_use]
    pub const fn joint_id(self) -> ArmJointId {
        self.joint_id
    }

    /// Returns the active joint kind.
    #[must_use]
    pub const fn kind(self) -> ArmJointKind {
        self.kind
    }
}

/// Nonempty duplicate-free bounded active-joint order.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArmJointOrder(ArrayVec<ArmJointDescriptor, MAX_JOINTS>);

impl ArmJointOrder {
    /// Validates one explicit stable order.
    pub fn new(joints: ArrayVec<ArmJointDescriptor, MAX_JOINTS>) -> Result<Self, SemanticError> {
        if joints.is_empty() {
            return Err(SemanticError::EmptyCollection {
                field: "arm.joint_order",
            });
        }
        for (index, joint) in joints.iter().enumerate() {
            if joints[..index]
                .iter()
                .any(|existing| existing.joint_id == joint.joint_id)
            {
                return Err(SemanticError::DuplicateIdentifier {
                    field: "arm.joint_order",
                });
            }
        }
        Ok(Self(joints))
    }

    /// Returns the stable ordered descriptors.
    #[must_use]
    pub fn joints(&self) -> &[ArmJointDescriptor] {
        &self.0
    }

    /// Returns the active degree of freedom.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns whether the order is empty.
    ///
    /// Valid constructed orders are never empty; this accessor supports
    /// generic collection code without exposing mutation.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// One stable joint-to-rotary-actuator binding.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmActuatorBinding {
    joint_id: ArmJointId,
    actuator_id: ArmActuatorId,
}

impl ArmActuatorBinding {
    /// Constructs a logical binding.
    #[must_use]
    pub const fn new(joint_id: ArmJointId, actuator_id: ArmActuatorId) -> Self {
        Self {
            joint_id,
            actuator_id,
        }
    }

    /// Returns the kinematic joint identity.
    #[must_use]
    pub const fn joint_id(self) -> ArmJointId {
        self.joint_id
    }

    /// Returns the rotary actuator identity.
    #[must_use]
    pub const fn actuator_id(self) -> ArmActuatorId {
        self.actuator_id
    }
}

/// Nonempty one-to-one joint-to-actuator binding order.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArmActuatorOrder(ArrayVec<ArmActuatorBinding, MAX_JOINTS>);

impl ArmActuatorOrder {
    /// Validates unique joints and actuators in their explicit compiled order.
    pub fn new(bindings: ArrayVec<ArmActuatorBinding, MAX_JOINTS>) -> Result<Self, SemanticError> {
        if bindings.is_empty() {
            return Err(SemanticError::EmptyCollection {
                field: "arm.actuator_order",
            });
        }
        for (index, binding) in bindings.iter().enumerate() {
            if bindings[..index].iter().any(|existing| {
                existing.joint_id == binding.joint_id || existing.actuator_id == binding.actuator_id
            }) {
                return Err(SemanticError::DuplicateIdentifier {
                    field: "arm.actuator_order",
                });
            }
        }
        Ok(Self(bindings))
    }

    /// Returns the compiled binding order.
    #[must_use]
    pub fn bindings(&self) -> &[ArmActuatorBinding] {
        &self.0
    }

    /// Returns the actuator degree of freedom.
    #[must_use]
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Returns whether the order is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

/// Immutable identity and generation carried by every arm-control proposal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ArmProposalContext {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
}

impl ArmProposalContext {
    /// Constructs a proposal context. Generation zero is unspecified.
    pub fn new(
        profile_id: ArmProfileId,
        group_id: KinematicGroupId,
        generation: u64,
    ) -> Result<Self, SemanticError> {
        if generation == 0 {
            return Err(SemanticError::ZeroCount {
                field: "arm.proposal.generation",
            });
        }
        Ok(Self {
            profile_id,
            group_id,
            generation,
        })
    }

    /// Returns the compiled arm-profile identity.
    #[must_use]
    pub const fn profile_id(self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact target group.
    #[must_use]
    pub const fn group_id(self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the producing model/policy generation.
    #[must_use]
    pub const fn generation(self) -> u64 {
        self.generation
    }
}

/// Stable-order vector of joint-space values.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmJointVector<T> {
    order: ArmJointOrder,
    values: ArrayVec<T, MAX_JOINTS>,
}

impl<T> ArmJointVector<T> {
    /// Requires one value for every entry in the declared joint order.
    pub fn new(
        order: ArmJointOrder,
        values: ArrayVec<T, MAX_JOINTS>,
    ) -> Result<Self, SemanticError> {
        if values.len() != order.len() {
            return Err(SemanticError::DimensionMismatch {
                field: "arm.joint_vector",
                expected: order.len(),
                actual: values.len(),
            });
        }
        Ok(Self { order, values })
    }

    /// Returns the exact joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns values in the declared joint order.
    #[must_use]
    pub fn values(&self) -> &[T] {
        &self.values
    }
}

/// Stable-order vector of rotary-actuator values.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmActuatorVector<T> {
    order: ArmActuatorOrder,
    values: ArrayVec<T, MAX_JOINTS>,
}

impl<T> ArmActuatorVector<T> {
    /// Requires one value for every compiled actuator binding.
    pub fn new(
        order: ArmActuatorOrder,
        values: ArrayVec<T, MAX_JOINTS>,
    ) -> Result<Self, SemanticError> {
        if values.len() != order.len() {
            return Err(SemanticError::DimensionMismatch {
                field: "arm.actuator_vector",
                expected: order.len(),
                actual: values.len(),
            });
        }
        Ok(Self { order, values })
    }

    /// Returns the exact joint-to-actuator binding order.
    #[must_use]
    pub const fn order(&self) -> &ArmActuatorOrder {
        &self.order
    }

    /// Returns values in compiled binding order.
    #[must_use]
    pub fn values(&self) -> &[T] {
        &self.values
    }
}

/// One coherent tagged arm-joint observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmJointSample {
    descriptor: ArmJointDescriptor,
    position: ArmPosition,
    velocity: ArmVelocity,
    effort: ArmEffort,
}

impl ArmJointSample {
    /// Constructs a sample and checks that every SI tag matches its joint kind.
    pub fn new(
        descriptor: ArmJointDescriptor,
        position: ArmPosition,
        velocity: ArmVelocity,
        effort: ArmEffort,
    ) -> Result<Self, SemanticError> {
        if position.kind() != descriptor.kind().coordinate_kind() {
            return Err(SemanticError::UnitMismatch {
                field: "arm.observation.position",
                index: 0,
            });
        }
        if velocity.kind() != descriptor.kind().coordinate_kind() {
            return Err(SemanticError::UnitMismatch {
                field: "arm.observation.velocity",
                index: 0,
            });
        }
        if effort.kind() != descriptor.kind().effort_kind() {
            return Err(SemanticError::UnitMismatch {
                field: "arm.observation.effort",
                index: 0,
            });
        }
        Ok(Self {
            descriptor,
            position,
            velocity,
            effort,
        })
    }

    /// Returns the joint descriptor.
    #[must_use]
    pub const fn descriptor(self) -> ArmJointDescriptor {
        self.descriptor
    }

    /// Returns the joint position.
    #[must_use]
    pub const fn position(self) -> ArmPosition {
        self.position
    }

    /// Returns the joint velocity.
    #[must_use]
    pub const fn velocity(self) -> ArmVelocity {
        self.velocity
    }

    /// Returns the joint effort.
    #[must_use]
    pub const fn effort(self) -> ArmEffort {
        self.effort
    }
}

/// Time-aligned arm observation bound to one compiled profile and group.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmJointObservation {
    header: Header,
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    samples: ArrayVec<ArmJointSample, MAX_JOINTS>,
}

impl ArmJointObservation {
    /// Constructs a nonempty duplicate-free observation in stable order.
    pub fn new(
        header: Header,
        profile_id: ArmProfileId,
        group_id: KinematicGroupId,
        samples: ArrayVec<ArmJointSample, MAX_JOINTS>,
    ) -> Result<Self, SemanticError> {
        if samples.is_empty() {
            return Err(SemanticError::EmptyCollection {
                field: "arm.observation.samples",
            });
        }
        for (index, sample) in samples.iter().enumerate() {
            if samples[..index]
                .iter()
                .any(|existing| existing.descriptor().joint_id() == sample.descriptor().joint_id())
            {
                return Err(SemanticError::DuplicateIdentifier {
                    field: "arm.observation.samples",
                });
            }
        }
        Ok(Self {
            header,
            profile_id,
            group_id,
            samples,
        })
    }

    /// Returns observation metadata.
    #[must_use]
    pub const fn header(&self) -> Header {
        self.header
    }

    /// Returns the compiled profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact observed group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns samples in stable group order.
    #[must_use]
    pub fn samples(&self) -> &[ArmJointSample] {
        &self.samples
    }
}

/// Validated actuator-space MIT tuple.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotaryMitSetpoint {
    position_rad: f64,
    velocity_radps: f64,
    kp: f64,
    kd: f64,
    feed_forward_torque_nm: f64,
}

impl RotaryMitSetpoint {
    /// Constructs a finite tuple with nonnegative impedance gains.
    pub fn new(
        position_rad: f64,
        velocity_radps: f64,
        kp: f64,
        kd: f64,
        feed_forward_torque_nm: f64,
    ) -> Result<Self, SemanticError> {
        require_finite(position_rad, "arm.mit.position_rad")?;
        require_finite(velocity_radps, "arm.mit.velocity_radps")?;
        require_finite(kp, "arm.mit.kp")?;
        require_finite(kd, "arm.mit.kd")?;
        require_finite(feed_forward_torque_nm, "arm.mit.feed_forward_torque_nm")?;
        if kp < 0.0 {
            return Err(SemanticError::OutsideRange {
                field: "arm.mit.kp",
            });
        }
        if kd < 0.0 {
            return Err(SemanticError::OutsideRange {
                field: "arm.mit.kd",
            });
        }
        Ok(Self {
            position_rad,
            velocity_radps,
            kp,
            kd,
            feed_forward_torque_nm,
        })
    }

    /// Returns actuator position in radians.
    #[must_use]
    pub const fn position_rad(self) -> f64 {
        self.position_rad
    }

    /// Returns actuator velocity in radians per second.
    #[must_use]
    pub const fn velocity_radps(self) -> f64 {
        self.velocity_radps
    }

    /// Returns proportional gain.
    #[must_use]
    pub const fn kp(self) -> f64 {
        self.kp
    }

    /// Returns derivative gain.
    #[must_use]
    pub const fn kd(self) -> f64 {
        self.kd
    }

    /// Returns feed-forward torque in newton metres.
    #[must_use]
    pub const fn feed_forward_torque_nm(self) -> f64 {
        self.feed_forward_torque_nm
    }
}

/// Closed action semantic selected by an arm-control proposal.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArmControlMode {
    /// Absolute joint-space position target.
    AbsolutePosition,
    /// Residual added to a coherent fresh measured position.
    PositionResidual,
    /// Joint-space velocity target.
    Velocity,
    /// Direct rotary-actuator torque proposal.
    DirectRotaryTorque,
    /// Fixed-gain actuator-space MIT impedance proposal.
    Mit,
}

/// One value in a uniform fixed-capacity proposal payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ArmControlValue {
    /// Tagged joint-space position or residual.
    Position(ArmPosition),
    /// Tagged joint-space velocity.
    Velocity(ArmVelocity),
    /// Rotary-actuator torque.
    RotaryTorque(RotaryTorqueNm),
    /// Full actuator-space MIT tuple.
    Mit(RotaryMitSetpoint),
}

/// One project-owned semantic arm-control proposal.
///
/// The profile/group context uniquely owns the stable order. The bounded
/// payload stores only the values and DOF, avoiding a repeated 64-entry name
/// table in every local proposal.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmControlProposal {
    context: ArmProposalContext,
    mode: ArmControlMode,
    dof: u8,
    values: ArrayVec<ArmControlValue, MAX_JOINTS>,
}

impl ArmControlProposal {
    /// Constructs an absolute joint-space position proposal.
    pub fn absolute_position(
        context: ArmProposalContext,
        target: ArmJointVector<ArmPosition>,
    ) -> Result<Self, SemanticError> {
        validate_coordinate_units(
            target.order(),
            target.values().iter().map(|value| value.kind()),
            "arm.proposal.position",
        )?;
        Ok(Self::from_values(
            context,
            ArmControlMode::AbsolutePosition,
            target
                .values
                .into_iter()
                .map(ArmControlValue::Position)
                .collect(),
        ))
    }

    /// Constructs a joint-space position-residual proposal.
    pub fn position_residual(
        context: ArmProposalContext,
        residual: ArmJointVector<ArmPosition>,
    ) -> Result<Self, SemanticError> {
        validate_coordinate_units(
            residual.order(),
            residual.values().iter().map(|value| value.kind()),
            "arm.proposal.position_residual",
        )?;
        Ok(Self::from_values(
            context,
            ArmControlMode::PositionResidual,
            residual
                .values
                .into_iter()
                .map(ArmControlValue::Position)
                .collect(),
        ))
    }

    /// Constructs a joint-space velocity proposal.
    pub fn velocity(
        context: ArmProposalContext,
        target: ArmJointVector<ArmVelocity>,
    ) -> Result<Self, SemanticError> {
        validate_coordinate_units(
            target.order(),
            target.values().iter().map(|value| value.kind()),
            "arm.proposal.velocity",
        )?;
        Ok(Self::from_values(
            context,
            ArmControlMode::Velocity,
            target
                .values
                .into_iter()
                .map(ArmControlValue::Velocity)
                .collect(),
        ))
    }

    /// Constructs a direct rotary-actuator torque proposal.
    #[must_use]
    pub fn direct_rotary_torque(
        context: ArmProposalContext,
        target: ArmActuatorVector<RotaryTorqueNm>,
    ) -> Self {
        Self::from_values(
            context,
            ArmControlMode::DirectRotaryTorque,
            target
                .values
                .into_iter()
                .map(ArmControlValue::RotaryTorque)
                .collect(),
        )
    }

    /// Constructs a fixed-gain actuator-space MIT proposal.
    #[must_use]
    pub fn mit(context: ArmProposalContext, target: ArmActuatorVector<RotaryMitSetpoint>) -> Self {
        Self::from_values(
            context,
            ArmControlMode::Mit,
            target
                .values
                .into_iter()
                .map(ArmControlValue::Mit)
                .collect(),
        )
    }

    fn from_values(
        context: ArmProposalContext,
        mode: ArmControlMode,
        values: ArrayVec<ArmControlValue, MAX_JOINTS>,
    ) -> Self {
        Self {
            context,
            mode,
            dof: values.len() as u8,
            values,
        }
    }

    /// Returns the immutable proposal identity/generation.
    #[must_use]
    pub const fn context(&self) -> ArmProposalContext {
        self.context
    }

    /// Returns the exact closed action semantic.
    #[must_use]
    pub const fn mode(&self) -> ArmControlMode {
        self.mode
    }

    /// Returns the expected group/actuator degree of freedom.
    #[must_use]
    pub const fn dof(&self) -> u8 {
        self.dof
    }

    /// Returns values in the context profile/group's stable compiled order.
    #[must_use]
    pub fn values(&self) -> &[ArmControlValue] {
        &self.values
    }
}

/// One finite point in a concrete joint-space arm trajectory.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmTrajectoryPoint {
    time_from_start_ns: u64,
    positions: ArrayVec<ArmPosition, MAX_JOINTS>,
    velocities: ArrayVec<ArmVelocity, MAX_JOINTS>,
    accelerations: ArrayVec<ArmAcceleration, MAX_JOINTS>,
}

impl ArmTrajectoryPoint {
    /// Constructs one point and validates dimensions and SI tags.
    pub fn new(
        time_from_start_ns: u64,
        order: &ArmJointOrder,
        positions: ArrayVec<ArmPosition, MAX_JOINTS>,
        velocities: ArrayVec<ArmVelocity, MAX_JOINTS>,
        accelerations: ArrayVec<ArmAcceleration, MAX_JOINTS>,
    ) -> Result<Self, SemanticError> {
        if time_from_start_ns == 0 {
            return Err(SemanticError::ZeroDeadline {
                field: "arm.trajectory.time_from_start_ns",
            });
        }
        validate_joint_value_count(order, positions.len(), "arm.trajectory.positions")?;
        validate_joint_value_count(order, velocities.len(), "arm.trajectory.velocities")?;
        validate_joint_value_count(order, accelerations.len(), "arm.trajectory.accelerations")?;
        validate_coordinate_units(
            order,
            positions.iter().map(|value| value.kind()),
            "position",
        )?;
        validate_coordinate_units(
            order,
            velocities.iter().map(|value| value.kind()),
            "velocity",
        )?;
        validate_coordinate_units(
            order,
            accelerations.iter().map(|value| value.kind()),
            "acceleration",
        )?;
        Ok(Self {
            time_from_start_ns,
            positions,
            velocities,
            accelerations,
        })
    }

    /// Returns the strictly positive offset from trajectory start.
    #[must_use]
    pub const fn time_from_start_ns(&self) -> u64 {
        self.time_from_start_ns
    }

    /// Returns positions in goal order.
    #[must_use]
    pub fn positions(&self) -> &[ArmPosition] {
        &self.positions
    }

    /// Returns velocities in goal order.
    #[must_use]
    pub fn velocities(&self) -> &[ArmVelocity] {
        &self.velocities
    }

    /// Returns accelerations in goal order.
    #[must_use]
    pub fn accelerations(&self) -> &[ArmAcceleration] {
        &self.accelerations
    }
}

/// Concrete ExecuteArmTrajectory goal payload.
#[derive(Clone, Debug, PartialEq)]
pub struct ExecuteArmTrajectoryGoal {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    points: Vec<ArmTrajectoryPoint>,
}

impl ExecuteArmTrajectoryGoal {
    /// Constructs a bounded, nonempty, strictly time-ordered trajectory.
    pub fn new(
        profile_id: ArmProfileId,
        group_id: KinematicGroupId,
        order: ArmJointOrder,
        points: Vec<ArmTrajectoryPoint>,
    ) -> Result<Self, SemanticError> {
        if points.is_empty() {
            return Err(SemanticError::EmptyCollection {
                field: "arm.trajectory.points",
            });
        }
        if points.len() > MAX_TRAJECTORY_POINTS {
            return Err(SemanticError::InvalidBound {
                field: "arm.trajectory.points",
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        let mut previous = 0;
        for point in &points {
            if point.time_from_start_ns() <= previous {
                return Err(SemanticError::InvalidTimeOrder {
                    field: "arm.trajectory.points",
                });
            }
            previous = point.time_from_start_ns();
        }
        Ok(Self {
            profile_id,
            group_id,
            order,
            points,
        })
    }

    /// Returns the compiled profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact target group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the declared stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns bounded points.
    #[must_use]
    pub fn points(&self) -> &[ArmTrajectoryPoint] {
        &self.points
    }
}

/// Concrete ExecuteArmTrajectory feedback payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExecuteArmTrajectoryFeedback {
    active_point: u32,
    progress: f64,
}

impl ExecuteArmTrajectoryFeedback {
    /// Constructs finite feedback with progress in `[0, 1]`.
    pub fn new(active_point: u32, progress: f64) -> Result<Self, SemanticError> {
        require_finite(progress, "arm.trajectory.feedback.progress")?;
        if !(0.0..=1.0).contains(&progress) {
            return Err(SemanticError::OutsideClosedUnitInterval {
                field: "arm.trajectory.feedback.progress",
            });
        }
        Ok(Self {
            active_point,
            progress,
        })
    }

    /// Returns the zero-based active point index.
    #[must_use]
    pub const fn active_point(self) -> u32 {
        self.active_point
    }

    /// Returns progress in `[0, 1]`.
    #[must_use]
    pub const fn progress(self) -> f64 {
        self.progress
    }
}

/// Concrete ExecuteArmTrajectory terminal result payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExecuteArmTrajectoryResult {
    completed_points: u32,
}

impl ExecuteArmTrajectoryResult {
    /// Constructs a terminal result count.
    #[must_use]
    pub const fn new(completed_points: u32) -> Self {
        Self { completed_points }
    }

    /// Returns the number of completed points.
    #[must_use]
    pub const fn completed_points(self) -> u32 {
        self.completed_points
    }
}

fn validate_joint_value_count(
    order: &ArmJointOrder,
    actual: usize,
    field: &'static str,
) -> Result<(), SemanticError> {
    if actual == order.len() {
        Ok(())
    } else {
        Err(SemanticError::DimensionMismatch {
            field,
            expected: order.len(),
            actual,
        })
    }
}

fn validate_coordinate_units(
    order: &ArmJointOrder,
    values: impl Iterator<Item = ArmCoordinateKind>,
    field: &'static str,
) -> Result<(), SemanticError> {
    for (index, (joint, kind)) in order.joints().iter().zip(values).enumerate() {
        if joint.kind().coordinate_kind() != kind {
            return Err(SemanticError::UnitMismatch { field, index });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use arrayvec::ArrayVec;

    use super::{
        ArmAcceleration, ArmActuatorBinding, ArmActuatorOrder, ArmActuatorVector,
        ArmControlProposal, ArmEffort, ArmJointDescriptor, ArmJointKind, ArmJointObservation,
        ArmJointOrder, ArmJointSample, ArmJointVector, ArmPosition, ArmProfileId,
        ArmProfileIdError, ArmProposalContext, ArmTrajectoryPoint, ArmVelocity,
        ExecuteArmTrajectoryGoal, RotaryMitSetpoint, RotaryTorqueNm,
    };
    use crate::{
        AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128, MAX_JOINTS,
        SemanticError,
    };

    fn bounded<T>(values: impl IntoIterator<Item = T>) -> ArrayVec<T, MAX_JOINTS> {
        values.into_iter().collect()
    }

    fn profile_id() -> ArmProfileId {
        ArmProfileId::new([7; 32]).unwrap()
    }

    fn group_id() -> super::KinematicGroupId {
        AsciiId::parse("left_arm").unwrap()
    }

    fn joint(name: &str, kind: ArmJointKind) -> ArmJointDescriptor {
        ArmJointDescriptor::new(AsciiId::parse(name).unwrap(), kind)
    }

    fn mixed_order() -> ArmJointOrder {
        ArmJointOrder::new(bounded([
            joint("shoulder", ArmJointKind::Revolute),
            joint("slide", ArmJointKind::Prismatic),
        ]))
        .unwrap()
    }

    fn header() -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("arm-state").unwrap(),
            boot_id: Id128::new([2; 16]).unwrap(),
            sequence: 1,
            source_monotonic_ns: 10,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 100,
            frame_id: Some(AsciiId::parse("base_link").unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    #[test]
    fn profile_identity_rejects_invalid_external_values() {
        assert_eq!(ArmProfileId::new([0; 32]), Err(ArmProfileIdError::AllZero));
        assert_eq!(
            ArmProfileId::from_slice(&[1; 31]),
            Err(ArmProfileIdError::InvalidLength { actual: 31 })
        );
        assert_eq!(profile_id().as_bytes(), &[7; 32]);
    }

    #[test]
    fn stable_orders_are_nonempty_and_one_to_one() {
        assert_eq!(
            ArmJointOrder::new(ArrayVec::new()),
            Err(SemanticError::EmptyCollection {
                field: "arm.joint_order"
            })
        );
        let duplicate = bounded([
            joint("joint_1", ArmJointKind::Revolute),
            joint("joint_1", ArmJointKind::Continuous),
        ]);
        assert_eq!(
            ArmJointOrder::new(duplicate),
            Err(SemanticError::DuplicateIdentifier {
                field: "arm.joint_order"
            })
        );

        let joint_id = AsciiId::parse("joint_1").unwrap();
        let actuator_id = AsciiId::parse("motor_1").unwrap();
        let duplicate_binding = bounded([
            ArmActuatorBinding::new(joint_id, actuator_id),
            ArmActuatorBinding::new(joint_id, AsciiId::parse("motor_2").unwrap()),
        ]);
        assert!(ArmActuatorOrder::new(duplicate_binding).is_err());
    }

    #[test]
    fn tagged_values_reject_nonfinite_and_unit_mismatch() {
        assert!(ArmPosition::angular_rad(f64::NAN).is_err());
        assert!(ArmVelocity::linear_mps(f64::INFINITY).is_err());
        assert!(ArmEffort::force_n(f64::NEG_INFINITY).is_err());

        let sample = ArmJointSample::new(
            joint("slide", ArmJointKind::Prismatic),
            ArmPosition::angular_rad(0.0).unwrap(),
            ArmVelocity::linear_mps(0.0).unwrap(),
            ArmEffort::force_n(0.0).unwrap(),
        );
        assert_eq!(
            sample,
            Err(SemanticError::UnitMismatch {
                field: "arm.observation.position",
                index: 0,
            })
        );
    }

    #[test]
    fn observation_rejects_duplicate_joint_identity() {
        let descriptor = joint("joint_1", ArmJointKind::Revolute);
        let sample = ArmJointSample::new(
            descriptor,
            ArmPosition::angular_rad(0.0).unwrap(),
            ArmVelocity::angular_radps(0.0).unwrap(),
            ArmEffort::torque_nm(0.0).unwrap(),
        )
        .unwrap();
        assert!(
            ArmJointObservation::new(
                header(),
                profile_id(),
                group_id(),
                bounded([sample, sample]),
            )
            .is_err()
        );
    }

    #[test]
    fn proposal_vectors_enforce_dimension_and_mit_gain_rules() {
        let context = ArmProposalContext::new(profile_id(), group_id(), 1).unwrap();
        assert!(ArmProposalContext::new(profile_id(), group_id(), 0).is_err());
        let order = mixed_order();
        let wrong = ArmJointVector::new(
            order.clone(),
            bounded([ArmPosition::angular_rad(0.0).unwrap()]),
        );
        assert_eq!(
            wrong,
            Err(SemanticError::DimensionMismatch {
                field: "arm.joint_vector",
                expected: 2,
                actual: 1,
            })
        );

        let actuator_order = ArmActuatorOrder::new(bounded([ArmActuatorBinding::new(
            AsciiId::parse("shoulder").unwrap(),
            AsciiId::parse("motor_1").unwrap(),
        )]))
        .unwrap();
        let torque = ArmActuatorVector::new(
            actuator_order.clone(),
            bounded([RotaryTorqueNm::new(1.0).unwrap()]),
        )
        .unwrap();
        let proposal = ArmControlProposal::direct_rotary_torque(context, torque);
        assert_eq!(proposal.context(), context);
        assert!(RotaryTorqueNm::new(f64::NAN).is_err());
        assert!(RotaryMitSetpoint::new(0.0, 0.0, -1.0, 0.0, 0.0).is_err());

        let mit = RotaryMitSetpoint::new(0.1, 0.2, 10.0, 0.5, 1.0).unwrap();
        let mit_vector = ArmActuatorVector::new(actuator_order, bounded([mit])).unwrap();
        assert_eq!(mit_vector.values()[0].kp(), 10.0);
    }

    #[test]
    fn trajectory_checks_units_dimensions_order_and_point_bound() {
        let order = mixed_order();
        let positions = bounded([
            ArmPosition::angular_rad(0.0).unwrap(),
            ArmPosition::linear_m(0.1).unwrap(),
        ]);
        let velocities = bounded([
            ArmVelocity::angular_radps(0.0).unwrap(),
            ArmVelocity::linear_mps(0.0).unwrap(),
        ]);
        let accelerations = bounded([
            ArmAcceleration::angular_radps2(0.0).unwrap(),
            ArmAcceleration::linear_mps2(0.0).unwrap(),
        ]);
        let first =
            ArmTrajectoryPoint::new(10, &order, positions, velocities, accelerations).unwrap();
        let second = ArmTrajectoryPoint::new(
            20,
            &order,
            bounded([
                ArmPosition::angular_rad(0.1).unwrap(),
                ArmPosition::linear_m(0.2).unwrap(),
            ]),
            bounded([
                ArmVelocity::angular_radps(0.0).unwrap(),
                ArmVelocity::linear_mps(0.0).unwrap(),
            ]),
            bounded([
                ArmAcceleration::angular_radps2(0.0).unwrap(),
                ArmAcceleration::linear_mps2(0.0).unwrap(),
            ]),
        )
        .unwrap();
        assert!(
            ExecuteArmTrajectoryGoal::new(
                profile_id(),
                group_id(),
                order.clone(),
                vec![first.clone(), second],
            )
            .is_ok()
        );
        assert!(
            ExecuteArmTrajectoryGoal::new(
                profile_id(),
                group_id(),
                order,
                vec![first.clone(), first],
            )
            .is_err()
        );
    }
}
