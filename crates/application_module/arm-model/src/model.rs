//! Immutable project-owned model values.

use message_model::{
    ArmActuatorOrder, ArmJointId, ArmJointKind, ArmJointOrder, ArmProfileId, AsciiId,
    KinematicGroupId, MAX_ID_LEN,
};

use crate::RotaryTransmission;

/// Maximum accepted URDF source size.
pub const MAX_URDF_BYTES: usize = 1_048_576;
/// Maximum links in one compiled model.
pub const MAX_MODEL_LINKS: usize = 256;
/// Maximum joints, including fixed and mimic joints.
pub const MAX_MODEL_JOINTS: usize = 256;
/// Maximum independently addressable groups.
pub const MAX_KINEMATIC_GROUPS: usize = 16;
/// Active profile schema implemented by this crate.
pub const ARM_PROFILE_SCHEMA_VERSION: u32 = 1;

/// Stable model/profile name.
pub type ArmModelName = AsciiId<MAX_ID_LEN>;
/// Stable link identifier.
pub type ArmLinkId = AsciiId<MAX_ID_LEN>;

/// Whether a compiled profile may be selected by production configuration.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ArmProfileClass {
    /// A reviewed deployable profile.
    Production,
    /// A host-only conformance fixture.
    HostFixture,
}

/// Link inertia about an explicit inertial-frame origin.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkInertia {
    origin_xyz_m: [f64; 3],
    origin_rpy_rad: [f64; 3],
    mass_kg: f64,
    inertia_kg_m2: [f64; 6],
}

impl LinkInertia {
    pub(crate) const fn new(
        origin_xyz_m: [f64; 3],
        origin_rpy_rad: [f64; 3],
        mass_kg: f64,
        inertia_kg_m2: [f64; 6],
    ) -> Self {
        Self {
            origin_xyz_m,
            origin_rpy_rad,
            mass_kg,
            inertia_kg_m2,
        }
    }

    /// Returns inertial-frame translation in metres.
    #[must_use]
    pub const fn origin_xyz_m(self) -> [f64; 3] {
        self.origin_xyz_m
    }

    /// Returns inertial-frame roll/pitch/yaw in radians.
    #[must_use]
    pub const fn origin_rpy_rad(self) -> [f64; 3] {
        self.origin_rpy_rad
    }

    /// Returns mass in kilograms.
    #[must_use]
    pub const fn mass_kg(self) -> f64 {
        self.mass_kg
    }

    /// Returns `[ixx, ixy, ixz, iyy, iyz, izz]` in kg·m².
    #[must_use]
    pub const fn inertia_kg_m2(self) -> [f64; 6] {
        self.inertia_kg_m2
    }
}

/// One project-owned robot link.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmLink {
    pub(crate) id: ArmLinkId,
    pub(crate) inertia: Option<LinkInertia>,
}

impl ArmLink {
    /// Returns the stable link identity.
    #[must_use]
    pub const fn id(&self) -> ArmLinkId {
        self.id
    }

    /// Returns validated dynamics data when supplied.
    #[must_use]
    pub const fn inertia(&self) -> Option<LinkInertia> {
        self.inertia
    }
}

/// Fixed or supported active joint kind.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ModelJointKind {
    /// Rigid connection with no coordinate.
    Fixed,
    /// Supported one-DOF active coordinate.
    Active(ArmJointKind),
}

impl ModelJointKind {
    /// Returns the active kind or `None` for a fixed joint.
    #[must_use]
    pub const fn active(self) -> Option<ArmJointKind> {
        match self {
            Self::Fixed => None,
            Self::Active(kind) => Some(kind),
        }
    }
}

/// Validated limits for one active joint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmJointLimits {
    position: Option<[f64; 2]>,
    max_velocity: f64,
    max_acceleration: f64,
    max_effort: f64,
}

impl ArmJointLimits {
    pub(crate) const fn new(
        position: Option<[f64; 2]>,
        max_velocity: f64,
        max_acceleration: f64,
        max_effort: f64,
    ) -> Self {
        Self {
            position,
            max_velocity,
            max_acceleration,
            max_effort,
        }
    }

    /// Returns the ordered position interval, absent for continuous joints.
    #[must_use]
    pub const fn position(self) -> Option<[f64; 2]> {
        self.position
    }

    /// Returns maximum absolute velocity in the joint coordinate per second.
    #[must_use]
    pub const fn max_velocity(self) -> f64 {
        self.max_velocity
    }

    /// Returns maximum absolute acceleration in the joint coordinate per second².
    #[must_use]
    pub const fn max_acceleration(self) -> f64 {
        self.max_acceleration
    }

    /// Returns maximum absolute torque or force according to the joint kind.
    #[must_use]
    pub const fn max_effort(self) -> f64 {
        self.max_effort
    }
}

/// Validated mimic relation. Mimic joints are not independent group DOF.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MimicRelation {
    source_joint: ArmJointId,
    multiplier: f64,
    offset: f64,
}

impl MimicRelation {
    pub(crate) const fn new(source_joint: ArmJointId, multiplier: f64, offset: f64) -> Self {
        Self {
            source_joint,
            multiplier,
            offset,
        }
    }

    /// Returns the independent/source joint.
    #[must_use]
    pub const fn source_joint(self) -> ArmJointId {
        self.source_joint
    }

    /// Returns the coordinate multiplier.
    #[must_use]
    pub const fn multiplier(self) -> f64 {
        self.multiplier
    }

    /// Returns the coordinate offset in the mimic joint's unit.
    #[must_use]
    pub const fn offset(self) -> f64 {
        self.offset
    }
}

/// One validated model joint.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmJoint {
    pub(crate) id: ArmJointId,
    pub(crate) parent_link: ArmLinkId,
    pub(crate) child_link: ArmLinkId,
    pub(crate) kind: ModelJointKind,
    pub(crate) origin_xyz_m: [f64; 3],
    pub(crate) origin_rpy_rad: [f64; 3],
    pub(crate) axis: [f64; 3],
    pub(crate) limits: Option<ArmJointLimits>,
    pub(crate) mimic: Option<MimicRelation>,
}

impl ArmJoint {
    /// Returns the joint identity.
    #[must_use]
    pub const fn id(&self) -> ArmJointId {
        self.id
    }

    /// Returns the parent link.
    #[must_use]
    pub const fn parent_link(&self) -> ArmLinkId {
        self.parent_link
    }

    /// Returns the child link.
    #[must_use]
    pub const fn child_link(&self) -> ArmLinkId {
        self.child_link
    }

    /// Returns fixed or active joint kind.
    #[must_use]
    pub const fn kind(&self) -> ModelJointKind {
        self.kind
    }

    /// Returns joint-frame translation in metres.
    #[must_use]
    pub const fn origin_xyz_m(&self) -> [f64; 3] {
        self.origin_xyz_m
    }

    /// Returns joint-frame roll/pitch/yaw in radians.
    #[must_use]
    pub const fn origin_rpy_rad(&self) -> [f64; 3] {
        self.origin_rpy_rad
    }

    /// Returns the normalized active-joint axis, or zeros for a fixed joint.
    #[must_use]
    pub const fn axis(&self) -> [f64; 3] {
        self.axis
    }

    /// Returns active-joint limits.
    #[must_use]
    pub const fn limits(&self) -> Option<ArmJointLimits> {
        self.limits
    }

    /// Returns a validated mimic relation when this joint is dependent.
    #[must_use]
    pub const fn mimic(&self) -> Option<MimicRelation> {
        self.mimic
    }

    /// Returns whether this joint contributes an independent group coordinate.
    #[must_use]
    pub const fn is_independent_active(&self) -> bool {
        self.kind.active().is_some() && self.mimic.is_none()
    }
}

/// One independently addressable base-to-tip open chain.
#[derive(Clone, Debug, PartialEq)]
pub struct KinematicGroup {
    pub(crate) id: KinematicGroupId,
    pub(crate) base_link: ArmLinkId,
    pub(crate) tip_link: ArmLinkId,
    pub(crate) joint_order: ArmJointOrder,
    pub(crate) path_joint_indices: Vec<usize>,
    pub(crate) actuator_order: Option<ArmActuatorOrder>,
}

impl KinematicGroup {
    /// Returns the stable group identity.
    #[must_use]
    pub const fn id(&self) -> KinematicGroupId {
        self.id
    }

    /// Returns the base link.
    #[must_use]
    pub const fn base_link(&self) -> ArmLinkId {
        self.base_link
    }

    /// Returns the tool/tip link.
    #[must_use]
    pub const fn tip_link(&self) -> ArmLinkId {
        self.tip_link
    }

    /// Returns the exact independent active-joint order.
    #[must_use]
    pub const fn joint_order(&self) -> &ArmJointOrder {
        &self.joint_order
    }

    /// Returns model joint indices along the full base-to-tip path.
    #[must_use]
    pub fn path_joint_indices(&self) -> &[usize] {
        &self.path_joint_indices
    }

    /// Returns complete one-to-one actuator order when MIT is supported.
    #[must_use]
    pub const fn actuator_order(&self) -> Option<&ArmActuatorOrder> {
        self.actuator_order.as_ref()
    }
}

/// Immutable compiled arm morphology.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmModel {
    pub(crate) profile_id: ArmProfileId,
    pub(crate) name: ArmModelName,
    pub(crate) class: ArmProfileClass,
    pub(crate) root_link: ArmLinkId,
    pub(crate) links: Vec<ArmLink>,
    pub(crate) joints: Vec<ArmJoint>,
    pub(crate) groups: Vec<KinematicGroup>,
    pub(crate) transmissions: Vec<RotaryTransmission>,
}

impl ArmModel {
    /// Returns the immutable semantic profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the stable profile name.
    #[must_use]
    pub const fn name(&self) -> ArmModelName {
        self.name
    }

    /// Returns the selection class.
    #[must_use]
    pub const fn class(&self) -> ArmProfileClass {
        self.class
    }

    /// Returns whether production configuration may select this profile.
    #[must_use]
    pub const fn is_production_selectable(&self) -> bool {
        matches!(self.class, ArmProfileClass::Production)
    }

    /// Returns the unique root link.
    #[must_use]
    pub const fn root_link(&self) -> ArmLinkId {
        self.root_link
    }

    /// Returns bounded links.
    #[must_use]
    pub fn links(&self) -> &[ArmLink] {
        &self.links
    }

    /// Returns bounded model joints.
    #[must_use]
    pub fn joints(&self) -> &[ArmJoint] {
        &self.joints
    }

    /// Returns groups in explicit profile order.
    #[must_use]
    pub fn groups(&self) -> &[KinematicGroup] {
        &self.groups
    }

    /// Returns project-owned transmission mappings.
    #[must_use]
    pub fn transmissions(&self) -> &[RotaryTransmission] {
        &self.transmissions
    }

    /// Finds a named group.
    #[must_use]
    pub fn group(&self, id: KinematicGroupId) -> Option<&KinematicGroup> {
        self.groups.iter().find(|group| group.id == id)
    }

    /// Finds a joint by stable identity.
    #[must_use]
    pub fn joint(&self, id: ArmJointId) -> Option<&ArmJoint> {
        self.joints.iter().find(|joint| joint.id == id)
    }

    /// Finds a link by stable identity.
    #[must_use]
    pub fn link(&self, id: ArmLinkId) -> Option<&ArmLink> {
        self.links.iter().find(|link| link.id == id)
    }

    /// Finds a rotary transmission by joint identity.
    #[must_use]
    pub fn transmission(&self, joint_id: ArmJointId) -> Option<&RotaryTransmission> {
        self.transmissions
            .iter()
            .find(|transmission| transmission.binding().joint_id() == joint_id)
    }
}
