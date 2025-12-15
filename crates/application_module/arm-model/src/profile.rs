//! Bounded URDF compilation and deterministic profile identity.

use std::collections::{BTreeMap, BTreeSet, VecDeque};

use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorBinding, ArmActuatorOrder, ArmJointDescriptor, ArmJointId, ArmJointKind,
    ArmJointOrder, ArmProfileId, AsciiId, KinematicGroupId, MAX_ID_LEN, MAX_JOINTS,
};
use sha2::{Digest, Sha256};

use crate::{
    ARM_PROFILE_SCHEMA_VERSION, ArmJoint, ArmJointLimits, ArmLink, ArmLinkId, ArmModel,
    ArmModelError, ArmModelName, ArmProfileClass, KinematicGroup, LinkInertia,
    MAX_KINEMATIC_GROUPS, MAX_MODEL_JOINTS, MAX_MODEL_LINKS, MAX_URDF_BYTES, MimicRelation,
    ModelJointKind, RotaryTransmission,
    urdf::{ParsedJoint, ParsedJointKind, ParsedLink, parse_urdf},
};

/// Explicit profile input used to compile a bounded model.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmProfileSpec {
    /// Must equal [`ARM_PROFILE_SCHEMA_VERSION`].
    pub schema_version: u32,
    /// Stable profile name.
    pub name: ArmModelName,
    /// Production or host-fixture selection class.
    pub class: ArmProfileClass,
    /// Groups in intentional stable profile order.
    pub groups: Vec<KinematicGroupSpec>,
    /// Explicit acceleration limits absent from URDF.
    pub acceleration_limits: Vec<JointAccelerationLimitSpec>,
    /// Optional one-to-one rotary-actuator mappings.
    pub transmissions: Vec<RotaryTransmissionSpec>,
}

/// Explicit base-to-tip group and independent active-joint order.
#[derive(Clone, Debug, PartialEq)]
pub struct KinematicGroupSpec {
    /// Stable group identity.
    pub id: KinematicGroupId,
    /// Existing model base link.
    pub base_link: ArmLinkId,
    /// Existing model tip/tool link.
    pub tip_link: ArmLinkId,
    /// Independent active joints in exact base-to-tip order.
    pub active_joint_order: Vec<ArmJointId>,
}

/// Required active-joint acceleration bound.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointAccelerationLimitSpec {
    /// Active or mimic joint.
    pub joint_id: ArmJointId,
    /// Positive absolute limit in rad/s² or m/s² according to joint kind.
    pub maximum: f64,
}

/// Optional joint-to-rotary-actuator mapping.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotaryTransmissionSpec {
    /// Independent active joint.
    pub joint_id: ArmJointId,
    /// Stable logical rotary actuator.
    pub actuator_id: message_model::ArmActuatorId,
    /// Actuator radians per joint radian/metre. Sign records direction.
    pub scale_rad_per_joint_unit: f64,
    /// Actuator position offset in radians.
    pub offset_rad: f64,
}

impl ArmModel {
    /// Parses bounded URDF and compiles one immutable project-owned profile.
    pub fn compile_urdf(urdf: &str, spec: &ArmProfileSpec) -> Result<Self, ArmModelError> {
        compile_urdf(urdf, spec)
    }
}

/// Production-only registry that structurally rejects host fixtures.
#[derive(Debug, Default)]
pub struct ProductionProfileRegistry {
    profiles: BTreeMap<ArmModelName, ArmProfileId>,
}

impl ProductionProfileRegistry {
    /// Registers a unique deployable profile.
    pub fn register(&mut self, model: &ArmModel) -> Result<(), ProductionProfileRegistryError> {
        if !model.is_production_selectable() {
            return Err(ProductionProfileRegistryError::HostFixture);
        }
        if self.profiles.contains_key(&model.name()) {
            return Err(ProductionProfileRegistryError::Duplicate);
        }
        self.profiles.insert(model.name(), model.profile_id());
        Ok(())
    }

    /// Returns a registered immutable identity.
    #[must_use]
    pub fn profile_id(&self, name: ArmModelName) -> Option<ArmProfileId> {
        self.profiles.get(&name).copied()
    }
}

/// Production profile registration failure.
#[derive(Clone, Copy, Debug, Eq, thiserror::Error, PartialEq)]
pub enum ProductionProfileRegistryError {
    /// Host conformance fixtures can never become production profiles.
    #[error("host fixture profile cannot enter the production registry")]
    HostFixture,
    /// One name cannot be re-bound to another identity.
    #[error("production profile name is already registered")]
    Duplicate,
}

fn compile_urdf(urdf: &str, spec: &ArmProfileSpec) -> Result<ArmModel, ArmModelError> {
    if urdf.len() > MAX_URDF_BYTES {
        return Err(ArmModelError::UrdfTooLarge {
            actual: urdf.len(),
            maximum: MAX_URDF_BYTES,
        });
    }
    if spec.schema_version != ARM_PROFILE_SCHEMA_VERSION {
        return Err(ArmModelError::UnsupportedSchema {
            expected: ARM_PROFILE_SCHEMA_VERSION,
            actual: spec.schema_version,
        });
    }
    if spec.groups.is_empty() {
        return Err(ArmModelError::Empty {
            field: "profile.groups",
        });
    }
    check_capacity("profile.groups", spec.groups.len(), MAX_KINEMATIC_GROUPS)?;

    let source = parse_urdf(urdf)?;
    let _robot_name = parse_id("robot", &source.name)?;
    check_capacity("links", source.links.len(), MAX_MODEL_LINKS)?;
    check_capacity("joints", source.joints.len(), MAX_MODEL_JOINTS)?;
    if source.links.is_empty() {
        return Err(ArmModelError::Empty { field: "links" });
    }

    let acceleration_limits = acceleration_limit_map(&spec.acceleration_limits)?;
    let mut links = compile_links(&source.links)?;
    let link_ids = links.iter().map(|link| link.id).collect::<BTreeSet<_>>();
    let mut joints = compile_joints(&source.joints, &link_ids, &acceleration_limits)?;
    validate_acceleration_coverage(&joints, &acceleration_limits)?;
    validate_mimic_relations(&joints)?;
    let root_link = validate_tree(&links, &joints)?;
    let transmissions = compile_transmissions(&spec.transmissions, &joints)?;
    let groups = compile_groups(spec, &joints, &link_ids, &transmissions)?;

    // Source order is not semantic model order. Stable identifiers own the
    // canonical storage order; group and path order remain explicitly stored.
    links.sort_by_key(|link| link.id);
    joints.sort_by_key(|joint| joint.id);

    // Rebuild path indices after the canonical joint sort.
    let groups = rebuild_group_path_indices(groups, &joints)?;
    let profile_id = profile_identity(spec, root_link, &links, &joints, &groups, &transmissions)?;

    Ok(ArmModel {
        profile_id,
        name: spec.name,
        class: spec.class,
        root_link,
        links,
        joints,
        groups,
        transmissions,
    })
}

fn compile_links(source: &[ParsedLink]) -> Result<Vec<ArmLink>, ArmModelError> {
    let mut seen = BTreeSet::new();
    let mut links = Vec::with_capacity(source.len());
    for link in source {
        let id = parse_id("link", &link.name)?;
        if !seen.insert(id) {
            return Err(ArmModelError::Duplicate {
                field: "link",
                name: link.name.clone(),
            });
        }
        links.push(ArmLink {
            id,
            inertia: compile_inertia(link)?,
        });
    }
    Ok(links)
}

fn compile_inertia(link: &ParsedLink) -> Result<Option<LinkInertia>, ArmModelError> {
    let Some(inertia) = &link.inertia else {
        return Ok(None);
    };
    let origin_xyz = inertia.origin_xyz;
    let origin_rpy = inertia.origin_rpy;
    let values = inertia.matrix;
    let mass = inertia.mass;
    if !mass.is_finite()
        || mass <= 0.0
        || origin_xyz.iter().any(|value| !value.is_finite())
        || origin_rpy.iter().any(|value| !value.is_finite())
        || values.iter().any(|value| !value.is_finite())
        || !is_positive_physical_inertia(values)
    {
        return Err(ArmModelError::InvalidInertia {
            link: link.name.clone(),
        });
    }
    Ok(Some(LinkInertia::new(
        canonical_array(origin_xyz),
        canonical_array(origin_rpy),
        canonical_float(mass),
        values.map(canonical_float),
    )))
}

fn is_positive_physical_inertia(values: [f64; 6]) -> bool {
    let [ixx, ixy, ixz, iyy, iyz, izz] = values;
    if ixx <= 0.0 || iyy <= 0.0 || izz <= 0.0 {
        return false;
    }
    let minor2 = ixx * iyy - ixy * ixy;
    let determinant = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz);
    minor2 > 0.0 && determinant > 0.0 && ixx <= iyy + izz && iyy <= ixx + izz && izz <= ixx + iyy
}

fn compile_joints(
    source: &[ParsedJoint],
    link_ids: &BTreeSet<ArmLinkId>,
    accelerations: &BTreeMap<ArmJointId, f64>,
) -> Result<Vec<ArmJoint>, ArmModelError> {
    let mut seen = BTreeSet::new();
    let mut joints = Vec::with_capacity(source.len());
    for joint in source {
        let id = parse_id("joint", &joint.name)?;
        if !seen.insert(id) {
            return Err(ArmModelError::Duplicate {
                field: "joint",
                name: joint.name.clone(),
            });
        }
        let parent_link = parse_id("joint.parent", &joint.parent)?;
        let child_link = parse_id("joint.child", &joint.child)?;
        if !link_ids.contains(&parent_link) {
            return Err(ArmModelError::MissingLink {
                owner: joint.name.clone(),
                link: joint.parent.clone(),
            });
        }
        if !link_ids.contains(&child_link) {
            return Err(ArmModelError::MissingLink {
                owner: joint.name.clone(),
                link: joint.child.clone(),
            });
        }
        if parent_link == child_link {
            return Err(ArmModelError::SelfEdge {
                joint: joint.name.clone(),
                link: joint.parent.clone(),
            });
        }
        let origin_xyz_m = finite_array(joint.origin_xyz, "origin.xyz", &joint.name)?;
        let origin_rpy_rad = finite_array(joint.origin_rpy, "origin.rpy", &joint.name)?;
        let kind = map_joint_kind(&joint.name, joint.kind)?;
        let (axis, limits) = match kind {
            ModelJointKind::Fixed => ([0.0; 3], None),
            ModelJointKind::Active(active_kind) => {
                let axis = normalize_axis(joint.axis, &joint.name)?;
                let acceleration = accelerations.get(&id).copied().ok_or_else(|| {
                    ArmModelError::MissingAccelerationLimit {
                        joint: joint.name.clone(),
                    }
                })?;
                let limits = compile_joint_limits(joint, active_kind, acceleration)?;
                (axis, Some(limits))
            }
        };
        let mimic = match &joint.mimic {
            Some(mimic) => {
                let source_joint = parse_id("joint.mimic", &mimic.joint)?;
                let multiplier = mimic.multiplier.unwrap_or(1.0);
                let offset = mimic.offset.unwrap_or(0.0);
                if !multiplier.is_finite() || !offset.is_finite() {
                    return Err(ArmModelError::InvalidMimic {
                        joint: joint.name.clone(),
                        source_joint: mimic.joint.clone(),
                    });
                }
                Some(MimicRelation::new(
                    source_joint,
                    canonical_float(multiplier),
                    canonical_float(offset),
                ))
            }
            None => None,
        };
        if kind == ModelJointKind::Fixed && mimic.is_some() {
            return Err(ArmModelError::InvalidMimic {
                joint: joint.name.clone(),
                source_joint: joint
                    .mimic
                    .as_ref()
                    .map_or_else(String::new, |mimic| mimic.joint.clone()),
            });
        }
        joints.push(ArmJoint {
            id,
            parent_link,
            child_link,
            kind,
            origin_xyz_m,
            origin_rpy_rad,
            axis,
            limits,
            mimic,
        });
    }
    Ok(joints)
}

fn map_joint_kind(name: &str, kind: ParsedJointKind) -> Result<ModelJointKind, ArmModelError> {
    match kind {
        ParsedJointKind::Fixed => Ok(ModelJointKind::Fixed),
        ParsedJointKind::Revolute => Ok(ModelJointKind::Active(ArmJointKind::Revolute)),
        ParsedJointKind::Continuous => Ok(ModelJointKind::Active(ArmJointKind::Continuous)),
        ParsedJointKind::Prismatic => Ok(ModelJointKind::Active(ArmJointKind::Prismatic)),
        ParsedJointKind::Floating => Err(ArmModelError::UnsupportedJointType {
            joint: name.to_owned(),
            joint_type: "floating",
        }),
        ParsedJointKind::Planar => Err(ArmModelError::UnsupportedJointType {
            joint: name.to_owned(),
            joint_type: "planar",
        }),
        ParsedJointKind::Spherical => Err(ArmModelError::UnsupportedJointType {
            joint: name.to_owned(),
            joint_type: "spherical",
        }),
    }
}

fn compile_joint_limits(
    joint: &ParsedJoint,
    kind: ArmJointKind,
    acceleration: f64,
) -> Result<ArmJointLimits, ArmModelError> {
    if !acceleration.is_finite() || acceleration <= 0.0 {
        return Err(ArmModelError::InvalidJointLimit {
            joint: joint.name.clone(),
            field: "acceleration",
        });
    }
    let Some(velocity) = joint.limit.velocity else {
        return Err(ArmModelError::InvalidJointLimit {
            joint: joint.name.clone(),
            field: "velocity",
        });
    };
    if !velocity.is_finite() || velocity <= 0.0 {
        return Err(ArmModelError::InvalidJointLimit {
            joint: joint.name.clone(),
            field: "velocity",
        });
    }
    let Some(effort) = joint.limit.effort else {
        return Err(ArmModelError::InvalidJointLimit {
            joint: joint.name.clone(),
            field: "effort",
        });
    };
    if !effort.is_finite() || effort <= 0.0 {
        return Err(ArmModelError::InvalidJointLimit {
            joint: joint.name.clone(),
            field: "effort",
        });
    }
    let position = match kind {
        ArmJointKind::Continuous => None,
        ArmJointKind::Revolute | ArmJointKind::Prismatic => {
            let (Some(lower), Some(upper)) = (joint.limit.lower, joint.limit.upper) else {
                return Err(ArmModelError::InvalidJointLimit {
                    joint: joint.name.clone(),
                    field: "position",
                });
            };
            if !lower.is_finite() || !upper.is_finite() || lower >= upper {
                return Err(ArmModelError::InvalidJointLimit {
                    joint: joint.name.clone(),
                    field: "position",
                });
            }
            Some([canonical_float(lower), canonical_float(upper)])
        }
    };
    Ok(ArmJointLimits::new(
        position,
        canonical_float(velocity),
        canonical_float(acceleration),
        canonical_float(effort),
    ))
}

fn acceleration_limit_map(
    source: &[JointAccelerationLimitSpec],
) -> Result<BTreeMap<ArmJointId, f64>, ArmModelError> {
    let mut limits = BTreeMap::new();
    for limit in source {
        if !limit.maximum.is_finite() || limit.maximum <= 0.0 {
            return Err(ArmModelError::InvalidJointLimit {
                joint: limit.joint_id.as_str().to_owned(),
                field: "acceleration",
            });
        }
        if limits
            .insert(limit.joint_id, canonical_float(limit.maximum))
            .is_some()
        {
            return Err(ArmModelError::Duplicate {
                field: "acceleration_limit",
                name: limit.joint_id.as_str().to_owned(),
            });
        }
    }
    Ok(limits)
}

fn validate_acceleration_coverage(
    joints: &[ArmJoint],
    accelerations: &BTreeMap<ArmJointId, f64>,
) -> Result<(), ArmModelError> {
    let active = joints
        .iter()
        .filter(|joint| joint.kind.active().is_some())
        .map(|joint| joint.id)
        .collect::<BTreeSet<_>>();
    for id in accelerations.keys() {
        if !active.contains(id) {
            return Err(ArmModelError::UnknownAccelerationJoint {
                joint: id.as_str().to_owned(),
            });
        }
    }
    Ok(())
}

fn validate_mimic_relations(joints: &[ArmJoint]) -> Result<(), ArmModelError> {
    let by_id = joints
        .iter()
        .enumerate()
        .map(|(index, joint)| (joint.id, index))
        .collect::<BTreeMap<_, _>>();
    for joint in joints {
        if let Some(mimic) = joint.mimic {
            let Some(source_index) = by_id.get(&mimic.source_joint()).copied() else {
                return Err(ArmModelError::InvalidMimic {
                    joint: joint.id.as_str().to_owned(),
                    source_joint: mimic.source_joint().as_str().to_owned(),
                });
            };
            let source = &joints[source_index];
            if source.id == joint.id
                || source.kind.active().is_none()
                || source.kind.active().map(ArmJointKind::coordinate_kind)
                    != joint.kind.active().map(ArmJointKind::coordinate_kind)
            {
                return Err(ArmModelError::InvalidMimic {
                    joint: joint.id.as_str().to_owned(),
                    source_joint: mimic.source_joint().as_str().to_owned(),
                });
            }
        }
    }
    for joint in joints {
        let mut visited = BTreeSet::new();
        let mut current = joint;
        while let Some(mimic) = current.mimic {
            if !visited.insert(current.id) {
                return Err(ArmModelError::MimicCycle {
                    joint: current.id.as_str().to_owned(),
                });
            }
            let index = by_id.get(&mimic.source_joint()).copied().ok_or_else(|| {
                ArmModelError::InvalidMimic {
                    joint: current.id.as_str().to_owned(),
                    source_joint: mimic.source_joint().as_str().to_owned(),
                }
            })?;
            current = &joints[index];
        }
    }
    Ok(())
}

fn validate_tree(links: &[ArmLink], joints: &[ArmJoint]) -> Result<ArmLinkId, ArmModelError> {
    let mut parent_by_child = BTreeMap::new();
    let mut children_by_parent = BTreeMap::<ArmLinkId, Vec<ArmLinkId>>::new();
    for joint in joints {
        if parent_by_child
            .insert(joint.child_link, joint.parent_link)
            .is_some()
        {
            return Err(ArmModelError::MultipleParents {
                link: joint.child_link.as_str().to_owned(),
            });
        }
        children_by_parent
            .entry(joint.parent_link)
            .or_default()
            .push(joint.child_link);
    }
    let roots = links
        .iter()
        .filter(|link| !parent_by_child.contains_key(&link.id))
        .map(|link| link.id)
        .collect::<Vec<_>>();
    if roots.len() != 1 {
        return Err(ArmModelError::RootCount {
            actual: roots.len(),
        });
    }
    let root = roots[0];
    let mut queue = VecDeque::from([root]);
    let mut visited = BTreeSet::new();
    while let Some(link) = queue.pop_front() {
        if !visited.insert(link) {
            return Err(ArmModelError::Cycle {
                link: link.as_str().to_owned(),
            });
        }
        if let Some(children) = children_by_parent.get(&link) {
            queue.extend(children.iter().copied());
        }
    }
    if visited.len() != links.len() {
        return Err(ArmModelError::Disconnected {
            unreachable: links.len() - visited.len(),
        });
    }
    Ok(root)
}

fn compile_transmissions(
    source: &[RotaryTransmissionSpec],
    joints: &[ArmJoint],
) -> Result<Vec<RotaryTransmission>, ArmModelError> {
    check_capacity("transmissions", source.len(), MAX_MODEL_JOINTS)?;
    let joint_by_id = joints
        .iter()
        .map(|joint| (joint.id, joint))
        .collect::<BTreeMap<_, _>>();
    let mut joint_ids = BTreeSet::new();
    let mut actuator_ids = BTreeSet::new();
    let mut result = Vec::with_capacity(source.len());
    for transmission in source {
        let Some(joint) = joint_by_id.get(&transmission.joint_id).copied() else {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "joint does not exist",
            });
        };
        let Some(kind) = joint.kind.active() else {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "fixed joint cannot drive a rotary actuator",
            });
        };
        if joint.mimic.is_some() {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "mimic joint is not an independent actuator coordinate",
            });
        }
        if !joint_ids.insert(transmission.joint_id) {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "joint binding is duplicated",
            });
        }
        if !actuator_ids.insert(transmission.actuator_id) {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "actuator binding is duplicated",
            });
        }
        if !transmission.scale_rad_per_joint_unit.is_finite()
            || transmission.scale_rad_per_joint_unit == 0.0
            || !transmission.offset_rad.is_finite()
        {
            return Err(ArmModelError::InvalidTransmission {
                joint: transmission.joint_id.as_str().to_owned(),
                reason: "scale/offset must be finite and scale nonzero",
            });
        }
        result.push(RotaryTransmission::new(
            ArmActuatorBinding::new(transmission.joint_id, transmission.actuator_id),
            kind,
            canonical_float(transmission.scale_rad_per_joint_unit),
            canonical_float(transmission.offset_rad),
        ));
    }
    result.sort_by_key(|transmission| transmission.binding().joint_id());
    Ok(result)
}

fn compile_groups(
    spec: &ArmProfileSpec,
    joints: &[ArmJoint],
    link_ids: &BTreeSet<ArmLinkId>,
    transmissions: &[RotaryTransmission],
) -> Result<Vec<KinematicGroup>, ArmModelError> {
    let parent_joint_by_child = joints
        .iter()
        .enumerate()
        .map(|(index, joint)| (joint.child_link, index))
        .collect::<BTreeMap<_, _>>();
    let transmission_by_joint = transmissions
        .iter()
        .map(|transmission| (transmission.binding().joint_id(), transmission))
        .collect::<BTreeMap<_, _>>();
    let mut group_ids = BTreeSet::new();
    let mut groups = Vec::with_capacity(spec.groups.len());

    for group_spec in &spec.groups {
        if !group_ids.insert(group_spec.id) {
            return Err(ArmModelError::Duplicate {
                field: "kinematic_group",
                name: group_spec.id.as_str().to_owned(),
            });
        }
        if !link_ids.contains(&group_spec.base_link) {
            return Err(ArmModelError::MissingLink {
                owner: group_spec.id.as_str().to_owned(),
                link: group_spec.base_link.as_str().to_owned(),
            });
        }
        if !link_ids.contains(&group_spec.tip_link) {
            return Err(ArmModelError::MissingLink {
                owner: group_spec.id.as_str().to_owned(),
                link: group_spec.tip_link.as_str().to_owned(),
            });
        }
        if group_spec.base_link == group_spec.tip_link {
            return Err(ArmModelError::InvalidGroupPath {
                group: group_spec.id.as_str().to_owned(),
                base: group_spec.base_link.as_str().to_owned(),
                tip: group_spec.tip_link.as_str().to_owned(),
            });
        }
        let path = group_path(group_spec, joints, &parent_joint_by_child)?;
        let active = path
            .iter()
            .filter_map(|index| {
                let joint = &joints[*index];
                if joint.mimic.is_none() {
                    joint
                        .kind
                        .active()
                        .map(|kind| ArmJointDescriptor::new(joint.id, kind))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        if active.is_empty() || active.len() > MAX_JOINTS {
            return Err(ArmModelError::Capacity {
                field: "group.active_joints",
                actual: active.len(),
                maximum: MAX_JOINTS,
            });
        }
        if active
            .iter()
            .map(|joint| joint.joint_id())
            .ne(group_spec.active_joint_order.iter().copied())
        {
            return Err(ArmModelError::GroupOrderMismatch {
                group: group_spec.id.as_str().to_owned(),
            });
        }
        let joint_order = ArmJointOrder::new(active.into_iter().collect()).map_err(|_| {
            ArmModelError::GroupOrderMismatch {
                group: group_spec.id.as_str().to_owned(),
            }
        })?;

        let mut bindings = ArrayVec::<ArmActuatorBinding, MAX_JOINTS>::new();
        let mut complete = true;
        for descriptor in joint_order.joints() {
            if let Some(transmission) = transmission_by_joint.get(&descriptor.joint_id()) {
                bindings.push(transmission.binding());
            } else {
                complete = false;
                break;
            }
        }
        let actuator_order = if complete {
            Some(ArmActuatorOrder::new(bindings).map_err(|_| {
                ArmModelError::InvalidTransmission {
                    joint: group_spec.id.as_str().to_owned(),
                    reason: "group actuator order is not one-to-one",
                }
            })?)
        } else {
            None
        };
        groups.push(KinematicGroup {
            id: group_spec.id,
            base_link: group_spec.base_link,
            tip_link: group_spec.tip_link,
            joint_order,
            path_joint_indices: path,
            actuator_order,
        });
    }
    Ok(groups)
}

fn group_path(
    group: &KinematicGroupSpec,
    joints: &[ArmJoint],
    parent_joint_by_child: &BTreeMap<ArmLinkId, usize>,
) -> Result<Vec<usize>, ArmModelError> {
    let mut reversed = Vec::new();
    let mut current = group.tip_link;
    while current != group.base_link {
        if reversed.len() >= joints.len() {
            return Err(invalid_group_path(group));
        }
        let Some(index) = parent_joint_by_child.get(&current).copied() else {
            return Err(invalid_group_path(group));
        };
        reversed.push(index);
        current = joints[index].parent_link;
    }
    reversed.reverse();
    Ok(reversed)
}

fn invalid_group_path(group: &KinematicGroupSpec) -> ArmModelError {
    ArmModelError::InvalidGroupPath {
        group: group.id.as_str().to_owned(),
        base: group.base_link.as_str().to_owned(),
        tip: group.tip_link.as_str().to_owned(),
    }
}

fn rebuild_group_path_indices(
    mut groups: Vec<KinematicGroup>,
    joints: &[ArmJoint],
) -> Result<Vec<KinematicGroup>, ArmModelError> {
    let by_id = joints
        .iter()
        .enumerate()
        .map(|(index, joint)| (joint.id, index))
        .collect::<BTreeMap<_, _>>();
    for group in &mut groups {
        let mut rebuilt = Vec::with_capacity(group.path_joint_indices.len());
        let mut current = group.base_link;
        while current != group.tip_link {
            let Some((index, joint)) = joints.iter().enumerate().find(|(_, joint)| {
                joint.parent_link == current
                    && group_path_reaches(joints, joint.child_link, group.tip_link)
            }) else {
                return Err(ArmModelError::InvalidGroupPath {
                    group: group.id.as_str().to_owned(),
                    base: group.base_link.as_str().to_owned(),
                    tip: group.tip_link.as_str().to_owned(),
                });
            };
            rebuilt.push(index);
            current = joint.child_link;
        }
        // Ensure every independent joint is still represented exactly.
        if group
            .joint_order
            .joints()
            .iter()
            .any(|descriptor| !by_id.contains_key(&descriptor.joint_id()))
        {
            return Err(ArmModelError::GroupOrderMismatch {
                group: group.id.as_str().to_owned(),
            });
        }
        group.path_joint_indices = rebuilt;
    }
    Ok(groups)
}

fn group_path_reaches(joints: &[ArmJoint], start: ArmLinkId, tip: ArmLinkId) -> bool {
    let parent_by_child = joints
        .iter()
        .map(|joint| (joint.child_link, joint.parent_link))
        .collect::<BTreeMap<_, _>>();
    let mut current = tip;
    for _ in 0..=joints.len() {
        if current == start {
            return true;
        }
        let Some(parent) = parent_by_child.get(&current).copied() else {
            return false;
        };
        current = parent;
    }
    false
}

fn profile_identity(
    spec: &ArmProfileSpec,
    root_link: ArmLinkId,
    links: &[ArmLink],
    joints: &[ArmJoint],
    groups: &[KinematicGroup],
    transmissions: &[RotaryTransmission],
) -> Result<ArmProfileId, ArmModelError> {
    let mut hash = CanonicalHash::new();
    hash.bytes(b"rs-control.arm-profile");
    hash.u32(ARM_PROFILE_SCHEMA_VERSION);
    hash.id(spec.name);
    hash.u8(match spec.class {
        ArmProfileClass::Production => 1,
        ArmProfileClass::HostFixture => 2,
    });
    hash.id(root_link);
    hash.usize(links.len());
    for link in links {
        hash.id(link.id);
        match link.inertia {
            Some(inertia) => {
                hash.u8(1);
                hash.f64_array(&inertia.origin_xyz_m());
                hash.f64_array(&inertia.origin_rpy_rad());
                hash.f64(inertia.mass_kg());
                hash.f64_array(&inertia.inertia_kg_m2());
            }
            None => hash.u8(0),
        }
    }
    hash.usize(joints.len());
    for joint in joints {
        hash.id(joint.id);
        hash.id(joint.parent_link);
        hash.id(joint.child_link);
        hash.u8(match joint.kind {
            ModelJointKind::Fixed => 0,
            ModelJointKind::Active(ArmJointKind::Revolute) => 1,
            ModelJointKind::Active(ArmJointKind::Continuous) => 2,
            ModelJointKind::Active(ArmJointKind::Prismatic) => 3,
        });
        hash.f64_array(&joint.origin_xyz_m);
        hash.f64_array(&joint.origin_rpy_rad);
        hash.f64_array(&joint.axis);
        match joint.limits {
            Some(limits) => {
                hash.u8(1);
                match limits.position() {
                    Some(position) => {
                        hash.u8(1);
                        hash.f64_array(&position);
                    }
                    None => hash.u8(0),
                }
                hash.f64(limits.max_velocity());
                hash.f64(limits.max_acceleration());
                hash.f64(limits.max_effort());
            }
            None => hash.u8(0),
        }
        match joint.mimic {
            Some(mimic) => {
                hash.u8(1);
                hash.id(mimic.source_joint());
                hash.f64(mimic.multiplier());
                hash.f64(mimic.offset());
            }
            None => hash.u8(0),
        }
    }
    hash.usize(groups.len());
    for group in groups {
        hash.id(group.id);
        hash.id(group.base_link);
        hash.id(group.tip_link);
        hash.usize(group.joint_order.len());
        for descriptor in group.joint_order.joints() {
            hash.id(descriptor.joint_id());
            hash.u8(match descriptor.kind() {
                ArmJointKind::Revolute => 1,
                ArmJointKind::Continuous => 2,
                ArmJointKind::Prismatic => 3,
            });
        }
    }
    hash.usize(transmissions.len());
    for transmission in transmissions {
        hash.id(transmission.binding().joint_id());
        hash.id(transmission.binding().actuator_id());
        hash.f64(transmission.scale_rad_per_joint_unit());
        hash.f64(transmission.offset_rad());
    }
    ArmProfileId::new(hash.finish()).map_err(ArmModelError::from)
}

struct CanonicalHash(Sha256);

impl CanonicalHash {
    fn new() -> Self {
        Self(Sha256::new())
    }

    fn bytes(&mut self, value: &[u8]) {
        self.usize(value.len());
        self.0.update(value);
    }

    fn id<const N: usize>(&mut self, value: AsciiId<N>) {
        self.bytes(value.as_str().as_bytes());
    }

    fn u8(&mut self, value: u8) {
        self.0.update([value]);
    }

    fn u32(&mut self, value: u32) {
        self.0.update(value.to_be_bytes());
    }

    fn usize(&mut self, value: usize) {
        self.0.update((value as u64).to_be_bytes());
    }

    fn f64(&mut self, value: f64) {
        self.0
            .update(canonical_float(value).to_bits().to_be_bytes());
    }

    fn f64_array(&mut self, values: &[f64]) {
        self.usize(values.len());
        for value in values {
            self.f64(*value);
        }
    }

    fn finish(self) -> [u8; 32] {
        self.0.finalize().into()
    }
}

fn parse_id(field: &'static str, value: &str) -> Result<AsciiId<MAX_ID_LEN>, ArmModelError> {
    AsciiId::parse(value).map_err(|error| ArmModelError::Identifier {
        field,
        name: value.to_owned(),
        error,
    })
}

fn check_capacity(field: &'static str, actual: usize, maximum: usize) -> Result<(), ArmModelError> {
    if actual <= maximum {
        Ok(())
    } else {
        Err(ArmModelError::Capacity {
            field,
            actual,
            maximum,
        })
    }
}

fn finite_array(
    values: [f64; 3],
    field: &'static str,
    owner: &str,
) -> Result<[f64; 3], ArmModelError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(canonical_array(values))
    } else {
        Err(ArmModelError::NonFinite {
            field,
            owner: owner.to_owned(),
        })
    }
}

fn normalize_axis(axis: [f64; 3], joint: &str) -> Result<[f64; 3], ArmModelError> {
    if axis.iter().any(|value| !value.is_finite()) {
        return Err(ArmModelError::InvalidAxis {
            joint: joint.to_owned(),
        });
    }
    let norm = axis
        .iter()
        .fold(0.0_f64, |sum, value| value.mul_add(*value, sum))
        .sqrt();
    if !norm.is_finite() || norm <= f64::EPSILON {
        return Err(ArmModelError::InvalidAxis {
            joint: joint.to_owned(),
        });
    }
    Ok(axis.map(|value| canonical_float(value / norm)))
}

fn canonical_array<const N: usize>(values: [f64; N]) -> [f64; N] {
    values.map(canonical_float)
}

fn canonical_float(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
}

#[cfg(test)]
mod tests {
    use message_model::{ArmJointId, AsciiId};

    use super::{
        ArmModel, ArmProfileClass, ArmProfileSpec, JointAccelerationLimitSpec, KinematicGroupSpec,
        ProductionProfileRegistry, RotaryTransmissionSpec,
    };
    use crate::{ARM_PROFILE_SCHEMA_VERSION, ArmModelError};

    const SYNTHETIC_6DOF: &str = include_str!("../tests/fixtures/synthetic_6dof.urdf");
    const MIXED_MULTI_GROUP: &str = include_str!("../tests/fixtures/mixed_multi_group.urdf");

    fn id(value: &str) -> ArmJointId {
        AsciiId::parse(value).unwrap()
    }

    fn synthetic_spec() -> ArmProfileSpec {
        let joints = (1..=6)
            .map(|index| id(&format!("joint_{index}")))
            .collect::<Vec<_>>();
        ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("synthetic_6dof"),
            class: ArmProfileClass::HostFixture,
            groups: vec![KinematicGroupSpec {
                id: id("arm"),
                base_link: id("base_link"),
                tip_link: id("tool0"),
                active_joint_order: joints.clone(),
            }],
            acceleration_limits: joints
                .iter()
                .copied()
                .map(|joint_id| JointAccelerationLimitSpec {
                    joint_id,
                    maximum: 4.0,
                })
                .collect(),
            transmissions: joints
                .iter()
                .enumerate()
                .map(|(index, joint_id)| RotaryTransmissionSpec {
                    joint_id: *joint_id,
                    actuator_id: id(&format!("motor_{}", index + 1)),
                    scale_rad_per_joint_unit: if index == 2 { -100.0 } else { 100.0 },
                    offset_rad: 0.0,
                })
                .collect(),
        }
    }

    fn mixed_spec() -> ArmProfileSpec {
        let active = ["left_yaw", "left_slide", "right_roll"];
        ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("mixed_multi_group"),
            class: ArmProfileClass::HostFixture,
            groups: vec![
                KinematicGroupSpec {
                    id: id("left_arm"),
                    base_link: id("base_link"),
                    tip_link: id("left_tip"),
                    active_joint_order: vec![id("left_yaw"), id("left_slide")],
                },
                KinematicGroupSpec {
                    id: id("right_arm"),
                    base_link: id("base_link"),
                    tip_link: id("right_tip"),
                    active_joint_order: vec![id("right_roll")],
                },
            ],
            acceleration_limits: active
                .into_iter()
                .map(|joint| JointAccelerationLimitSpec {
                    joint_id: id(joint),
                    maximum: 2.0,
                })
                .collect(),
            transmissions: vec![
                RotaryTransmissionSpec {
                    joint_id: id("left_yaw"),
                    actuator_id: id("left_yaw_motor"),
                    scale_rad_per_joint_unit: 80.0,
                    offset_rad: 0.0,
                },
                RotaryTransmissionSpec {
                    joint_id: id("left_slide"),
                    actuator_id: id("left_slide_motor"),
                    scale_rad_per_joint_unit: 200.0,
                    offset_rad: 0.1,
                },
                RotaryTransmissionSpec {
                    joint_id: id("right_roll"),
                    actuator_id: id("right_roll_motor"),
                    scale_rad_per_joint_unit: -60.0,
                    offset_rad: 0.0,
                },
            ],
        }
    }

    #[test]
    fn compiles_synthetic_profile_with_stable_identity_and_order() {
        let first = ArmModel::compile_urdf(SYNTHETIC_6DOF, &synthetic_spec()).unwrap();
        let second = ArmModel::compile_urdf(SYNTHETIC_6DOF, &synthetic_spec()).unwrap();
        assert_eq!(first.profile_id(), second.profile_id());
        assert_eq!(first.groups()[0].joint_order().len(), 6);
        assert_eq!(first.groups()[0].actuator_order().unwrap().len(), 6);
        assert!(!first.is_production_selectable());
        assert!(first.links().iter().all(|link| link.inertia().is_some()));
    }

    #[test]
    fn semantic_limit_or_mapping_change_changes_profile_identity() {
        let baseline = ArmModel::compile_urdf(SYNTHETIC_6DOF, &synthetic_spec()).unwrap();
        let mut limit_changed = synthetic_spec();
        limit_changed.acceleration_limits[0].maximum = 5.0;
        let limit_changed = ArmModel::compile_urdf(SYNTHETIC_6DOF, &limit_changed).unwrap();
        assert_ne!(baseline.profile_id(), limit_changed.profile_id());

        let mut mapping_changed = synthetic_spec();
        mapping_changed.transmissions[0].scale_rad_per_joint_unit = 90.0;
        let mapping_changed = ArmModel::compile_urdf(SYNTHETIC_6DOF, &mapping_changed).unwrap();
        assert_ne!(baseline.profile_id(), mapping_changed.profile_id());
    }

    #[test]
    fn mixed_fixture_supports_variable_dof_units_and_multiple_groups() {
        let model = ArmModel::compile_urdf(MIXED_MULTI_GROUP, &mixed_spec()).unwrap();
        assert_eq!(model.groups().len(), 2);
        assert_eq!(model.groups()[0].joint_order().len(), 2);
        assert_eq!(model.groups()[1].joint_order().len(), 1);
        assert!(
            model
                .joint(id("left_slide"))
                .unwrap()
                .kind()
                .active()
                .is_some()
        );
        assert!(model.links().iter().all(|link| link.inertia().is_none()));
    }

    #[test]
    fn explicit_group_order_cannot_silently_change() {
        let mut spec = mixed_spec();
        spec.groups[0].active_joint_order.swap(0, 1);
        assert!(matches!(
            ArmModel::compile_urdf(MIXED_MULTI_GROUP, &spec),
            Err(ArmModelError::GroupOrderMismatch { .. })
        ));
    }

    #[test]
    fn fixtures_cannot_enter_production_registry() {
        let model = ArmModel::compile_urdf(SYNTHETIC_6DOF, &synthetic_spec()).unwrap();
        let mut registry = ProductionProfileRegistry::default();
        assert!(registry.register(&model).is_err());
    }

    #[test]
    fn malformed_topology_limits_and_unsupported_joint_fail_closed() {
        let invalid_axis =
            SYNTHETIC_6DOF.replace(r#"<axis xyz="0 0 1"/>"#, r#"<axis xyz="0 0 0"/>"#);
        assert!(matches!(
            ArmModel::compile_urdf(&invalid_axis, &synthetic_spec()),
            Err(ArmModelError::InvalidAxis { .. })
        ));

        let floating = SYNTHETIC_6DOF.replacen(r#"type="revolute""#, r#"type="floating""#, 1);
        assert!(matches!(
            ArmModel::compile_urdf(&floating, &synthetic_spec()),
            Err(ArmModelError::UnsupportedJointType { .. })
        ));

        let mut missing_acceleration = synthetic_spec();
        missing_acceleration.acceleration_limits.pop();
        assert!(matches!(
            ArmModel::compile_urdf(SYNTHETIC_6DOF, &missing_acceleration),
            Err(ArmModelError::MissingAccelerationLimit { .. })
        ));
    }
}
