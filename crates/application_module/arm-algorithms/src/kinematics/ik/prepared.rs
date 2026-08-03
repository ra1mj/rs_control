// SPDX-License-Identifier: Apache-2.0
//
// Project-native prepared kinematics used by IK kernels adapted from
// softmata/kinetic commit 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// This integration and storage layout are local modifications; KINETIC types
// and runtime dependencies are intentionally not used.

use std::{collections::BTreeMap, f64::consts::PI};

use arm_model::{ArmJoint, ArmModel, ModelJointKind};
use message_model::{
    ArmJointId, ArmJointKind, ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId,
    KinematicGroupId, MAX_JOINTS,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use super::super::KinematicChain;
use crate::ArmAlgorithmError;

/// Hard upper bound for one prepared IK candidate set.
pub const MAX_IK_CANDIDATES: usize = 32;
/// Hard upper bound for a deterministic Bio-IK population.
pub const MAX_IK_POPULATION: usize = 256;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum SpecializedGeometry {
    None,
    SixAxis,
    SevenAxis,
}

/// Truthful geometry analysis computed once for a profile/group generation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GeometryCapabilities {
    all_rotary: bool,
    has_mimic: bool,
    opw: bool,
    paden_kahan: bool,
    specialized: SpecializedGeometry,
}

impl GeometryCapabilities {
    /// Returns whether all independent active coordinates are rotary.
    #[must_use]
    pub const fn all_rotary(self) -> bool {
        self.all_rotary
    }

    /// Returns whether the selected path contains a dependent mimic joint.
    #[must_use]
    pub const fn has_mimic(self) -> bool {
        self.has_mimic
    }

    /// Returns whether a six-axis analytical path passed the conservative
    /// topology precheck. Exact solver geometry is validated by the kernel.
    #[must_use]
    pub const fn supports_six_axis_analysis(self) -> bool {
        matches!(self.specialized, SpecializedGeometry::SixAxis)
    }

    /// Returns whether the stricter KINETIC OPW axis convention matched.
    #[must_use]
    pub const fn supports_opw(self) -> bool {
        self.opw
    }

    /// Returns whether a spherical-wrist Paden-Kahan decomposition matched.
    #[must_use]
    pub const fn supports_paden_kahan(self) -> bool {
        self.paden_kahan
    }

    /// Returns whether a redundant seven-axis analytical path passed the
    /// conservative topology precheck.
    #[must_use]
    pub const fn supports_seven_axis_analysis(self) -> bool {
        matches!(self.specialized, SpecializedGeometry::SevenAxis)
    }
}

#[derive(Clone, Copy)]
pub(super) struct CoordinateBinding {
    pub column: usize,
    pub multiplier: f64,
    pub offset: f64,
}

#[derive(Clone)]
pub(super) struct PreparedPathJoint {
    pub origin: Isometry3<f64>,
    pub axis: Vector3<f64>,
    pub kind: Option<ArmJointKind>,
    pub binding: Option<CoordinateBinding>,
    pub joint_id: ArmJointId,
    pub position_limit: Option<[f64; 2]>,
}

#[derive(Clone, Copy, Debug)]
pub(super) struct JointBounds {
    pub position: Option<[f64; 2]>,
    pub sampling: [f64; 2],
}

/// Immutable, morphology-bound data shared by all KINETIC-derived IK kernels.
#[derive(Clone)]
pub struct PreparedKinematics<'model> {
    pub(super) chain: KinematicChain<'model>,
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    generation: u64,
    pub(super) path: Vec<PreparedPathJoint>,
    pub(super) bounds: Vec<JointBounds>,
    pub(super) active_path_indices: Vec<usize>,
    pub(super) zero_axes: Vec<Vector3<f64>>,
    pub(super) zero_origins: Vec<Vector3<f64>>,
    pub(super) zero_tip: Isometry3<f64>,
    pub(super) wrist_center: Option<Vector3<f64>>,
    geometry: GeometryCapabilities,
}

impl<'model> PreparedKinematics<'model> {
    /// Prepares bounded chain geometry for one non-zero model generation.
    pub fn new(chain: KinematicChain<'model>, generation: u64) -> Result<Self, ArmAlgorithmError> {
        if generation == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "ik.prepared_generation",
            });
        }
        let independent = chain
            .group()
            .joint_order()
            .joints()
            .iter()
            .enumerate()
            .map(|(column, descriptor)| (descriptor.joint_id(), column))
            .collect::<BTreeMap<_, _>>();
        let mut path = Vec::with_capacity(chain.group().path_joint_indices().len());
        let mut active_path_indices = vec![usize::MAX; chain.dof()];
        let mut has_mimic = false;

        for (path_index, &joint_index) in chain.group().path_joint_indices().iter().enumerate() {
            let joint =
                chain
                    .model()
                    .joints()
                    .get(joint_index)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "prepared path joint index is out of bounds",
                    })?;
            let origin = Isometry3::from_parts(
                Translation3::from(Vector3::from(joint.origin_xyz_m())),
                UnitQuaternion::from_euler_angles(
                    joint.origin_rpy_rad()[0],
                    joint.origin_rpy_rad()[1],
                    joint.origin_rpy_rad()[2],
                ),
            );
            let kind = joint.kind().active();
            let binding = if kind.is_some() {
                let binding = resolve_binding(chain.model(), joint, &independent, &mut Vec::new())?;
                has_mimic |= joint.mimic().is_some();
                if active_path_indices[binding.column] == usize::MAX {
                    active_path_indices[binding.column] = path_index;
                }
                Some(binding)
            } else {
                None
            };
            path.push(PreparedPathJoint {
                origin,
                axis: Vector3::from(joint.axis()),
                kind,
                binding,
                joint_id: joint.id(),
                position_limit: joint.limits().and_then(|limits| limits.position()),
            });
        }
        if active_path_indices.contains(&usize::MAX) {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "prepared path omits an independent active joint",
            });
        }

        let mut bounds = Vec::with_capacity(chain.dof());
        let mut all_rotary = true;
        for descriptor in chain.group().joint_order().joints() {
            let joint = chain.model().joint(descriptor.joint_id()).ok_or(
                ArmAlgorithmError::InvalidModel {
                    reason: "prepared joint order references an unknown joint",
                },
            )?;
            let position = joint.limits().and_then(|limits| limits.position());
            let sampling = position.unwrap_or([-PI, PI]);
            bounds.push(JointBounds { position, sampling });
            all_rotary &= matches!(
                descriptor.kind(),
                ArmJointKind::Revolute | ArmJointKind::Continuous
            );
        }
        let ZeroGeometry {
            axes: zero_axes,
            origins: zero_origins,
            tip: zero_tip,
        } = zero_geometry(&path, chain.dof())?;
        let wrist_start = chain.dof().saturating_sub(3);
        let wrist_center = if all_rotary && !has_mimic && chain.dof() >= 6 {
            find_axis_intersection(
                &zero_axes[wrist_start..],
                &zero_origins[wrist_start..],
                1.0e-4,
            )
        } else {
            None
        };
        let paden_kahan = chain.dof() == 6
            && wrist_center.is_some()
            && body_wrist_convention(&zero_axes, zero_tip, 3, WristConvention::Zyz);
        let opw = chain.dof() == 6
            && wrist_center.is_some()
            && body_wrist_convention(&zero_axes, zero_tip, 3, WristConvention::Yzy)
            && opw_axis_convention(&zero_axes, &zero_origins, wrist_center);
        let seven_axis = chain.dof() == 7
            && wrist_center.is_some()
            && body_wrist_convention(&zero_axes, zero_tip, 4, WristConvention::Zyz);
        let specialized = if paden_kahan || opw {
            SpecializedGeometry::SixAxis
        } else if all_rotary && !has_mimic && seven_axis {
            SpecializedGeometry::SevenAxis
        } else {
            SpecializedGeometry::None
        };

        Ok(Self {
            chain,
            profile_id: chain.model().profile_id(),
            group_id: chain.group().id(),
            order: chain.group().joint_order().clone(),
            generation,
            path,
            bounds,
            active_path_indices,
            zero_axes,
            zero_origins,
            zero_tip,
            wrist_center,
            geometry: GeometryCapabilities {
                all_rotary,
                has_mimic,
                opw,
                paden_kahan,
                specialized,
            },
        })
    }

    /// Returns the immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact selected group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the stable active-joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns the caller-managed model generation.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns the validated chain used by downstream prepared algorithms.
    #[must_use]
    pub(crate) const fn chain(&self) -> KinematicChain<'model> {
        self.chain
    }

    /// Returns one preparation-time sampling interval in stable joint order.
    #[must_use]
    pub(crate) fn sampling_bound(&self, index: usize) -> [f64; 2] {
        self.bounds[index].sampling
    }

    /// Returns active degrees of freedom.
    #[must_use]
    pub fn dof(&self) -> usize {
        self.order.len()
    }

    /// Returns conservative specialized-geometry evidence.
    #[must_use]
    pub const fn geometry(&self) -> GeometryCapabilities {
        self.geometry
    }

    /// Allocates all reusable scratch storage once.
    pub fn workspace(&self, maximum_population: usize) -> Result<IkWorkspace, ArmAlgorithmError> {
        IkWorkspace::new(self, maximum_population)
    }

    /// Allocates a workspace only when its deterministic storage estimate fits
    /// the caller's preparation-time byte ceiling.
    pub fn workspace_with_byte_limit(
        &self,
        maximum_population: usize,
        maximum_bytes: usize,
    ) -> Result<IkWorkspace, ArmAlgorithmError> {
        let required = IkWorkspace::required_bytes(self, maximum_population)?;
        if required > maximum_bytes {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.workspace_bytes",
                actual: required,
                maximum: maximum_bytes,
            });
        }
        IkWorkspace::new(self, maximum_population)
    }

    pub(crate) fn validate_values(&self, values: &[f64]) -> Result<(), ArmAlgorithmError> {
        if values.len() != self.dof() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "ik.coordinates",
                expected: self.dof(),
                actual: values.len(),
            });
        }
        for (index, value) in values.iter().copied().enumerate() {
            if !value.is_finite() {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "ik.coordinates",
                });
            }
            if let Some([lower, upper]) = self.bounds[index].position
                && !(lower..=upper).contains(&value)
            {
                return Err(ArmAlgorithmError::JointLimit {
                    joint: self.order.joints()[index].joint_id(),
                    value,
                    lower,
                    upper,
                });
            }
        }
        Ok(())
    }

    pub(super) fn copy_positions(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        output: &mut [f64; MAX_JOINTS],
    ) -> Result<(), ArmAlgorithmError> {
        if positions.order() != &self.order {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        for (index, (descriptor, value)) in self
            .order
            .joints()
            .iter()
            .zip(positions.values())
            .enumerate()
        {
            if descriptor.kind().coordinate_kind() != value.kind() {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            output[index] = value.value_si();
        }
        self.validate_values(&output[..self.dof()])
    }

    pub(super) fn project_limits(&self, values: &mut [f64]) {
        for (value, bound) in values.iter_mut().zip(&self.bounds) {
            if let Some([lower, upper]) = bound.position {
                *value = value.clamp(lower, upper);
            }
        }
    }

    pub(super) fn evaluate(
        &self,
        workspace: &mut IkWorkspace,
        values: &[f64],
    ) -> Result<Isometry3<f64>, ArmAlgorithmError> {
        workspace.require_context(self)?;
        self.validate_values(values)?;
        workspace.frames.clear();
        workspace.link_points[0] = Vector3::zeros();
        let mut transform = Isometry3::identity();

        for (path_index, joint) in self.path.iter().enumerate() {
            let frame = transform * joint.origin;
            if let (Some(kind), Some(binding)) = (joint.kind, joint.binding) {
                let coordinate = binding
                    .multiplier
                    .mul_add(values[binding.column], binding.offset);
                if !coordinate.is_finite() {
                    return Err(ArmAlgorithmError::NonFinite {
                        field: "ik.mimic_coordinate",
                    });
                }
                if let Some([lower, upper]) = joint.position_limit
                    && !(lower..=upper).contains(&coordinate)
                {
                    return Err(ArmAlgorithmError::JointLimit {
                        joint: joint.joint_id,
                        value: coordinate,
                        lower,
                        upper,
                    });
                }
                let axis =
                    Unit::try_new(joint.axis, 1.0e-15).ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "prepared active joint axis is invalid",
                    })?;
                workspace.frames.push(EvaluatedFrame {
                    frame,
                    axis: joint.axis,
                    kind,
                    binding,
                });
                transform = match kind {
                    ArmJointKind::Revolute | ArmJointKind::Continuous => {
                        frame
                            * Isometry3::from_parts(
                                Translation3::identity(),
                                UnitQuaternion::from_axis_angle(&axis, coordinate),
                            )
                    }
                    ArmJointKind::Prismatic => {
                        frame
                            * Isometry3::from_parts(
                                Translation3::from(axis.into_inner() * coordinate),
                                UnitQuaternion::identity(),
                            )
                    }
                };
            } else {
                transform = frame;
            }
            workspace.link_points[path_index + 1] = transform.translation.vector;
        }

        workspace.jacobian.fill(0.0);
        let tip = transform.translation.vector;
        for evaluated in &workspace.frames {
            let world_axis = evaluated.frame.rotation * evaluated.axis;
            let origin = evaluated.frame.translation.vector;
            let (linear, angular) = match evaluated.kind {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    (world_axis.cross(&(tip - origin)), world_axis)
                }
                ArmJointKind::Prismatic => (world_axis, Vector3::zeros()),
            };
            let column = evaluated.binding.column;
            let scale = evaluated.binding.multiplier;
            for row in 0..3 {
                workspace.jacobian[row * self.dof() + column] += linear[row] * scale;
                workspace.jacobian[(row + 3) * self.dof() + column] += angular[row] * scale;
            }
        }
        if workspace.jacobian.iter().any(|value| !value.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite {
                field: "ik.jacobian",
            });
        }
        Ok(transform)
    }
}

struct ZeroGeometry {
    axes: Vec<Vector3<f64>>,
    origins: Vec<Vector3<f64>>,
    tip: Isometry3<f64>,
}

fn zero_geometry(
    path: &[PreparedPathJoint],
    dof: usize,
) -> Result<ZeroGeometry, ArmAlgorithmError> {
    let mut axes = vec![Vector3::zeros(); dof];
    let mut origins = vec![Vector3::zeros(); dof];
    let mut seen = vec![false; dof];
    let mut transform = Isometry3::identity();
    for joint in path {
        transform *= joint.origin;
        if let Some(binding) = joint.binding {
            if seen[binding.column] {
                continue;
            }
            let axis = transform.rotation * joint.axis;
            axes[binding.column] =
                axis.try_normalize(1.0e-15)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "prepared zero-configuration axis is invalid",
                    })?;
            origins[binding.column] = transform.translation.vector;
            seen[binding.column] = true;
        }
    }
    if seen.contains(&false) {
        return Err(ArmAlgorithmError::InvalidModel {
            reason: "prepared zero geometry omits an independent joint",
        });
    }
    Ok(ZeroGeometry {
        axes,
        origins,
        tip: transform,
    })
}

fn opw_axis_convention(
    axes: &[Vector3<f64>],
    origins: &[Vector3<f64>],
    wrist_center: Option<Vector3<f64>>,
) -> bool {
    let Some(wrist_center) = wrist_center else {
        return false;
    };
    let z_axis = axes[0];
    let y_axis = axes[1];
    let Some(x_axis) = y_axis.cross(&z_axis).try_normalize(1.0e-10) else {
        return false;
    };
    let upper_arm = origins[2] - origins[0];
    let forearm = wrist_center - origins[2];
    axes[2].dot(&y_axis) > 1.0 - 1.0e-6
        && upper_arm.dot(&(-x_axis)) > 1.0e-8
        && upper_arm.dot(&y_axis).abs() < 1.0e-6
        && upper_arm.dot(&z_axis).abs() < 1.0e-6
        && forearm.dot(&y_axis).abs() < 1.0e-6
}

#[derive(Clone, Copy)]
enum WristConvention {
    Yzy,
    Zyz,
}

fn body_wrist_convention(
    axes: &[Vector3<f64>],
    zero_tip: Isometry3<f64>,
    start: usize,
    convention: WristConvention,
) -> bool {
    let expected = match convention {
        WristConvention::Yzy => [Vector3::y(), Vector3::z(), Vector3::y()],
        WristConvention::Zyz => [Vector3::z(), Vector3::y(), Vector3::z()],
    };
    (0..3).all(|offset| {
        let body_axis = zero_tip.rotation.inverse() * axes[start + offset];
        body_axis.dot(&expected[offset]) > 1.0 - 1.0e-6
    })
}

fn find_axis_intersection(
    axes: &[Vector3<f64>],
    origins: &[Vector3<f64>],
    tolerance: f64,
) -> Option<Vector3<f64>> {
    let mut sum = Vector3::zeros();
    let mut count = 0_usize;
    for left in 0..axes.len() {
        for right in left + 1..axes.len() {
            let (distance, first, second) =
                closest_points_on_lines(origins[left], axes[left], origins[right], axes[right])?;
            if distance > tolerance {
                return None;
            }
            sum += (first + second) * 0.5;
            count += 1;
        }
    }
    (count > 0).then(|| sum / count as f64)
}

fn closest_points_on_lines(
    first_origin: Vector3<f64>,
    first_axis: Vector3<f64>,
    second_origin: Vector3<f64>,
    second_axis: Vector3<f64>,
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let offset = first_origin - second_origin;
    let a = first_axis.dot(&first_axis);
    let b = first_axis.dot(&second_axis);
    let c = second_axis.dot(&second_axis);
    let d = first_axis.dot(&offset);
    let e = second_axis.dot(&offset);
    let denominator = a * c - b * b;
    if denominator.abs() < 1.0e-15 {
        // Parallel coincident axes are acceptable for a spherical wrist;
        // distinct parallel axes are not.
        let perpendicular = offset - first_axis * first_axis.dot(&offset);
        if perpendicular.norm() > 1.0e-4 {
            return None;
        }
        return Some((0.0, first_origin, first_origin));
    }
    let first_t = (b * e - c * d) / denominator;
    let second_t = (a * e - b * d) / denominator;
    let first = first_origin + first_axis * first_t;
    let second = second_origin + second_axis * second_t;
    Some(((first - second).norm(), first, second))
}

#[derive(Clone, Copy)]
pub(super) struct EvaluatedFrame {
    frame: Isometry3<f64>,
    axis: Vector3<f64>,
    kind: ArmJointKind,
    binding: CoordinateBinding,
}

#[derive(Clone)]
pub(super) struct PopulationMember {
    pub coordinates: [f64; MAX_JOINTS],
    pub fitness: f64,
}

impl Default for PopulationMember {
    fn default() -> Self {
        Self {
            coordinates: [0.0; MAX_JOINTS],
            fitness: f64::INFINITY,
        }
    }
}

/// Single-owner reusable memory for warm IK calls.
pub struct IkWorkspace {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    dof: usize,
    path_len: usize,
    pub(super) frames: Vec<EvaluatedFrame>,
    pub(super) link_points: Vec<Vector3<f64>>,
    pub(super) probe_points: Vec<Vector3<f64>>,
    pub(super) link_lengths: Vec<f64>,
    pub(super) jacobian: Vec<f64>,
    pub(super) linear_system: Vec<f64>,
    pub(super) delta: [f64; MAX_JOINTS],
    pub(super) population: Vec<PopulationMember>,
    pub(super) next_population: Vec<PopulationMember>,
    pub(super) rng: ChaCha8Rng,
}

impl IkWorkspace {
    fn required_bytes(
        prepared: &PreparedKinematics<'_>,
        maximum_population: usize,
    ) -> Result<usize, ArmAlgorithmError> {
        if !(2..=MAX_IK_POPULATION).contains(&maximum_population) {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.maximum_population",
                actual: maximum_population,
                maximum: MAX_IK_POPULATION,
            });
        }
        let path = prepared.path.len();
        let dof = prepared.dof();
        Ok(std::mem::size_of::<Self>()
            + path * std::mem::size_of::<EvaluatedFrame>()
            + (path + 1) * 2 * std::mem::size_of::<Vector3<f64>>()
            + path * std::mem::size_of::<f64>()
            + 6 * dof * std::mem::size_of::<f64>()
            + dof * (dof + 1) * std::mem::size_of::<f64>()
            + maximum_population * 2 * std::mem::size_of::<PopulationMember>())
    }

    fn new(
        prepared: &PreparedKinematics<'_>,
        maximum_population: usize,
    ) -> Result<Self, ArmAlgorithmError> {
        if !(2..=MAX_IK_POPULATION).contains(&maximum_population) {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.maximum_population",
                actual: maximum_population,
                maximum: MAX_IK_POPULATION,
            });
        }
        Ok(Self {
            profile_id: prepared.profile_id,
            group_id: prepared.group_id,
            generation: prepared.generation,
            dof: prepared.dof(),
            path_len: prepared.path.len(),
            frames: Vec::with_capacity(prepared.path.len()),
            link_points: vec![Vector3::zeros(); prepared.path.len() + 1],
            probe_points: vec![Vector3::zeros(); prepared.path.len() + 1],
            link_lengths: vec![0.0; prepared.path.len()],
            jacobian: vec![0.0; 6 * prepared.dof()],
            linear_system: vec![0.0; prepared.dof() * (prepared.dof() + 1)],
            delta: [0.0; MAX_JOINTS],
            population: vec![PopulationMember::default(); maximum_population],
            next_population: vec![PopulationMember::default(); maximum_population],
            rng: ChaCha8Rng::from_seed([0; 32]),
        })
    }

    /// Returns the fixed maximum population allocated at preparation.
    #[must_use]
    pub fn maximum_population(&self) -> usize {
        self.population.len()
    }

    /// Returns the deterministic preparation-time storage estimate in bytes.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        std::mem::size_of::<Self>()
            + self.frames.capacity() * std::mem::size_of::<EvaluatedFrame>()
            + self.link_points.capacity() * std::mem::size_of::<Vector3<f64>>()
            + self.probe_points.capacity() * std::mem::size_of::<Vector3<f64>>()
            + self.link_lengths.capacity() * std::mem::size_of::<f64>()
            + self.jacobian.capacity() * std::mem::size_of::<f64>()
            + self.linear_system.capacity() * std::mem::size_of::<f64>()
            + self.population.capacity() * std::mem::size_of::<PopulationMember>()
            + self.next_population.capacity() * std::mem::size_of::<PopulationMember>()
    }

    pub(super) fn require_context(
        &self,
        prepared: &PreparedKinematics<'_>,
    ) -> Result<(), ArmAlgorithmError> {
        if self.generation != prepared.generation {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: prepared.generation,
                actual: self.generation,
            });
        }
        if self.profile_id != prepared.profile_id || self.group_id != prepared.group_id {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "profile or group identity differs",
            });
        }
        if self.dof != prepared.dof() || self.path_len != prepared.path.len() {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "prepared dimensions differ",
            });
        }
        Ok(())
    }
}

fn resolve_binding(
    model: &ArmModel,
    joint: &ArmJoint,
    independent: &BTreeMap<ArmJointId, usize>,
    visited: &mut Vec<ArmJointId>,
) -> Result<CoordinateBinding, ArmAlgorithmError> {
    if let Some(column) = independent.get(&joint.id()).copied() {
        return Ok(CoordinateBinding {
            column,
            multiplier: 1.0,
            offset: 0.0,
        });
    }
    if visited.contains(&joint.id()) {
        return Err(ArmAlgorithmError::InvalidModel {
            reason: "mimic cycle passed model compilation",
        });
    }
    visited.push(joint.id());
    let mimic = joint
        .mimic()
        .ok_or(ArmAlgorithmError::MimicSourceUnavailable {
            joint: Box::new(joint.id()),
            source_joint: Box::new(joint.id()),
        })?;
    let source =
        model
            .joint(mimic.source_joint())
            .ok_or(ArmAlgorithmError::MimicSourceUnavailable {
                joint: Box::new(joint.id()),
                source_joint: Box::new(mimic.source_joint()),
            })?;
    if source.kind() == ModelJointKind::Fixed {
        return Err(ArmAlgorithmError::MimicSourceUnavailable {
            joint: Box::new(joint.id()),
            source_joint: Box::new(mimic.source_joint()),
        });
    }
    let source_binding = resolve_binding(model, source, independent, visited)?;
    visited.pop();
    Ok(CoordinateBinding {
        column: source_binding.column,
        multiplier: mimic.multiplier() * source_binding.multiplier,
        offset: mimic
            .multiplier()
            .mul_add(source_binding.offset, mimic.offset()),
    })
}
