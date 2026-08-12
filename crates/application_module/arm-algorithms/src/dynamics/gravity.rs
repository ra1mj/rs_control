//! Allocation-free prepared gravity geometry and payload regressor.

use std::collections::BTreeMap;

use arm_model::ModelJointKind;
use arrayvec::ArrayVec;
use message_model::{
    ArmEffort, ArmJointId, ArmJointKind, ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId,
    KinematicGroupId, MAX_JOINTS,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

use super::super::kinematics::KinematicChain;
use crate::ArmAlgorithmError;

#[derive(Clone, Copy)]
struct PreparedJoint {
    joint_id: ArmJointId,
    origin: Isometry3<f64>,
    axis: Vector3<f64>,
    kind: Option<ArmJointKind>,
    column: Option<usize>,
    position_limit: Option<[f64; 2]>,
}

#[derive(Clone, Copy)]
struct PreparedLink {
    mass: f64,
    cog: Vector3<f64>,
}

/// Immutable geometry compiled for one profile/group generation.
#[derive(Clone)]
pub struct PreparedGravity {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    generation: u64,
    gravity: Vector3<f64>,
    joints: Vec<PreparedJoint>,
    links: Vec<PreparedLink>,
}

impl PreparedGravity {
    /// Compiles link CoGs and active joint frames once. Mimic dynamics are
    /// rejected because the dynamics adapter has the same fail-closed policy.
    pub fn new(
        chain: KinematicChain<'_>,
        gravity_mps2: [f64; 3],
        generation: u64,
    ) -> Result<Self, ArmAlgorithmError> {
        if generation == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "gravity.prepared_generation",
            });
        }
        if gravity_mps2.iter().any(|v| !v.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite {
                field: "gravity.gravity",
            });
        }
        let order = chain.group().joint_order().clone();
        let columns = order
            .joints()
            .iter()
            .enumerate()
            .map(|(index, descriptor)| (descriptor.joint_id(), index))
            .collect::<BTreeMap<_, _>>();
        let mut joints = Vec::with_capacity(chain.group().path_joint_indices().len());
        let mut links = Vec::with_capacity(chain.group().path_joint_indices().len());
        let mut active_count = 0_usize;
        for &joint_index in chain.group().path_joint_indices() {
            let joint =
                chain
                    .model()
                    .joints()
                    .get(joint_index)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "gravity path joint index is out of bounds",
                    })?;
            if joint.mimic().is_some() {
                return Err(ArmAlgorithmError::DynamicsUnavailable {
                    reason: "mimic-joint gravity reduction is not implemented",
                });
            }
            let origin = Isometry3::from_parts(
                Translation3::from(Vector3::from(joint.origin_xyz_m())),
                UnitQuaternion::from_euler_angles(
                    joint.origin_rpy_rad()[0],
                    joint.origin_rpy_rad()[1],
                    joint.origin_rpy_rad()[2],
                ),
            );
            let child =
                chain
                    .model()
                    .link(joint.child_link())
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "gravity path child link does not exist",
                    })?;
            let inertia = child
                .inertia()
                .ok_or(ArmAlgorithmError::DynamicsUnavailable {
                    reason: "selected path contains a link without inertia",
                })?;
            links.push(PreparedLink {
                mass: inertia.mass_kg(),
                cog: Vector3::from(inertia.origin_xyz_m()),
            });
            if let ModelJointKind::Active(kind) = joint.kind() {
                let column =
                    columns
                        .get(&joint.id())
                        .copied()
                        .ok_or(ArmAlgorithmError::InvalidModel {
                            reason: "gravity active joint is absent from group order",
                        })?;
                let axis = Unit::try_new(Vector3::from(joint.axis()), 1.0e-15)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "gravity active joint axis is invalid",
                    })?
                    .into_inner();
                joints.push(PreparedJoint {
                    joint_id: joint.id(),
                    origin,
                    axis,
                    kind: Some(kind),
                    column: Some(column),
                    position_limit: joint.limits().and_then(|limits| limits.position()),
                });
                active_count += 1;
            } else {
                joints.push(PreparedJoint {
                    joint_id: joint.id(),
                    origin,
                    axis: Vector3::zeros(),
                    kind: None,
                    column: None,
                    position_limit: None,
                });
            }
        }
        if active_count != order.len() {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "gravity generalized-coordinate order differs from path",
            });
        }
        // Keep link count aligned with path joints; all selected child links
        // have been checked above.
        Ok(Self {
            profile_id: chain.model().profile_id(),
            group_id: chain.group().id(),
            order,
            generation,
            gravity: Vector3::from(gravity_mps2),
            joints,
            links,
        })
    }

    /// Returns the generation this prepared evaluator is bound to.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns immutable group identity.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Allocates fixed-capacity warm-path scratch.
    pub fn workspace(&self) -> GravityWorkspace {
        GravityWorkspace {
            profile_id: self.profile_id,
            group_id: self.group_id,
            generation: self.generation,
            dof: self.order.len(),
            path_len: self.links.len(),
            transforms: vec![Isometry3::identity(); self.links.len() + 1],
            origins: vec![Vector3::zeros(); self.order.len()],
            axes: vec![Vector3::zeros(); self.order.len()],
            cogs: vec![Vector3::zeros(); self.links.len()],
            nominal: [0.0; MAX_JOINTS],
            regressor: vec![0.0; self.order.len() * 4],
        }
    }

    /// Evaluates nominal gravity compensation into a stable tagged vector.
    pub fn gravity_compensation(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        workspace: &mut GravityWorkspace,
    ) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
        self.evaluate(positions, workspace)?;
        effort_vector_f64(&self.order, &workspace.nominal[..self.order.len()])
    }

    /// Evaluates the row-major `dof x 4` payload regressor. Columns are
    /// `[mass, m*cx, m*cy, m*cz]` in the tool frame.
    pub fn payload_regressor(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        workspace: &mut GravityWorkspace,
        output: &mut [f64],
    ) -> Result<(), ArmAlgorithmError> {
        self.evaluate(positions, workspace)?;
        let required = self.order.len() * 4;
        if output.len() < required {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "gravity.regressor",
                expected: required,
                actual: output.len(),
            });
        }
        output[..required].copy_from_slice(&workspace.regressor);
        Ok(())
    }

    /// Computes payload gravity effort for `[m, m*cx, m*cy, m*cz]`.
    pub fn payload_gravity(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        theta: [f64; 4],
        workspace: &mut GravityWorkspace,
    ) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
        self.evaluate(positions, workspace)?;
        if theta.iter().any(|v| !v.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite {
                field: "gravity.theta",
            });
        }
        let values = workspace.regressor_values(theta, self.order.len());
        effort_vector_f64(&self.order, &values[..self.order.len()])
    }

    fn evaluate(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        workspace: &mut GravityWorkspace,
    ) -> Result<(), ArmAlgorithmError> {
        workspace.require(self)?;
        if positions.order() != &self.order {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        if positions.values().len() != self.order.len() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "gravity.positions",
                expected: self.order.len(),
                actual: positions.values().len(),
            });
        }
        workspace.nominal.fill(0.0);
        workspace.regressor.fill(0.0);
        workspace.transforms[0] = Isometry3::identity();
        let mut column_values = [0.0; MAX_JOINTS];
        for (index, (descriptor, position)) in self
            .order
            .joints()
            .iter()
            .zip(positions.values())
            .enumerate()
        {
            if descriptor.kind().coordinate_kind() != position.kind() {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            column_values[index] = position.value_si();
            if !column_values[index].is_finite() {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "gravity.positions",
                });
            }
        }
        // Reconstruct the path using the prepared joint frames. The prepared
        // evaluator stores one entry per active joint, while fixed joints are
        // folded into the following frame during compilation.
        let mut transform = Isometry3::identity();
        for (link_index, link) in self.links.iter().enumerate() {
            let joint = &self.joints[link_index];
            let frame = transform * joint.origin;
            transform = if let (Some(kind), Some(column)) = (joint.kind, joint.column) {
                let coordinate = column_values[column];
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
                workspace.origins[column] = frame.translation.vector;
                workspace.axes[column] = frame.rotation * joint.axis;
                let axis = Unit::new_normalize(joint.axis);
                frame
                    * match kind {
                        ArmJointKind::Revolute | ArmJointKind::Continuous => Isometry3::from_parts(
                            Translation3::identity(),
                            UnitQuaternion::from_axis_angle(&axis, coordinate),
                        ),
                        ArmJointKind::Prismatic => Isometry3::from_parts(
                            Translation3::from(axis.into_inner() * coordinate),
                            UnitQuaternion::identity(),
                        ),
                    }
            } else {
                frame
            };
            workspace.transforms[link_index + 1] = transform;
            workspace.cogs[link_index] = transform
                .transform_point(&nalgebra::Point3::from(link.cog))
                .coords;
        }
        let tip = workspace.transforms[self.links.len()];
        for (joint_index, joint) in self.joints.iter().enumerate() {
            let (Some(kind), Some(column)) = (joint.kind, joint.column) else {
                continue;
            };
            for (link_index, link) in self.links.iter().enumerate().skip(joint_index) {
                let com = workspace.cogs[link_index];
                let force = self.gravity * link.mass;
                let axis = workspace.axes[column];
                let origin = workspace.origins[column];
                let value = match kind {
                    ArmJointKind::Revolute | ArmJointKind::Continuous => {
                        -axis.dot(&((com - origin).cross(&force)))
                    }
                    ArmJointKind::Prismatic => -axis.dot(&force),
                };
                workspace.nominal[column] += value;
            }
        }
        let tip_com = tip.translation.vector;
        for joint in &self.joints {
            let (Some(kind), Some(column)) = (joint.kind, joint.column) else {
                continue;
            };
            let axis = workspace.axes[column];
            let origin = workspace.origins[column];
            let row = column * 4;
            workspace.regressor[row] = match kind {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    -axis.dot(&((tip_com - origin).cross(&self.gravity)))
                }
                ArmJointKind::Prismatic => -axis.dot(&self.gravity),
            };
            if matches!(kind, ArmJointKind::Revolute | ArmJointKind::Continuous) {
                for component in 0..3 {
                    let mut unit = Vector3::zeros();
                    unit[component] = 1.0;
                    let world = tip.rotation * unit;
                    workspace.regressor[row + component + 1] =
                        -axis.dot(&(world.cross(&self.gravity)));
                }
            }
        }
        Ok(())
    }
}

/// Caller-owned reusable scratch for one prepared gravity evaluator.
pub struct GravityWorkspace {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    dof: usize,
    path_len: usize,
    transforms: Vec<Isometry3<f64>>,
    origins: Vec<Vector3<f64>>,
    axes: Vec<Vector3<f64>>,
    cogs: Vec<Vector3<f64>>,
    nominal: [f64; MAX_JOINTS],
    regressor: Vec<f64>,
}

impl GravityWorkspace {
    fn require(&self, prepared: &PreparedGravity) -> Result<(), ArmAlgorithmError> {
        if self.generation != prepared.generation {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: prepared.generation,
                actual: self.generation,
            });
        }
        if self.profile_id != prepared.profile_id
            || self.group_id != prepared.group_id
            || self.dof != prepared.order.len()
            || self.path_len != prepared.links.len()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "gravity workspace identity or dimensions differ",
            });
        }
        Ok(())
    }

    fn regressor_values(&self, theta: [f64; 4], dof: usize) -> [f64; MAX_JOINTS] {
        let mut values = [0.0; MAX_JOINTS];
        for (row, value) in values[..dof].iter_mut().enumerate() {
            let base = row * 4;
            *value = self.regressor[base] * theta[0]
                + self.regressor[base + 1] * theta[1]
                + self.regressor[base + 2] * theta[2]
                + self.regressor[base + 3] * theta[3];
        }
        values
    }
}

fn effort_vector_f64(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
    let mut tagged = ArrayVec::<ArmEffort, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        if !value.is_finite() {
            return Err(ArmAlgorithmError::NonFinite {
                field: "gravity.effort",
            });
        }
        tagged.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(*value),
                ArmJointKind::Prismatic => ArmEffort::force_n(*value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "gravity.effort",
            })?,
        );
    }
    ArmJointVector::new(order.clone(), tagged).map_err(|_| ArmAlgorithmError::InvalidModel {
        reason: "gravity effort output dimension changed",
    })
}
