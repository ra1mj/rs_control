//! Strict project adapter for articulated-body dynamics.
//!
//! The backend uses `f32`; every project input is validated in `f64`, checked
//! during conversion, and every result is converted back and revalidated.

use arm_model::{LinkInertia, ModelJointKind};
use arrayvec::ArrayVec;
use featherstone::prelude::{
    ArticulatedBody, GenJoint, SpatialInertia, SpatialTransform, aba_forward_dynamics,
    crba_mass_matrix, gravity_compensation, rnea_inverse_dynamics,
};
use message_model::{
    ArmAcceleration, ArmEffort, ArmJointKind, ArmJointOrder, ArmJointVector, ArmPosition,
    ArmProfileId, ArmVelocity, KinematicGroupId, MAX_JOINTS,
};
use nalgebra::DMatrix;
use nalgebra_featherstone::{Matrix3, Rotation3, Vector3};

use crate::{
    ArmAlgorithmError, TrajectoryViolationKind, kinematics::KinematicChain,
    trajectory::ArmTrajectory,
};

/// Validated symmetric group mass matrix in row-major order.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmMassMatrix {
    dof: usize,
    data: Vec<f64>,
}

impl ArmMassMatrix {
    /// Returns the exact matrix dimension.
    #[must_use]
    pub const fn dof(&self) -> usize {
        self.dof
    }

    /// Returns one matrix entry.
    #[must_use]
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row < self.dof && column < self.dof {
            Some(self.data[row * self.dof + column])
        } else {
            None
        }
    }

    /// Returns validated row-major storage.
    #[must_use]
    pub fn as_row_major(&self) -> &[f64] {
        &self.data
    }

    fn as_matrix(&self) -> DMatrix<f64> {
        DMatrix::from_row_slice(self.dof, self.dof, &self.data)
    }
}

/// Effort-feasibility evidence for a completely timed trajectory.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EffortFeasibility {
    evaluated_points: usize,
    maximum_utilization: f64,
}

impl EffortFeasibility {
    /// Returns the number of evaluated timed points.
    #[must_use]
    pub const fn evaluated_points(self) -> usize {
        self.evaluated_points
    }

    /// Returns maximum `abs(effort)/limit`.
    #[must_use]
    pub const fn maximum_utilization(self) -> f64 {
        self.maximum_utilization
    }
}

/// Compiled strict dynamics adapter for one selected open-chain group.
#[derive(Clone, Debug)]
pub struct ArmDynamics {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    backend: ArticulatedBody,
    effort_limits: Vec<f64>,
}

impl ArmDynamics {
    /// Builds a backend only when every selected link has valid inertia.
    pub fn new(
        chain: KinematicChain<'_>,
        gravity_mps2: [f64; 3],
    ) -> Result<Self, ArmAlgorithmError> {
        if gravity_mps2.iter().any(|value| !value.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite {
                field: "dynamics.gravity",
            });
        }
        let mut backend = ArticulatedBody::new();
        backend.set_gravity(vector3_f32(gravity_mps2, "dynamics.gravity")?);

        let base_link = chain.model().link(chain.group().base_link()).ok_or(
            ArmAlgorithmError::InvalidModel {
                reason: "dynamics group base link does not exist",
            },
        )?;
        let base_inertia = base_link
            .inertia()
            .ok_or(ArmAlgorithmError::DynamicsUnavailable {
                reason: "group base link has no inertia",
            })?;
        let base_index = backend.add_body(
            base_link.id().as_str(),
            -1,
            GenJoint::Fixed,
            spatial_inertia(base_inertia)?,
            SpatialTransform::identity(),
        );
        if base_index != 0 {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "backend base insertion order changed",
            });
        }

        let mut parent_body = base_index;
        let mut effort_limits = Vec::with_capacity(chain.dof());
        for &joint_index in chain.group().path_joint_indices() {
            let joint =
                chain
                    .model()
                    .joints()
                    .get(joint_index)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "dynamics path joint index is out of bounds",
                    })?;
            if joint.mimic().is_some() {
                return Err(ArmAlgorithmError::DynamicsUnavailable {
                    reason: "mimic-joint dynamics reduction is not implemented",
                });
            }
            let child =
                chain
                    .model()
                    .link(joint.child_link())
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "dynamics path child link does not exist",
                    })?;
            let inertia = child
                .inertia()
                .ok_or(ArmAlgorithmError::DynamicsUnavailable {
                    reason: "selected path contains a link without inertia",
                })?;
            let joint_type = match joint.kind() {
                ModelJointKind::Fixed => GenJoint::Fixed,
                ModelJointKind::Active(ArmJointKind::Revolute | ArmJointKind::Continuous) => {
                    GenJoint::Revolute {
                        axis: vector3_f32(joint.axis(), "dynamics.joint_axis")?,
                    }
                }
                ModelJointKind::Active(ArmJointKind::Prismatic) => GenJoint::Prismatic {
                    axis: vector3_f32(joint.axis(), "dynamics.joint_axis")?,
                },
            };
            if let Some(limits) = joint.limits() {
                effort_limits.push(limits.max_effort());
            }
            let rotation = Rotation3::from_euler_angles(
                f32_checked(joint.origin_rpy_rad()[0], "dynamics.origin_rpy")?,
                f32_checked(joint.origin_rpy_rad()[1], "dynamics.origin_rpy")?,
                f32_checked(joint.origin_rpy_rad()[2], "dynamics.origin_rpy")?,
            )
            .into_inner();
            let transform = SpatialTransform::from_rotation_translation(
                rotation,
                vector3_f32(joint.origin_xyz_m(), "dynamics.origin_xyz")?,
            );
            let body_index = backend.add_body(
                child.id().as_str(),
                parent_body as i32,
                joint_type,
                spatial_inertia(inertia)?,
                transform,
            );
            if body_index <= parent_body {
                return Err(ArmAlgorithmError::InvalidModel {
                    reason: "backend parent ordering invariant failed",
                });
            }
            parent_body = body_index;
        }
        if backend.dof_count() != chain.dof() || effort_limits.len() != chain.dof() {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "backend generalized-coordinate order differs from group order",
            });
        }
        Ok(Self {
            profile_id: chain.model().profile_id(),
            group_id: chain.group().id(),
            order: chain.group().joint_order().clone(),
            backend,
            effort_limits,
        })
    }

    /// Returns the immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact compiled group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Computes `tau = M(q) qdd + C(q,qd) + g(q)` with RNEA.
    pub fn inverse_dynamics(
        &mut self,
        positions: &ArmJointVector<ArmPosition>,
        velocities: &ArmJointVector<ArmVelocity>,
        accelerations: &ArmJointVector<ArmAcceleration>,
    ) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
        let q = position_values(&self.order, positions)?;
        let qd = velocity_values(&self.order, velocities)?;
        let qdd = acceleration_values(&self.order, accelerations)?;
        set_backend_state(&mut self.backend, &q, &qd, Some(&qdd), None)?;
        let (effort, _) = rnea_inverse_dynamics(&self.backend);
        effort_vector(&self.order, effort.as_slice())
    }

    /// Computes zero-velocity/zero-acceleration gravity compensation.
    pub fn gravity_compensation(
        &mut self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
        let q = position_values(&self.order, positions)?;
        let zeros = vec![0.0; self.order.len()];
        set_backend_state(&mut self.backend, &q, &zeros, Some(&zeros), None)?;
        let effort = gravity_compensation(&self.backend);
        effort_vector(&self.order, effort.as_slice())
    }

    /// Computes the CRBA mass matrix and validates symmetry/positive definiteness.
    pub fn mass_matrix(
        &mut self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<ArmMassMatrix, ArmAlgorithmError> {
        let q = position_values(&self.order, positions)?;
        let zeros = vec![0.0; self.order.len()];
        set_backend_state(&mut self.backend, &q, &zeros, None, None)?;
        let backend = crba_mass_matrix(&self.backend);
        let data = backend
            .iter()
            .copied()
            .map(|value| output_f64(value, "dynamics.mass_matrix"))
            .collect::<Result<Vec<_>, _>>()?;
        let matrix = ArmMassMatrix {
            dof: self.order.len(),
            data,
        };
        validate_mass_matrix(&matrix)?;
        Ok(matrix)
    }

    /// Computes generalized acceleration with ABA.
    pub fn forward_dynamics(
        &mut self,
        positions: &ArmJointVector<ArmPosition>,
        velocities: &ArmJointVector<ArmVelocity>,
        efforts: &ArmJointVector<ArmEffort>,
    ) -> Result<ArmJointVector<ArmAcceleration>, ArmAlgorithmError> {
        let q = position_values(&self.order, positions)?;
        let qd = velocity_values(&self.order, velocities)?;
        let tau = effort_values(&self.order, efforts)?;
        set_backend_state(&mut self.backend, &q, &qd, None, Some(&tau))?;
        // Validate the matrix before entering a backend solve.
        let matrix = crba_mass_matrix(&self.backend);
        let matrix_f64 = ArmMassMatrix {
            dof: self.order.len(),
            data: matrix.iter().map(|value| f64::from(*value)).collect(),
        };
        validate_mass_matrix(&matrix_f64)?;
        let acceleration = aba_forward_dynamics(&mut self.backend).clone();
        acceleration_vector(&self.order, acceleration.as_slice())
    }

    /// Computes `sqrt(det(J M^-1 J^T))`, rejecting singular operational inertia.
    pub fn dynamic_manipulability(
        &mut self,
        chain: KinematicChain<'_>,
        positions: &ArmJointVector<ArmPosition>,
        singular_tolerance: f64,
    ) -> Result<f64, ArmAlgorithmError> {
        self.ensure_chain(chain)?;
        if !singular_tolerance.is_finite() || singular_tolerance <= 0.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "dynamics.singular_tolerance",
            });
        }
        let mass = self.mass_matrix(positions)?.as_matrix();
        let inverse = mass.try_inverse().ok_or(ArmAlgorithmError::Singular {
            rank: 0,
            required_rank: self.order.len(),
        })?;
        let jacobian = chain.jacobian(positions)?.as_matrix();
        let operational = &jacobian * inverse * jacobian.transpose();
        let eigenvalues = operational.symmetric_eigen().eigenvalues;
        let rank = eigenvalues
            .iter()
            .filter(|value| **value > singular_tolerance)
            .count();
        if rank < 6 {
            return Err(ArmAlgorithmError::Singular {
                rank,
                required_rank: 6,
            });
        }
        let determinant = eigenvalues.iter().product::<f64>();
        if !determinant.is_finite() || determinant <= 0.0 {
            return Err(ArmAlgorithmError::Singular {
                rank,
                required_rank: 6,
            });
        }
        Ok(determinant.sqrt())
    }

    /// Validates inverse-dynamics effort for every timed trajectory point.
    pub fn check_trajectory_effort(
        &mut self,
        trajectory: &ArmTrajectory,
    ) -> Result<EffortFeasibility, ArmAlgorithmError> {
        if trajectory.profile_id() != self.profile_id
            || trajectory.group_id() != self.group_id
            || trajectory.order() != &self.order
        {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        let mut maximum_utilization = 0.0_f64;
        for (point_index, point) in trajectory.points().iter().enumerate() {
            let positions = position_vector(&self.order, point.positions_si())?;
            let velocities = velocity_vector(&self.order, point.velocities_si())?;
            let accelerations = acceleration_vector_f64(&self.order, point.accelerations_si())?;
            let effort = self.inverse_dynamics(&positions, &velocities, &accelerations)?;
            for (joint_index, value) in effort.values().iter().enumerate() {
                let utilization = value.value_si().abs() / self.effort_limits[joint_index];
                maximum_utilization = maximum_utilization.max(utilization);
                if utilization > 1.0 {
                    return Err(ArmAlgorithmError::TrajectoryViolation {
                        kind: TrajectoryViolationKind::Effort,
                        point_index,
                        joint: Box::new(self.order.joints()[joint_index].joint_id()),
                        value: value.value_si(),
                        limit: self.effort_limits[joint_index],
                    });
                }
            }
        }
        Ok(EffortFeasibility {
            evaluated_points: trajectory.points().len(),
            maximum_utilization,
        })
    }

    fn ensure_chain(&self, chain: KinematicChain<'_>) -> Result<(), ArmAlgorithmError> {
        if chain.model().profile_id() != self.profile_id
            || chain.group().id() != self.group_id
            || chain.group().joint_order() != &self.order
        {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        Ok(())
    }
}

fn spatial_inertia(inertia: LinkInertia) -> Result<SpatialInertia, ArmAlgorithmError> {
    let [ixx, ixy, ixz, iyy, iyz, izz] = inertia.inertia_kg_m2();
    let inertia_frame = Matrix3::new(
        f32_checked(ixx, "dynamics.inertia")?,
        f32_checked(ixy, "dynamics.inertia")?,
        f32_checked(ixz, "dynamics.inertia")?,
        f32_checked(ixy, "dynamics.inertia")?,
        f32_checked(iyy, "dynamics.inertia")?,
        f32_checked(iyz, "dynamics.inertia")?,
        f32_checked(ixz, "dynamics.inertia")?,
        f32_checked(iyz, "dynamics.inertia")?,
        f32_checked(izz, "dynamics.inertia")?,
    );
    let rpy = inertia.origin_rpy_rad();
    let rotation = Rotation3::from_euler_angles(
        f32_checked(rpy[0], "dynamics.inertia_rpy")?,
        f32_checked(rpy[1], "dynamics.inertia_rpy")?,
        f32_checked(rpy[2], "dynamics.inertia_rpy")?,
    )
    .into_inner();
    let body_inertia = rotation * inertia_frame * rotation.transpose();
    Ok(SpatialInertia::from_mass_inertia(
        f32_checked(inertia.mass_kg(), "dynamics.mass")?,
        vector3_f32(inertia.origin_xyz_m(), "dynamics.com")?,
        body_inertia,
    ))
}

fn set_backend_state(
    backend: &mut ArticulatedBody,
    q: &[f64],
    qd: &[f64],
    qdd: Option<&[f64]>,
    tau: Option<&[f64]>,
) -> Result<(), ArmAlgorithmError> {
    let dof = backend.dof_count();
    for (field, actual) in [
        ("dynamics.q", q.len()),
        ("dynamics.qd", qd.len()),
        ("dynamics.qdd", qdd.map_or(dof, <[f64]>::len)),
        ("dynamics.tau", tau.map_or(dof, <[f64]>::len)),
    ] {
        if actual != dof {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field,
                expected: dof,
                actual,
            });
        }
    }
    for (destination, source) in backend.q.iter_mut().zip(q) {
        *destination = f32_checked(*source, "dynamics.q")?;
    }
    for (destination, source) in backend.qd.iter_mut().zip(qd) {
        *destination = f32_checked(*source, "dynamics.qd")?;
    }
    backend.qdd.fill(0.0);
    if let Some(qdd) = qdd {
        for (destination, source) in backend.qdd.iter_mut().zip(qdd) {
            *destination = f32_checked(*source, "dynamics.qdd")?;
        }
    }
    backend.tau.fill(0.0);
    if let Some(tau) = tau {
        for (destination, source) in backend.tau.iter_mut().zip(tau) {
            *destination = f32_checked(*source, "dynamics.tau")?;
        }
    }
    backend.clear_external_forces();
    Ok(())
}

fn validate_mass_matrix(matrix: &ArmMassMatrix) -> Result<(), ArmAlgorithmError> {
    if matrix.data.len() != matrix.dof * matrix.dof
        || matrix.data.iter().any(|value| !value.is_finite())
    {
        return Err(ArmAlgorithmError::NonFinite {
            field: "dynamics.mass_matrix",
        });
    }
    let value = matrix.as_matrix();
    for row in 0..matrix.dof {
        for column in 0..matrix.dof {
            if (value[(row, column)] - value[(column, row)]).abs() > 1.0e-4 {
                return Err(ArmAlgorithmError::InvalidModel {
                    reason: "mass matrix is not symmetric",
                });
            }
        }
    }
    let eigenvalues = value.symmetric_eigen().eigenvalues;
    let rank = eigenvalues.iter().filter(|value| **value > 1.0e-8).count();
    if rank != matrix.dof {
        return Err(ArmAlgorithmError::Singular {
            rank,
            required_rank: matrix.dof,
        });
    }
    Ok(())
}

fn position_values(
    order: &ArmJointOrder,
    vector: &ArmJointVector<ArmPosition>,
) -> Result<Vec<f64>, ArmAlgorithmError> {
    validate_order(order, vector.order())?;
    tagged_coordinate_values(
        order,
        vector
            .values()
            .iter()
            .map(|value| (value.kind(), value.value_si())),
    )
}

fn velocity_values(
    order: &ArmJointOrder,
    vector: &ArmJointVector<ArmVelocity>,
) -> Result<Vec<f64>, ArmAlgorithmError> {
    validate_order(order, vector.order())?;
    tagged_coordinate_values(
        order,
        vector
            .values()
            .iter()
            .map(|value| (value.kind(), value.value_si())),
    )
}

fn acceleration_values(
    order: &ArmJointOrder,
    vector: &ArmJointVector<ArmAcceleration>,
) -> Result<Vec<f64>, ArmAlgorithmError> {
    validate_order(order, vector.order())?;
    tagged_coordinate_values(
        order,
        vector
            .values()
            .iter()
            .map(|value| (value.kind(), value.value_si())),
    )
}

fn effort_values(
    order: &ArmJointOrder,
    vector: &ArmJointVector<ArmEffort>,
) -> Result<Vec<f64>, ArmAlgorithmError> {
    validate_order(order, vector.order())?;
    let mut result = Vec::with_capacity(order.len());
    for (descriptor, value) in order.joints().iter().zip(vector.values()) {
        if descriptor.kind().effort_kind() != value.kind() {
            return Err(ArmAlgorithmError::UnitMismatch {
                joint: descriptor.joint_id(),
            });
        }
        result.push(value.value_si());
    }
    Ok(result)
}

fn tagged_coordinate_values(
    order: &ArmJointOrder,
    values: impl Iterator<Item = (message_model::ArmCoordinateKind, f64)>,
) -> Result<Vec<f64>, ArmAlgorithmError> {
    let mut result = Vec::with_capacity(order.len());
    for (descriptor, (kind, value)) in order.joints().iter().zip(values) {
        if descriptor.kind().coordinate_kind() != kind {
            return Err(ArmAlgorithmError::UnitMismatch {
                joint: descriptor.joint_id(),
            });
        }
        result.push(value);
    }
    Ok(result)
}

fn validate_order(
    expected: &ArmJointOrder,
    actual: &ArmJointOrder,
) -> Result<(), ArmAlgorithmError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ArmAlgorithmError::InvalidModel {
            reason: "dynamics vector joint order mismatch",
        })
    }
}

fn effort_vector(
    order: &ArmJointOrder,
    values: &[f32],
) -> Result<ArmJointVector<ArmEffort>, ArmAlgorithmError> {
    if values.len() != order.len() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "dynamics.effort",
            expected: order.len(),
            actual: values.len(),
        });
    }
    let mut result = ArrayVec::<ArmEffort, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        let value = output_f64(*value, "dynamics.effort")?;
        result.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(value),
                ArmJointKind::Prismatic => ArmEffort::force_n(value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "dynamics.effort",
            })?,
        );
    }
    ArmJointVector::new(order.clone(), result).map_err(|_| ArmAlgorithmError::InvalidModel {
        reason: "dynamics effort output dimension changed",
    })
}

fn acceleration_vector(
    order: &ArmJointOrder,
    values: &[f32],
) -> Result<ArmJointVector<ArmAcceleration>, ArmAlgorithmError> {
    let values = values
        .iter()
        .map(|value| output_f64(*value, "dynamics.acceleration"))
        .collect::<Result<Vec<_>, _>>()?;
    acceleration_vector_f64(order, &values)
}

fn position_vector(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmPosition>, ArmAlgorithmError> {
    tagged_position_vector(order, values)
}

fn tagged_position_vector(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmPosition>, ArmAlgorithmError> {
    if values.len() != order.len() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "dynamics.position",
            expected: order.len(),
            actual: values.len(),
        });
    }
    let mut result = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        result.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmPosition::angular_rad(*value)
                }
                ArmJointKind::Prismatic => ArmPosition::linear_m(*value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "dynamics.position",
            })?,
        );
    }
    ArmJointVector::new(order.clone(), result).map_err(|_| ArmAlgorithmError::InvalidModel {
        reason: "dynamics position output dimension changed",
    })
}

fn velocity_vector(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmVelocity>, ArmAlgorithmError> {
    if values.len() != order.len() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "dynamics.velocity",
            expected: order.len(),
            actual: values.len(),
        });
    }
    let mut result = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        result.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(*value)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(*value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "dynamics.velocity",
            })?,
        );
    }
    ArmJointVector::new(order.clone(), result).map_err(|_| ArmAlgorithmError::InvalidModel {
        reason: "dynamics velocity output dimension changed",
    })
}

fn acceleration_vector_f64(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmAcceleration>, ArmAlgorithmError> {
    if values.len() != order.len() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "dynamics.acceleration",
            expected: order.len(),
            actual: values.len(),
        });
    }
    let mut result = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        result.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmAcceleration::angular_radps2(*value)
                }
                ArmJointKind::Prismatic => ArmAcceleration::linear_mps2(*value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "dynamics.acceleration",
            })?,
        );
    }
    ArmJointVector::new(order.clone(), result).map_err(|_| ArmAlgorithmError::InvalidModel {
        reason: "dynamics acceleration output dimension changed",
    })
}

fn vector3_f32(values: [f64; 3], field: &'static str) -> Result<Vector3<f32>, ArmAlgorithmError> {
    Ok(Vector3::new(
        f32_checked(values[0], field)?,
        f32_checked(values[1], field)?,
        f32_checked(values[2], field)?,
    ))
}

fn f32_checked(value: f64, field: &'static str) -> Result<f32, ArmAlgorithmError> {
    if !value.is_finite() {
        return Err(ArmAlgorithmError::NonFinite { field });
    }
    let converted = value as f32;
    if converted.is_finite() {
        Ok(converted)
    } else {
        Err(ArmAlgorithmError::NonFinite { field })
    }
}

fn output_f64(value: f32, field: &'static str) -> Result<f64, ArmAlgorithmError> {
    if value.is_finite() {
        Ok(f64::from(value))
    } else {
        Err(ArmAlgorithmError::NonFinite { field })
    }
}
