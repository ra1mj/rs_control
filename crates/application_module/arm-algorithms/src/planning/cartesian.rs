//! KINETIC-informed bounded Cartesian waypoint planning.
//!
//! Source: `kinetic-planning/src/cartesian.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `3b7e6fc0acdf908558b4240afc5cac32797e6b7e1de49f11e893ae196ca94523`).
//! Pose interpolation is combined with the project's sole prepared KINETIC IK
//! boundary, continuity checks and exact semantic path validation.

use std::{marker::PhantomData, mem::size_of};

use message_model::{
    ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId,
    MAX_TRAJECTORY_POINTS,
};
use nalgebra::{Isometry3, Translation3};

use crate::{
    ArmAlgorithmError, CollisionUnchecked,
    kinematics::{ArmPose, IkSolveConfig, IkSolverKind, IkWorkspace},
};

use super::{
    ArmStateValidator, PreparedPlanning,
    core::{State, copy_position, edge_is_valid, validate_state},
};

/// Why a bounded Cartesian request stopped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CartesianTermination {
    /// Every requested pose step reached the target.
    Complete,
    /// Prepared IK rejected or did not converge at the next pose.
    IkFailure,
    /// The next IK solution exceeded the explicit joint jump bound.
    JointJump,
    /// The next state or exact joint-space edge failed semantic validation.
    Constraint,
}

/// Explicit Cartesian interpolation, continuity and IK settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CartesianConfig {
    maximum_translation_step_m: f64,
    maximum_rotation_step_rad: f64,
    maximum_joint_jump_si: f64,
    edge_resolution_si: f64,
    allow_partial: bool,
    solver: IkSolverKind,
    ik: IkSolveConfig,
}

impl CartesianConfig {
    /// Validates every pose, joint-continuity and interpolation bound.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        maximum_translation_step_m: f64,
        maximum_rotation_step_rad: f64,
        maximum_joint_jump_si: f64,
        edge_resolution_si: f64,
        allow_partial: bool,
        solver: IkSolverKind,
        ik: IkSolveConfig,
    ) -> Result<Self, ArmAlgorithmError> {
        if !maximum_translation_step_m.is_finite()
            || maximum_translation_step_m <= 0.0
            || !maximum_rotation_step_rad.is_finite()
            || maximum_rotation_step_rad <= 0.0
            || !maximum_joint_jump_si.is_finite()
            || maximum_joint_jump_si <= 0.0
            || !edge_resolution_si.is_finite()
            || edge_resolution_si <= 0.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.cartesian_config",
            });
        }
        Ok(Self {
            maximum_translation_step_m,
            maximum_rotation_step_rad,
            maximum_joint_jump_si,
            edge_resolution_si,
            allow_partial,
            solver,
            ik,
        })
    }

    /// Returns deterministic conservative host defaults.
    #[must_use]
    pub const fn host_default(seed: [u8; 32]) -> Self {
        Self {
            maximum_translation_step_m: 0.02,
            maximum_rotation_step_rad: 0.1,
            maximum_joint_jump_si: 0.5,
            edge_resolution_si: 0.03,
            allow_partial: false,
            solver: IkSolverKind::DampedLeastSquares,
            ik: IkSolveConfig::host_default(seed),
        }
    }

    /// Returns whether bounded failure produces diagnostic partial progress.
    #[must_use]
    pub const fn allow_partial(self) -> bool {
        self.allow_partial
    }

    /// Returns the prepared IK population capacity required by this policy.
    #[must_use]
    pub const fn maximum_ik_population(self) -> usize {
        self.ik.population_size()
    }
}

/// Diagnostic Cartesian output. Partial output is not an executable `ArmPlan`.
#[derive(Clone, Debug, PartialEq)]
pub struct CartesianPlan<Evidence> {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    target: ArmPose,
    waypoints: Vec<ArmJointVector<ArmPosition>>,
    requested_steps: usize,
    achieved_steps: usize,
    termination: CartesianTermination,
    seed: [u8; 32],
    _evidence: PhantomData<Evidence>,
}

impl<Evidence> CartesianPlan<Evidence> {
    /// Returns the immutable arm profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the selected kinematic group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the exact stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns the requested terminal pose.
    #[must_use]
    pub const fn target(&self) -> ArmPose {
        self.target
    }

    /// Returns accepted joint waypoints, including the start.
    #[must_use]
    pub fn waypoints(&self) -> &[ArmJointVector<ArmPosition>] {
        &self.waypoints
    }

    /// Returns the total pose interpolation count.
    #[must_use]
    pub const fn requested_steps(&self) -> usize {
        self.requested_steps
    }

    /// Returns the number of successfully solved pose steps.
    #[must_use]
    pub const fn achieved_steps(&self) -> usize {
        self.achieved_steps
    }

    /// Returns exact progress in `[0, 1]`.
    #[must_use]
    pub fn fraction(&self) -> f64 {
        self.achieved_steps as f64 / self.requested_steps.max(1) as f64
    }

    /// Returns the bounded completion or partial-progress reason.
    #[must_use]
    pub const fn termination(&self) -> CartesianTermination {
        self.termination
    }

    /// Returns whether all requested steps reached the target.
    #[must_use]
    pub const fn is_complete(&self) -> bool {
        matches!(self.termination, CartesianTermination::Complete)
    }

    /// Returns the IK request seed.
    #[must_use]
    pub const fn seed(&self) -> [u8; 32] {
        self.seed
    }
}

/// Prepared Cartesian planner with reusable IK and waypoint storage.
pub struct PreparedCartesianPlanner {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    model_generation: u64,
    constraint_generation: u64,
    maximum_waypoints: usize,
    ik_workspace: IkWorkspace,
    states: Vec<State>,
    positions: Vec<ArmJointVector<ArmPosition>>,
}

impl PreparedCartesianPlanner {
    /// Allocates reusable IK and Cartesian path storage once.
    pub fn new(
        prepared: &PreparedPlanning<'_, '_>,
        maximum_waypoints: usize,
        maximum_ik_population: usize,
    ) -> Result<Self, ArmAlgorithmError> {
        if !(2..=MAX_TRAJECTORY_POINTS).contains(&maximum_waypoints) {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.cartesian_waypoints",
                actual: maximum_waypoints,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            maximum_waypoints,
            ik_workspace: prepared.kinematics.workspace(maximum_ik_population)?,
            states: Vec::with_capacity(maximum_waypoints),
            positions: Vec::with_capacity(maximum_waypoints),
        })
    }

    /// Interpolates a pose request and solves each step with prepared IK.
    pub fn plan(
        &mut self,
        prepared: &PreparedPlanning<'_, '_>,
        start: &ArmJointVector<ArmPosition>,
        target: ArmPose,
        config: CartesianConfig,
        validator: &impl ArmStateValidator,
    ) -> Result<CartesianPlan<CollisionUnchecked>, ArmAlgorithmError> {
        self.require_context(prepared)?;
        self.states.clear();
        self.positions.clear();
        let start_state = copy_position(prepared, start)?;
        validate_state(prepared, &start_state, validator, 0)?;
        self.states.push(start_state);
        self.positions.push(start.clone());

        let start_pose = prepared.kinematics.chain().forward(start)?.tool_pose();
        let start_isometry = start_pose.to_isometry();
        let target_isometry = target.to_isometry();
        let translation_distance =
            (target_isometry.translation.vector - start_isometry.translation.vector).norm();
        let rotation_distance = start_isometry.rotation.angle_to(&target_isometry.rotation);
        let requested_steps = ((translation_distance / config.maximum_translation_step_m)
            .ceil()
            .max((rotation_distance / config.maximum_rotation_step_rad).ceil())
            .max(1.0)) as usize;
        let required_waypoints = requested_steps + 1;
        if required_waypoints > self.maximum_waypoints {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.cartesian_waypoints",
                actual: required_waypoints,
                maximum: self.maximum_waypoints,
            });
        }

        for step in 1..=requested_steps {
            let alpha = step as f64 / requested_steps as f64;
            let pose = interpolate_pose(&start_isometry, &target_isometry, alpha)?;
            let previous = self.positions.last().expect("start waypoint is retained");
            let solution = match prepared.kinematics.solve(
                config.solver,
                pose,
                previous,
                config.ik,
                &mut self.ik_workspace,
            ) {
                Ok(solution) => solution,
                Err(_error) if config.allow_partial => {
                    return Ok(self.finish(
                        target,
                        requested_steps,
                        step - 1,
                        CartesianTermination::IkFailure,
                        config.ik.seed(),
                    ));
                }
                Err(error) => return Err(error),
            };
            let state = copy_position(prepared, solution.positions())?;
            if maximum_jump(
                self.states.last().expect("start state is retained"),
                &state,
                prepared.dof(),
            ) > config.maximum_joint_jump_si
            {
                if config.allow_partial {
                    return Ok(self.finish(
                        target,
                        requested_steps,
                        step - 1,
                        CartesianTermination::JointJump,
                        config.ik.seed(),
                    ));
                }
                return Err(ArmAlgorithmError::ConstraintViolation {
                    sample_index: step,
                    reason: "cartesian_joint_jump",
                });
            }
            let exact_valid = validate_state(prepared, &state, validator, step).is_ok()
                && edge_is_valid(
                    prepared,
                    self.states.last().expect("start state is retained"),
                    &state,
                    config.edge_resolution_si,
                    validator,
                );
            if !exact_valid {
                if config.allow_partial {
                    return Ok(self.finish(
                        target,
                        requested_steps,
                        step - 1,
                        CartesianTermination::Constraint,
                        config.ik.seed(),
                    ));
                }
                return Err(ArmAlgorithmError::ConstraintViolation {
                    sample_index: step,
                    reason: "cartesian_path",
                });
            }
            self.states.push(state);
            self.positions.push(solution.positions().clone());
        }
        Ok(self.finish(
            target,
            requested_steps,
            requested_steps,
            CartesianTermination::Complete,
            config.ik.seed(),
        ))
    }

    /// Returns the storage reserved during Cartesian and IK preparation.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.ik_workspace.allocated_bytes()
            + self.states.capacity() * size_of::<State>()
            + self.positions.capacity() * size_of::<ArmJointVector<ArmPosition>>()
    }

    fn require_context(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || &self.order != prepared.order()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "Cartesian profile, group or order changed",
            });
        }
        if self.model_generation != prepared.model_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.model_generation,
                actual: prepared.model_generation(),
            });
        }
        if self.constraint_generation != prepared.constraint_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.constraint_generation,
                actual: prepared.constraint_generation(),
            });
        }
        Ok(())
    }

    fn finish(
        &self,
        target: ArmPose,
        requested_steps: usize,
        achieved_steps: usize,
        termination: CartesianTermination,
        seed: [u8; 32],
    ) -> CartesianPlan<CollisionUnchecked> {
        let _ = CollisionUnchecked::new();
        CartesianPlan {
            profile_id: self.profile_id,
            group_id: self.group_id,
            order: self.order.clone(),
            target,
            waypoints: self.positions.clone(),
            requested_steps,
            achieved_steps,
            termination,
            seed,
            _evidence: PhantomData,
        }
    }
}

fn interpolate_pose(
    start: &Isometry3<f64>,
    target: &Isometry3<f64>,
    alpha: f64,
) -> Result<ArmPose, ArmAlgorithmError> {
    let translation = start
        .translation
        .vector
        .lerp(&target.translation.vector, alpha);
    let rotation = start.rotation.slerp(&target.rotation, alpha);
    ArmPose::from_isometry(&Isometry3::from_parts(
        Translation3::from(translation),
        rotation,
    ))
}

fn maximum_jump(previous: &State, next: &State, dof: usize) -> f64 {
    previous
        .as_slice(dof)
        .iter()
        .zip(next.as_slice(dof))
        .map(|(previous, next)| (next - previous).abs())
        .fold(0.0, f64::max)
}
