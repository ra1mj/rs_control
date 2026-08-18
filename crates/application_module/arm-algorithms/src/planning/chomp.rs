//! KINETIC-informed prepared CHOMP optimizer.
//!
//! Source: `kinetic-planning/src/chomp.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `5b9e776a451fbfdae65e120f6fab258a803c349f486381b802d40496614ee07b`).
//! Adapted to precomputed metric inversion, caller-owned gradients, bounded
//! line search and allocation-stable prepared storage.

use message_model::{ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};

use crate::{ArmAlgorithmError, CollisionUnchecked};

use super::{
    ArmPlan, ArmStateValidator, OptimizationConfig, OptimizationWorkspace, PlannerConfig,
    PlannerEvidence, PlannerKind, PlannerStatistics, PreparedPlanning, SmoothnessOnly,
    TrajectoryCost, build_plan,
    core::{PlannerWorkspace, State, copy_position},
    optimization::{
        prepare_feasible_seed, require_cost_generation, trajectory_cost, trajectory_valid,
    },
};

/// Prepared CHOMP metric and immutable identity.
pub struct PreparedChomp {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    model_generation: u64,
    constraint_generation: u64,
    dof: usize,
    waypoint_count: usize,
    inverse_metric: Vec<f64>,
}

impl PreparedChomp {
    /// Precomputes the covariant smoothness metric for one exact path shape.
    pub fn new(
        prepared: &PreparedPlanning<'_, '_>,
        config: OptimizationConfig,
    ) -> Result<Self, ArmAlgorithmError> {
        let interior = config.waypoint_count - 2;
        let mut inverse_metric = vec![0.0; interior * interior];
        for row in 0..interior {
            for column in 0..interior {
                let minimum = row.min(column) + 1;
                let maximum = row.max(column) + 1;
                inverse_metric[row * interior + column] =
                    (minimum * (interior + 1 - maximum)) as f64 / (interior + 1) as f64;
            }
        }
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            dof: prepared.dof(),
            waypoint_count: config.waypoint_count,
            inverse_metric,
        })
    }

    /// Returns the storage reserved while preparing the covariant metric.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.inverse_metric.capacity() * std::mem::size_of::<f64>()
    }

    /// Optimizes one caller-supplied feasible seed trajectory.
    #[allow(clippy::too_many_arguments)]
    pub fn optimize(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        seed: &[ArmJointVector<ArmPosition>],
        planner_config: PlannerConfig,
        config: OptimizationConfig,
        cost: &impl TrajectoryCost,
        validator: &impl ArmStateValidator,
        workspace: &mut OptimizationWorkspace,
    ) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
        workspace.reset(prepared, config)?;
        self.require_context(prepared, config)?;
        require_cost_generation(prepared, cost)?;
        if seed.len() != self.waypoint_count {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "planner.chomp_seed",
                expected: self.waypoint_count,
                actual: seed.len(),
            });
        }
        for point in seed {
            workspace.trajectory.push(copy_position(prepared, point)?);
        }
        if !trajectory_valid(
            prepared,
            &workspace.trajectory,
            planner_config.edge_resolution,
            validator,
        ) {
            return Err(ArmAlgorithmError::ConstraintViolation {
                sample_index: 0,
                reason: "chomp_seed",
            });
        }
        let statistics = self.optimize_states(
            prepared,
            planner_config,
            config,
            cost,
            validator,
            workspace,
            0,
        )?;
        build_plan(
            prepared,
            PlannerKind::Chomp,
            config.seed,
            &workspace.trajectory,
            statistics,
        )
    }

    fn require_context(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        config: OptimizationConfig,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || self.dof != prepared.dof()
            || self.waypoint_count != config.waypoint_count
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "CHOMP identity or waypoint shape changed",
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

    #[allow(clippy::too_many_arguments)]
    fn optimize_states(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        planner_config: PlannerConfig,
        config: OptimizationConfig,
        cost: &impl TrajectoryCost,
        validator: &impl ArmStateValidator,
        workspace: &mut OptimizationWorkspace,
        samples: u32,
    ) -> Result<PlannerStatistics, ArmAlgorithmError> {
        let initial_cost = trajectory_cost(prepared, &workspace.trajectory, cost)?;
        let mut best_cost = initial_cost;
        let mut accepted_updates = 0_u32;
        let mut rejected_updates = 0_u32;
        let mut completed_iterations = 0_u32;
        let interior = self.waypoint_count - 2;
        let minimum_improvement = config.tolerance * initial_cost.abs().max(1.0);

        for iteration in 1..=config.maximum_iterations {
            completed_iterations = iteration;
            workspace.gradient.clear();
            workspace
                .gradient
                .resize(self.waypoint_count, State::zero());
            for window_start in 0..self.waypoint_count - 2 {
                for joint in 0..self.dof {
                    let acceleration = workspace.trajectory[window_start + 2].as_slice(self.dof)
                        [joint]
                        - 2.0 * workspace.trajectory[window_start + 1].as_slice(self.dof)[joint]
                        + workspace.trajectory[window_start].as_slice(self.dof)[joint];
                    workspace.gradient[window_start].as_mut_slice(self.dof)[joint] +=
                        2.0 * acceleration;
                    workspace.gradient[window_start + 1].as_mut_slice(self.dof)[joint] -=
                        4.0 * acceleration;
                    workspace.gradient[window_start + 2].as_mut_slice(self.dof)[joint] +=
                        2.0 * acceleration;
                }
            }
            for index in 1..self.waypoint_count - 1 {
                let mut objective_gradient = State::zero();
                cost.gradient(
                    workspace.trajectory[index].as_slice(self.dof),
                    objective_gradient.as_mut_slice(self.dof),
                )
                .map_err(|reason| ArmAlgorithmError::ConstraintViolation {
                    sample_index: index,
                    reason,
                })?;
                for joint in 0..self.dof {
                    workspace.gradient[index].as_mut_slice(self.dof)[joint] +=
                        objective_gradient.as_slice(self.dof)[joint];
                }
            }

            let mut accepted = false;
            for line_search in 0..6 {
                workspace.candidate.clear();
                workspace.candidate.extend_from_slice(&workspace.trajectory);
                let rate = config.learning_rate * 0.5_f64.powi(line_search);
                for row in 0..interior {
                    for joint in 0..self.dof {
                        let mut update = 0.0;
                        for column in 0..interior {
                            update += self.inverse_metric[row * interior + column]
                                * workspace.gradient[column + 1].as_slice(self.dof)[joint];
                        }
                        let [lower, upper] = prepared.kinematics.sampling_bound(joint);
                        workspace.candidate[row + 1].as_mut_slice(self.dof)[joint] =
                            (workspace.trajectory[row + 1].as_slice(self.dof)[joint]
                                - rate * update)
                                .clamp(lower, upper);
                    }
                }
                if !trajectory_valid(
                    prepared,
                    &workspace.candidate,
                    planner_config.edge_resolution,
                    validator,
                ) {
                    rejected_updates += 1;
                    continue;
                }
                let candidate_cost = trajectory_cost(prepared, &workspace.candidate, cost)?;
                if candidate_cost < best_cost - minimum_improvement {
                    std::mem::swap(&mut workspace.trajectory, &mut workspace.candidate);
                    best_cost = candidate_cost;
                    accepted_updates += 1;
                    accepted = true;
                    break;
                }
                rejected_updates += 1;
            }
            if !accepted || (initial_cost - best_cost) <= config.tolerance {
                break;
            }
        }
        if accepted_updates == 0 || best_cost >= initial_cost || !best_cost.is_finite() {
            return Err(ArmAlgorithmError::OptimizationNonConvergent {
                planner: PlannerKind::Chomp.name(),
                iterations: completed_iterations,
                initial_cost,
                final_cost: best_cost,
            });
        }
        Ok(PlannerStatistics::new(
            completed_iterations,
            samples,
            self.waypoint_count as u32,
            PlannerEvidence::Chomp {
                initial_cost,
                final_cost: best_cost,
                accepted_updates,
                rejected_updates,
            },
        ))
    }
}

pub(super) fn solve(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    planner_config: PlannerConfig,
    validator: &impl ArmStateValidator,
    planner_workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    let waypoint_count = 12_usize.min(planner_config.maximum_nodes as usize);
    if waypoint_count < 3 {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.chomp_waypoints",
            actual: waypoint_count,
            maximum: planner_config.maximum_nodes as usize,
        });
    }
    let config = OptimizationConfig::new(
        32,
        waypoint_count,
        0.05,
        1.0e-7,
        12,
        0.08,
        10.0,
        planner_config.seed,
    )?;
    let optimizer = PreparedChomp::new(prepared, config)?;
    let mut workspace = OptimizationWorkspace::new(prepared, config)?;
    let samples = prepare_feasible_seed(
        prepared,
        start,
        goal,
        planner_config,
        config,
        validator,
        &mut workspace,
    )?;
    let objective = SmoothnessOnly::new(prepared.constraint_generation())?;
    let statistics = optimizer.optimize_states(
        prepared,
        planner_config,
        config,
        &objective,
        validator,
        &mut workspace,
        samples,
    )?;
    planner_workspace.path.clear();
    planner_workspace
        .path
        .extend_from_slice(&workspace.trajectory);
    Ok(statistics)
}
