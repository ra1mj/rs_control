//! KINETIC-informed prepared STOMP optimizer.
//!
//! Source: `kinetic-planning/src/stomp.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `64a7602a42dc3f170e1936ad06117cf76425f07acce49d635b8debc353ae7b3d`).
//! Adapted to explicit ChaCha8 noise, precomputed covariance, fixed rollout
//! storage and feasibility-preserving weighted updates.

use message_model::{ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};
use rand::RngExt;

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

/// Prepared smooth-noise covariance and immutable STOMP identity.
pub struct PreparedStomp {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    model_generation: u64,
    constraint_generation: u64,
    dof: usize,
    waypoint_count: usize,
    inverse_covariance: Vec<f64>,
}

impl PreparedStomp {
    /// Precomputes the smooth temporal covariance for one exact path shape.
    pub fn new(
        prepared: &PreparedPlanning<'_, '_>,
        config: OptimizationConfig,
    ) -> Result<Self, ArmAlgorithmError> {
        let interior = config.waypoint_count - 2;
        let mut inverse_covariance = vec![0.0; interior * interior];
        for row in 0..interior {
            for column in 0..interior {
                let minimum = row.min(column) + 1;
                let maximum = row.max(column) + 1;
                inverse_covariance[row * interior + column] =
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
            inverse_covariance,
        })
    }

    /// Optimizes one caller-supplied feasible seed with deterministic rollouts.
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
                field: "planner.stomp_seed",
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
                reason: "stomp_seed",
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
            PlannerKind::Stomp,
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
                reason: "STOMP identity or waypoint shape changed",
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
        seed_samples: u32,
    ) -> Result<PlannerStatistics, ArmAlgorithmError> {
        let initial_cost = trajectory_cost(prepared, &workspace.trajectory, cost)?;
        let mut best_cost = initial_cost;
        let mut accepted_updates = 0_u32;
        let mut evaluated_rollouts = 0_u32;
        let mut completed_iterations = 0_u32;
        let interior = self.waypoint_count - 2;
        let minimum_improvement = config.tolerance * initial_cost.abs().max(1.0);

        for iteration in 1..=config.maximum_iterations {
            completed_iterations = iteration;
            workspace.noises.clear();
            workspace.sample_costs.clear();
            let mut best_rollout = 0_usize;
            let mut best_rollout_cost = f64::INFINITY;

            for sample in 0..config.population_size {
                let base = workspace.noises.len();
                workspace
                    .noises
                    .resize(base + self.waypoint_count, State::zero());
                workspace.candidate.clear();
                workspace
                    .candidate
                    .resize(self.waypoint_count, State::zero());
                for joint in 0..self.dof {
                    for row in 0..interior {
                        workspace.candidate[row + 1].as_mut_slice(self.dof)[joint] =
                            workspace.rng.random_range(-1.0..=1.0) * config.noise_magnitude;
                    }
                    for row in 0..interior {
                        let mut smooth_noise = 0.0;
                        for column in 0..interior {
                            smooth_noise += self.inverse_covariance[row * interior + column]
                                * workspace.candidate[column + 1].as_slice(self.dof)[joint];
                        }
                        workspace.noises[base + row + 1].as_mut_slice(self.dof)[joint] =
                            smooth_noise / interior as f64;
                    }
                }
                build_rollout(
                    prepared,
                    &workspace.trajectory,
                    &workspace.noises[base..base + self.waypoint_count],
                    &mut workspace.candidate,
                );
                let rollout_cost = if trajectory_valid(
                    prepared,
                    &workspace.candidate,
                    planner_config.edge_resolution,
                    validator,
                ) {
                    trajectory_cost(prepared, &workspace.candidate, cost)?
                } else {
                    f64::INFINITY
                };
                workspace.sample_costs.push(rollout_cost);
                evaluated_rollouts += 1;
                if rollout_cost < best_rollout_cost {
                    best_rollout = sample;
                    best_rollout_cost = rollout_cost;
                }
            }

            compute_weights(
                &workspace.sample_costs,
                config.temperature,
                &mut workspace.weights,
            );
            workspace.candidate.clear();
            workspace.candidate.extend_from_slice(&workspace.trajectory);
            for waypoint in 1..self.waypoint_count - 1 {
                for joint in 0..self.dof {
                    let mut update = 0.0;
                    for sample in 0..config.population_size {
                        update += workspace.weights[sample]
                            * workspace.noises[sample * self.waypoint_count + waypoint]
                                .as_slice(self.dof)[joint];
                    }
                    let [lower, upper] = prepared.kinematics.sampling_bound(joint);
                    workspace.candidate[waypoint].as_mut_slice(self.dof)[joint] =
                        (workspace.trajectory[waypoint].as_slice(self.dof)[joint]
                            + config.learning_rate * update)
                            .clamp(lower, upper);
                }
            }
            let mut candidate_cost = if trajectory_valid(
                prepared,
                &workspace.candidate,
                planner_config.edge_resolution,
                validator,
            ) {
                trajectory_cost(prepared, &workspace.candidate, cost)?
            } else {
                f64::INFINITY
            };
            if best_rollout_cost < candidate_cost {
                let base = best_rollout * self.waypoint_count;
                build_rollout(
                    prepared,
                    &workspace.trajectory,
                    &workspace.noises[base..base + self.waypoint_count],
                    &mut workspace.candidate,
                );
                candidate_cost = best_rollout_cost;
            }
            if candidate_cost < best_cost - minimum_improvement
                && trajectory_valid(
                    prepared,
                    &workspace.candidate,
                    planner_config.edge_resolution,
                    validator,
                )
            {
                std::mem::swap(&mut workspace.trajectory, &mut workspace.candidate);
                best_cost = candidate_cost;
                accepted_updates += 1;
            }
        }
        if accepted_updates == 0 || best_cost >= initial_cost || !best_cost.is_finite() {
            return Err(ArmAlgorithmError::OptimizationNonConvergent {
                planner: PlannerKind::Stomp.name(),
                iterations: completed_iterations,
                initial_cost,
                final_cost: best_cost,
            });
        }
        Ok(PlannerStatistics::new(
            completed_iterations,
            seed_samples + evaluated_rollouts,
            self.waypoint_count as u32,
            PlannerEvidence::Stomp {
                initial_cost,
                final_cost: best_cost,
                evaluated_rollouts,
                accepted_updates,
            },
        ))
    }
}

fn build_rollout(
    prepared: &PreparedPlanning<'_, '_>,
    trajectory: &[State],
    noise: &[State],
    output: &mut Vec<State>,
) {
    output.clear();
    output.extend_from_slice(trajectory);
    for waypoint in 1..trajectory.len() - 1 {
        for joint in 0..prepared.dof() {
            let [lower, upper] = prepared.kinematics.sampling_bound(joint);
            output[waypoint].as_mut_slice(prepared.dof())[joint] = (trajectory[waypoint]
                .as_slice(prepared.dof())[joint]
                + noise[waypoint].as_slice(prepared.dof())[joint])
                .clamp(lower, upper);
        }
    }
}

fn compute_weights(costs: &[f64], temperature: f64, output: &mut Vec<f64>) {
    output.clear();
    let minimum = costs.iter().copied().fold(f64::INFINITY, f64::min);
    let maximum = costs
        .iter()
        .copied()
        .filter(|value| value.is_finite())
        .fold(minimum, f64::max);
    let range = (maximum - minimum).max(1.0e-12);
    let mut total = 0.0;
    for cost in costs {
        let weight = if cost.is_finite() {
            (-temperature * (*cost - minimum) / range).exp()
        } else {
            0.0
        };
        output.push(weight);
        total += weight;
    }
    if total <= f64::EPSILON {
        let uniform = 1.0 / costs.len() as f64;
        output.fill(uniform);
    } else {
        for weight in output {
            *weight /= total;
        }
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
            field: "planner.stomp_waypoints",
            actual: waypoint_count,
            maximum: planner_config.maximum_nodes as usize,
        });
    }
    let config = OptimizationConfig::new(
        24,
        waypoint_count,
        0.05,
        1.0e-7,
        12,
        0.08,
        10.0,
        planner_config.seed,
    )?;
    let optimizer = PreparedStomp::new(prepared, config)?;
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
