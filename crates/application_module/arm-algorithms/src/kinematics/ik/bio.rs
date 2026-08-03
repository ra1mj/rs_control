// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/bio_ik.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: caller-recorded ChaCha8 seed, fixed maximum population,
// reusable members, stable total ordering, project FK/limits and typed failure.

//! KINETIC-derived deterministic fixed-population Bio-IK kernel.

use message_model::MAX_JOINTS;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

use super::{ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics, pose_error};
use crate::ArmAlgorithmError;

pub(super) fn solve(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    seed: &[f64; MAX_JOINTS],
    config: IkSolveConfig,
) -> Result<KernelOutcome, ArmAlgorithmError> {
    let dof = prepared.dof();
    let population_size = config.population_size;
    workspace.rng = ChaCha8Rng::from_seed(config.seed);
    workspace.population[0].coordinates = *seed;
    workspace.population[0].fitness = f64::INFINITY;
    for member in 1..population_size {
        for joint in 0..dof {
            let [lower, upper] = prepared.bounds[joint].sampling;
            workspace.population[member].coordinates[joint] =
                workspace.rng.random_range(lower..=upper);
        }
        workspace.population[member].fitness = f64::INFINITY;
    }

    let mut best = *seed;
    let mut best_fitness = f64::INFINITY;
    let mut sigma = 0.3_f64;
    let mut evaluations = 0_u32;

    for generation in 1..=config.max_iterations {
        for member in 0..population_size {
            let coordinates = workspace.population[member].coordinates;
            let fitness = evaluate_fitness(prepared, workspace, target, &coordinates[..dof])?;
            evaluations = evaluations.saturating_add(1);
            workspace.population[member].fitness = fitness;
        }
        workspace.population[..population_size]
            .sort_by(|left, right| left.fitness.total_cmp(&right.fitness));
        if workspace.population[0].fitness < best_fitness {
            best_fitness = workspace.population[0].fitness;
            best = workspace.population[0].coordinates;
        }

        let pose = prepared.evaluate(workspace, &best[..dof])?;
        evaluations = evaluations.saturating_add(1);
        let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
        if position_error_m < config.position_tolerance_m
            && orientation_error_rad < config.orientation_tolerance_rad
        {
            return Ok(KernelOutcome {
                coordinates: best,
                iterations: generation,
                evaluations,
                generations: generation,
                candidates: population_size as u32,
                position_error_m,
                orientation_error_rad,
                degraded: false,
            });
        }

        for elite in 0..config.elite_count {
            workspace.next_population[elite] = workspace.population[elite].clone();
        }
        for member in config.elite_count..population_size {
            let parent = workspace.rng.random_range(0..=population_size / 2);
            let mut child = workspace.population[parent].coordinates;
            if workspace.rng.random_bool(0.5) && population_size > 1 {
                let other = workspace.rng.random_range(0..=population_size / 2);
                let alpha = workspace.rng.random_range(0.3..0.7);
                for (joint, value) in child.iter_mut().enumerate().take(dof) {
                    *value = alpha * *value
                        + (1.0 - alpha) * workspace.population[other].coordinates[joint];
                }
            }
            for (joint, value) in child.iter_mut().enumerate().take(dof) {
                *value += workspace.rng.random_range(-sigma..sigma);
                let [lower, upper] = prepared.bounds[joint].sampling;
                *value = value.clamp(lower, upper);
            }
            workspace.next_population[member].coordinates = child;
            workspace.next_population[member].fitness = f64::INFINITY;
        }
        std::mem::swap(&mut workspace.population, &mut workspace.next_population);
        sigma = (sigma * 0.995).max(0.01);
    }

    let pose = prepared.evaluate(workspace, &best[..dof])?;
    let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
    Err(ArmAlgorithmError::NonConvergent {
        iterations: config.max_iterations,
        position_error_m,
        orientation_error_rad,
    })
}

fn evaluate_fitness(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    coordinates: &[f64],
) -> Result<f64, ArmAlgorithmError> {
    let pose = prepared.evaluate(workspace, coordinates)?;
    let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
    let mut centering = 0.0;
    for (joint, value) in coordinates.iter().copied().enumerate() {
        let [lower, upper] = prepared.bounds[joint].sampling;
        let center = (lower + upper) * 0.5;
        let range = (upper - lower).max(1.0e-6);
        centering += ((value - center) / range).powi(2);
    }
    centering /= coordinates.len() as f64;
    Ok(position_error_m + 0.1 * orientation_error_rad + 0.01 * centering)
}
