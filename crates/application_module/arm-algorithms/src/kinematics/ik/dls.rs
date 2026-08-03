// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/dls.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: project model/FK types, fixed-size task-space algebra,
// caller-seeded stall recovery, reusable workspace, exact work evidence and
// typed convergence failures.

//! KINETIC-derived damped least-squares IK kernel.

use message_model::MAX_JOINTS;
use nalgebra::{SMatrix, SVector};
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
    let mut current = *seed;
    let mut best = current;
    let mut best_error = f64::INFINITY;
    let mut previous_error = f64::INFINITY;
    let mut current_damping = config.damping;
    let mut stall_count = 0_usize;
    let mut evaluations = 0_u32;
    let mut degraded = false;
    workspace.rng = ChaCha8Rng::from_seed(config.seed);

    for iteration in 1..=config.max_iterations {
        let pose = prepared.evaluate(workspace, &current[..dof])?;
        evaluations = evaluations.saturating_add(1);
        let (error, position_error_m, orientation_error_rad) = pose_error(&pose, target);
        let combined_error = position_error_m + 0.1 * orientation_error_rad;

        if combined_error < best_error {
            best_error = combined_error;
            best = current;
            stall_count = 0;
        } else {
            stall_count = stall_count.saturating_add(1);
        }
        if position_error_m < config.position_tolerance_m
            && orientation_error_rad < config.orientation_tolerance_rad
        {
            return Ok(outcome(
                current,
                iteration,
                evaluations,
                position_error_m,
                orientation_error_rad,
                degraded,
            ));
        }

        // KINETIC restarts after 40 non-improving iterations. Resetting the
        // reusable RNG for each request makes that recovery reproducible.
        if stall_count >= 40 {
            current = best;
            for value in &mut current[..dof] {
                *value += workspace.rng.random_range(-0.3..0.3);
            }
            prepared.project_limits(&mut current[..dof]);
            stall_count = 0;
            current_damping = config.damping;
            previous_error = f64::INFINITY;
            continue;
        }

        if combined_error < previous_error * 0.99 {
            current_damping = (current_damping * 0.8).max(config.damping * 0.1);
        } else {
            current_damping = (current_damping * 1.5).min(config.damping * 20.0);
        }
        previous_error = combined_error;

        let (delta, used_fallback) = task_step(
            &workspace.jacobian,
            dof,
            error,
            current_damping,
            config.max_step_norm,
        );
        degraded |= used_fallback;
        for joint in 0..dof {
            current[joint] += delta[joint];
        }
        prepared.project_limits(&mut current[..dof]);
    }

    let final_pose = prepared.evaluate(workspace, &best[..dof])?;
    let (_, position_error_m, orientation_error_rad) = pose_error(&final_pose, target);
    Err(ArmAlgorithmError::NonConvergent {
        iterations: config.max_iterations,
        position_error_m,
        orientation_error_rad,
    })
}

pub(super) fn task_step(
    jacobian: &[f64],
    dof: usize,
    error: [f64; 6],
    damping: f64,
    maximum_norm: f64,
) -> ([f64; MAX_JOINTS], bool) {
    let mut system = SMatrix::<f64, 6, 6>::zeros();
    for row in 0..6 {
        for column in 0..6 {
            let mut value = 0.0;
            for joint in 0..dof {
                value += jacobian[row * dof + joint] * jacobian[column * dof + joint];
            }
            system[(row, column)] = value;
        }
        system[(row, row)] += damping * damping;
    }
    let task_error = SVector::<f64, 6>::from_row_slice(&error);
    let solved = system.lu().solve(&task_error);
    let mut delta = [0.0; MAX_JOINTS];
    let degraded = solved.is_none();
    for joint in 0..dof {
        delta[joint] = if let Some(ref task_step) = solved {
            (0..6)
                .map(|row| jacobian[row * dof + joint] * task_step[row])
                .sum()
        } else {
            0.1 * (0..6)
                .map(|row| jacobian[row * dof + joint] * error[row])
                .sum::<f64>()
        };
    }
    let norm = delta[..dof]
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    if norm > maximum_norm {
        let scale = maximum_norm / norm;
        for value in &mut delta[..dof] {
            *value *= scale;
        }
    }
    (delta, degraded)
}

fn outcome(
    coordinates: [f64; MAX_JOINTS],
    iterations: u32,
    evaluations: u32,
    position_error_m: f64,
    orientation_error_rad: f64,
    degraded: bool,
) -> KernelOutcome {
    KernelOutcome {
        coordinates,
        iterations,
        evaluations,
        generations: 0,
        candidates: 1,
        position_error_m,
        orientation_error_rad,
        degraded,
    }
}
