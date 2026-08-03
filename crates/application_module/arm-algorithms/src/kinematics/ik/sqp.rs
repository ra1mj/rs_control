// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/sqp.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: project analytical Jacobian, reusable augmented matrix,
// fixed coordinate arrays, exact projection and typed convergence failures.

//! KINETIC-derived projected SQP IK kernel.

use message_model::MAX_JOINTS;

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
    let mut evaluations = 0_u32;

    for iteration in 1..=config.max_iterations {
        let pose = prepared.evaluate(workspace, &current[..dof])?;
        evaluations = evaluations.saturating_add(1);
        let (error, position_error_m, orientation_error_rad) = pose_error(&pose, target);
        if position_error_m < config.position_tolerance_m
            && orientation_error_rad < config.orientation_tolerance_rad
        {
            return Ok(KernelOutcome {
                coordinates: current,
                iterations: iteration,
                evaluations,
                generations: 0,
                candidates: 1,
                position_error_m,
                orientation_error_rad,
                degraded: false,
            });
        }

        fill_normal_equations(
            &workspace.jacobian,
            dof,
            error,
            config.damping,
            &mut workspace.linear_system,
        );
        solve_augmented(&mut workspace.linear_system, dof, &mut workspace.delta)?;
        limit_step(&mut workspace.delta[..dof], config.max_step_norm);

        let current_objective = position_error_m + 0.1 * orientation_error_rad;
        let mut alpha = 1.0;
        let mut accepted = false;
        while alpha >= 0.1 {
            let mut trial = current;
            for (joint, value) in trial.iter_mut().enumerate().take(dof) {
                *value += alpha * workspace.delta[joint];
            }
            prepared.project_limits(&mut trial[..dof]);
            let trial_pose = prepared.evaluate(workspace, &trial[..dof])?;
            evaluations = evaluations.saturating_add(1);
            let (_, trial_position, trial_orientation) = pose_error(&trial_pose, target);
            if trial_position + 0.1 * trial_orientation < current_objective || alpha <= 0.1 {
                current = trial;
                accepted = true;
                break;
            }
            alpha *= 0.5;
        }
        if !accepted {
            for (joint, value) in current.iter_mut().enumerate().take(dof) {
                *value += 0.1 * workspace.delta[joint];
            }
            prepared.project_limits(&mut current[..dof]);
        }
    }

    let pose = prepared.evaluate(workspace, &current[..dof])?;
    let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
    Err(ArmAlgorithmError::NonConvergent {
        iterations: config.max_iterations,
        position_error_m,
        orientation_error_rad,
    })
}

fn fill_normal_equations(
    jacobian: &[f64],
    dof: usize,
    error: [f64; 6],
    damping: f64,
    augmented: &mut [f64],
) {
    augmented.fill(0.0);
    let width = dof + 1;
    for row in 0..dof {
        for column in 0..dof {
            let mut value = 0.0;
            for task in 0..6 {
                value += jacobian[task * dof + row] * jacobian[task * dof + column];
            }
            augmented[row * width + column] = value;
        }
        augmented[row * width + row] += damping;
        augmented[row * width + dof] = (0..6)
            .map(|task| jacobian[task * dof + row] * error[task])
            .sum();
    }
}

fn solve_augmented(
    augmented: &mut [f64],
    size: usize,
    solution: &mut [f64; MAX_JOINTS],
) -> Result<(), ArmAlgorithmError> {
    let width = size + 1;
    for column in 0..size {
        let mut pivot_row = column;
        let mut pivot_magnitude = augmented[column * width + column].abs();
        for row in column + 1..size {
            let magnitude = augmented[row * width + column].abs();
            if magnitude > pivot_magnitude {
                pivot_row = row;
                pivot_magnitude = magnitude;
            }
        }
        if pivot_magnitude < 1.0e-15 {
            return Err(ArmAlgorithmError::Singular {
                rank: column,
                required_rank: size,
            });
        }
        if pivot_row != column {
            for entry in column..=size {
                augmented.swap(column * width + entry, pivot_row * width + entry);
            }
        }
        let pivot = augmented[column * width + column];
        for row in column + 1..size {
            let factor = augmented[row * width + column] / pivot;
            for entry in column..=size {
                augmented[row * width + entry] -= factor * augmented[column * width + entry];
            }
        }
    }
    solution[..size].fill(0.0);
    for row in (0..size).rev() {
        let mut value = augmented[row * width + size];
        for column in row + 1..size {
            value -= augmented[row * width + column] * solution[column];
        }
        solution[row] = value / augmented[row * width + row];
    }
    Ok(())
}

fn limit_step(step: &mut [f64], maximum_norm: f64) {
    let norm = step.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm > maximum_norm {
        let scale = maximum_norm / norm;
        for value in step {
            *value *= scale;
        }
    }
}
