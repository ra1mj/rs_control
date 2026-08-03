// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/fabrik.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: project prepared-chain types, fixed reusable point
// buffers, exact joint limits, bounded numerical point-to-joint conversion and
// typed convergence evidence.

//! KINETIC-derived FABRIK kernel with its bounded DLS orientation refinement.

use message_model::MAX_JOINTS;

use super::{
    ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics, dls, pose_error,
};
use crate::ArmAlgorithmError;

pub(super) fn solve(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    seed: &[f64; MAX_JOINTS],
    config: IkSolveConfig,
) -> Result<KernelOutcome, ArmAlgorithmError> {
    let dof = prepared.dof();
    let point_count = prepared.path.len() + 1;
    let mut current = *seed;
    let mut evaluations = 0_u32;
    prepared.evaluate(workspace, &current[..dof])?;
    evaluations = evaluations.saturating_add(1);

    workspace.probe_points[..point_count].copy_from_slice(&workspace.link_points[..point_count]);
    for index in 0..point_count - 1 {
        workspace.link_lengths[index] =
            (workspace.probe_points[index + 1] - workspace.probe_points[index]).norm();
    }
    let base = workspace.probe_points[0];
    let target_position = target.to_isometry().translation.vector;
    let mut iterations = 0_u32;

    for iteration in 1..=config.max_iterations {
        iterations = iteration;
        if (workspace.probe_points[point_count - 1] - target_position).norm()
            < config.position_tolerance_m
        {
            break;
        }
        workspace.probe_points[point_count - 1] = target_position;
        for index in (0..point_count - 1).rev() {
            let direction = workspace.probe_points[index] - workspace.probe_points[index + 1];
            let distance = direction.norm();
            if distance > 1.0e-12 {
                workspace.probe_points[index] = workspace.probe_points[index + 1]
                    + direction * (workspace.link_lengths[index] / distance);
            }
        }
        workspace.probe_points[0] = base;
        for index in 0..point_count - 1 {
            let direction = workspace.probe_points[index + 1] - workspace.probe_points[index];
            let distance = direction.norm();
            if distance > 1.0e-12 {
                workspace.probe_points[index + 1] = workspace.probe_points[index]
                    + direction * (workspace.link_lengths[index] / distance);
            }
        }
    }

    // KINETIC uses five coordinate-descent refinements to map its point chain
    // back to joint coordinates. Each probe reuses the prepared FK buffers.
    for _ in 0..5 {
        for joint in 0..dof {
            prepared.evaluate(workspace, &current[..dof])?;
            evaluations = evaluations.saturating_add(1);
            let path_index = prepared.active_path_indices[joint];
            let point_index = path_index + 1;
            let current_position = workspace.link_points[point_index];
            let error = workspace.probe_points[point_index] - current_position;
            if error.norm() < 1.0e-8 {
                continue;
            }
            let mut probe = current;
            probe[joint] += 1.0e-5;
            prepared.project_limits(&mut probe[..dof]);
            let actual_step = probe[joint] - current[joint];
            if actual_step.abs() <= 1.0e-15 {
                continue;
            }
            prepared.evaluate(workspace, &probe[..dof])?;
            evaluations = evaluations.saturating_add(1);
            let gradient = (workspace.link_points[point_index] - current_position) / actual_step;
            let gradient_norm_squared = gradient.dot(&gradient);
            if gradient_norm_squared > 1.0e-12 {
                let step = (gradient.dot(&error) / gradient_norm_squared).clamp(-0.5, 0.5);
                current[joint] += step;
                prepared.project_limits(&mut current[..dof]);
            }
        }
    }

    let orientation_iterations = config.max_iterations.min(20);
    let mut degraded = false;
    for _ in 0..orientation_iterations {
        let pose = prepared.evaluate(workspace, &current[..dof])?;
        evaluations = evaluations.saturating_add(1);
        let (error, position_error_m, orientation_error_rad) = pose_error(&pose, target);
        if position_error_m < config.position_tolerance_m
            && orientation_error_rad < config.orientation_tolerance_rad
        {
            return Ok(KernelOutcome {
                coordinates: current,
                iterations,
                evaluations,
                generations: 0,
                candidates: 1,
                position_error_m,
                orientation_error_rad,
                degraded,
            });
        }
        let (delta, used_fallback) = dls::task_step(
            &workspace.jacobian,
            dof,
            error,
            0.1,
            config.max_step_norm * 0.5,
        );
        degraded |= used_fallback;
        for joint in 0..dof {
            current[joint] += delta[joint];
        }
        prepared.project_limits(&mut current[..dof]);
    }

    let pose = prepared.evaluate(workspace, &current[..dof])?;
    let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
    Err(ArmAlgorithmError::NonConvergent {
        iterations: iterations.saturating_add(orientation_iterations),
        position_error_m,
        orientation_error_rad,
    })
}
