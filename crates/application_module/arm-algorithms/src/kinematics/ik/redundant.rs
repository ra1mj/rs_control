// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/subproblem.rs (SubproblemIK7DOF) at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: fixed streaming candidates, caller-seed-stable sweep
// order, project prepared geometry and mandatory full seven-axis FK checks.

//! KINETIC-derived redundant seven-axis analytical sweep.

use message_model::MAX_JOINTS;

use super::{
    ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics,
    analytical::CandidateAccumulator,
    paden_kahan::{decompose_zyz, position_candidates},
};
use crate::ArmAlgorithmError;

pub(super) fn solve(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    seed: &[f64; MAX_JOINTS],
    config: IkSolveConfig,
) -> Result<KernelOutcome, ArmAlgorithmError> {
    let wrist = prepared
        .wrist_center
        .ok_or(ArmAlgorithmError::UnsupportedGeometry {
            solver: "redundant_7dof",
            reason: "spherical wrist intersection is unavailable",
        })?;
    let target_pose = target.to_isometry();
    let tool_offset =
        prepared.zero_tip.rotation.inverse() * (prepared.zero_tip.translation.vector - wrist);
    let target_wrist = target_pose.translation.vector - target_pose.rotation * tool_offset;
    let position_solutions = position_candidates(
        &prepared.zero_axes[..3],
        &prepared.zero_origins[..3],
        wrist,
        target_wrist,
    );
    let [lower, upper] = prepared.bounds[3].sampling;
    let sample_count = config.redundant_samples;
    let start =
        usize::from_le_bytes(config.seed[..size_of::<usize>()].try_into().map_err(|_| {
            ArmAlgorithmError::InvalidModel {
                reason: "IK seed width is smaller than usize",
            }
        })?) % sample_count;
    let mut accumulator = CandidateAccumulator::new(*seed);

    for offset in 0..sample_count {
        let sample = (start + offset) % sample_count;
        let fraction = if sample_count > 1 {
            sample as f64 / (sample_count - 1) as f64
        } else {
            0.5
        };
        let redundant = lower + fraction * (upper - lower);
        for &(joint_1, joint_2, joint_3) in &position_solutions {
            let mut partial = [0.0; MAX_JOINTS];
            partial[..4].copy_from_slice(&[joint_1, joint_2, joint_3, redundant]);
            if prepared.validate_values(&partial[..7]).is_err() {
                continue;
            }
            let partial_pose = prepared.evaluate(workspace, &partial[..7])?;
            let needed =
                (partial_pose.rotation.inverse() * target_pose.rotation).to_rotation_matrix();
            let (orientations, count) = decompose_zyz(needed.matrix());
            for &(joint_5, joint_6, joint_7) in &orientations[..count] {
                let mut candidate = partial;
                candidate[4] = joint_5;
                candidate[5] = joint_6;
                candidate[6] = joint_7;
                accumulator.consider(prepared, workspace, target, candidate)?;
            }
        }
    }
    accumulator.finish(config, sample_count as u32)
}
