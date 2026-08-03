// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/paden_kahan.rs and subproblem.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: fixed candidate storage, corrected cosine-law angle
// convention, prepared geometry and mandatory project-FK validation.

//! KINETIC-derived Paden-Kahan spherical-wrist decomposition.

use std::f64::consts::PI;

use arrayvec::ArrayVec;
use message_model::MAX_JOINTS;
use nalgebra::{Matrix3, Vector3};

use super::{
    ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics,
    analytical::CandidateAccumulator,
};
use crate::ArmAlgorithmError;

pub(super) fn solve(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    seed: &[f64; MAX_JOINTS],
    config: IkSolveConfig,
) -> Result<KernelOutcome, ArmAlgorithmError> {
    let target_pose = target.to_isometry();
    let wrist = prepared
        .wrist_center
        .ok_or(ArmAlgorithmError::UnsupportedGeometry {
            solver: "paden_kahan",
            reason: "spherical wrist intersection is unavailable",
        })?;
    let tool_offset =
        prepared.zero_tip.rotation.inverse() * (prepared.zero_tip.translation.vector - wrist);
    let target_wrist = target_pose.translation.vector - target_pose.rotation * tool_offset;
    let positions = position_candidates(
        &prepared.zero_axes[..3],
        &prepared.zero_origins[..3],
        wrist,
        target_wrist,
    );
    let mut accumulator = CandidateAccumulator::new(*seed);
    for &(joint_1, joint_2, joint_3) in &positions {
        let mut partial = [0.0; MAX_JOINTS];
        partial[..3].copy_from_slice(&[joint_1, joint_2, joint_3]);
        if prepared.validate_values(&partial[..6]).is_err() {
            continue;
        }
        let partial_pose = prepared.evaluate(workspace, &partial[..6])?;
        let needed = (partial_pose.rotation.inverse() * target_pose.rotation).to_rotation_matrix();
        let (orientations, count) = decompose_zyz(needed.matrix());
        for &(joint_4, joint_5, joint_6) in &orientations[..count] {
            let mut candidate = partial;
            candidate[3] = normalize(joint_4);
            candidate[4] = normalize(joint_5);
            candidate[5] = normalize(joint_6);
            accumulator.consider(prepared, workspace, target, candidate)?;
        }
    }
    accumulator.finish(config, 1)
}

pub(super) fn position_candidates(
    axes: &[Vector3<f64>],
    origins: &[Vector3<f64>],
    wrist_zero: Vector3<f64>,
    target: Vector3<f64>,
) -> ArrayVec<(f64, f64, f64), 4> {
    let mut solutions = ArrayVec::new();
    let first_length = (origins[2] - origins[1]).norm();
    let wrist_offset = wrist_zero - origins[2];
    let second_length = wrist_offset.norm();
    if first_length < 1.0e-10 || second_length < 1.0e-10 {
        return solutions;
    }
    let target_relative = target - origins[0];
    let target_projection = target_relative - axes[0] * axes[0].dot(&target_relative);
    let zero_relative = wrist_zero - origins[0];
    let zero_projection = zero_relative - axes[0] * axes[0].dot(&zero_relative);
    let (Some(target_direction), Some(reference)) = (
        target_projection.try_normalize(1.0e-10),
        zero_projection.try_normalize(1.0e-10),
    ) else {
        return solutions;
    };
    let first = axes[0]
        .dot(&reference.cross(&target_direction))
        .atan2(reference.dot(&target_direction));
    for joint_1 in [normalize(first), normalize(first + PI)] {
        let rotation_1 = axis_angle_rotation(axes[0], joint_1);
        let shoulder = origins[0] + rotation_1 * (origins[1] - origins[0]);
        let distance = (target - shoulder).norm();
        let cosine =
            (distance * distance - first_length * first_length - second_length * second_length)
                / (2.0 * first_length * second_length);
        if cosine.abs() > 1.0 + 1.0e-6 {
            continue;
        }
        let elbow = cosine.clamp(-1.0, 1.0).acos();
        for joint_3 in [elbow, -elbow] {
            let rotation_3 = axis_angle_rotation(axes[2], joint_3);
            let wrist_at_joint_2_zero = shoulder
                + rotation_1 * (origins[2] - origins[1])
                + rotation_1 * rotation_3 * wrist_offset;
            let axis_2_world = rotation_1 * axes[1];
            if let Some(joint_2) =
                subproblem_1(axis_2_world, shoulder, wrist_at_joint_2_zero, target)
            {
                solutions.push((joint_1, normalize(joint_2), normalize(joint_3)));
            }
        }
    }
    solutions
}

pub(super) fn decompose_zyz(rotation: &Matrix3<f64>) -> ([(f64, f64, f64); 2], usize) {
    let beta = rotation[(2, 2)].clamp(-1.0, 1.0).acos();
    if beta.abs() < 1.0e-10 {
        let alpha = rotation[(1, 0)].atan2(rotation[(0, 0)]);
        return ([(alpha, 0.0, 0.0), (0.0, 0.0, 0.0)], 1);
    }
    if (beta - PI).abs() < 1.0e-10 {
        let alpha = (-rotation[(1, 0)]).atan2(-rotation[(0, 0)]);
        return ([(alpha, PI, 0.0), (0.0, 0.0, 0.0)], 1);
    }
    let sine = beta.sin();
    let positive = (
        (rotation[(1, 2)] / sine).atan2(rotation[(0, 2)] / sine),
        beta,
        (rotation[(2, 1)] / sine).atan2(-rotation[(2, 0)] / sine),
    );
    let negative = (
        (-rotation[(1, 2)] / sine).atan2(-rotation[(0, 2)] / sine),
        -beta,
        (-rotation[(2, 1)] / sine).atan2(rotation[(2, 0)] / sine),
    );
    ([positive, negative], 2)
}

fn subproblem_1(
    axis: Vector3<f64>,
    origin: Vector3<f64>,
    point: Vector3<f64>,
    target: Vector3<f64>,
) -> Option<f64> {
    let axis = axis.try_normalize(1.0e-15)?;
    let point_relative = point - origin;
    let target_relative = target - origin;
    let point_plane = point_relative - axis * axis.dot(&point_relative);
    let target_plane = target_relative - axis * axis.dot(&target_relative);
    if point_plane.norm() < 1.0e-10 || target_plane.norm() < 1.0e-10 {
        return None;
    }
    if (point_plane.norm() - target_plane.norm()).abs()
        > 1.0e-6 * point_plane.norm().max(target_plane.norm()).max(1.0)
        || (axis.dot(&point_relative) - axis.dot(&target_relative)).abs() > 1.0e-6
    {
        return None;
    }
    Some(
        axis.dot(&point_plane.cross(&target_plane))
            .atan2(point_plane.dot(&target_plane)),
    )
}

fn axis_angle_rotation(axis: Vector3<f64>, angle: f64) -> Matrix3<f64> {
    let axis = axis.normalize();
    let cosine = angle.cos();
    let sine = angle.sin();
    let one_minus_cosine = 1.0 - cosine;
    Matrix3::new(
        one_minus_cosine * axis.x * axis.x + cosine,
        one_minus_cosine * axis.x * axis.y - sine * axis.z,
        one_minus_cosine * axis.x * axis.z + sine * axis.y,
        one_minus_cosine * axis.x * axis.y + sine * axis.z,
        one_minus_cosine * axis.y * axis.y + cosine,
        one_minus_cosine * axis.y * axis.z - sine * axis.x,
        one_minus_cosine * axis.x * axis.z - sine * axis.y,
        one_minus_cosine * axis.y * axis.z + sine * axis.x,
        one_minus_cosine * axis.z * axis.z + cosine,
    )
}

fn normalize(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}
