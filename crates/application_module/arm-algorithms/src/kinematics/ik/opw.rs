// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/opw.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: conservative prepared-axis admission, canonical base
// frame, fixed candidate iteration and mandatory project-FK validation.

//! KINETIC-derived OPW analytical six-axis IK.

use std::f64::consts::PI;

use message_model::MAX_JOINTS;
use nalgebra::{Matrix3, Vector3};

use super::{
    ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics,
    analytical::CandidateAccumulator,
};
use crate::ArmAlgorithmError;

struct Parameters {
    basis: Matrix3<f64>,
    shoulder_height: f64,
    upper_arm_length: f64,
    forearm_length: f64,
    forearm_phi: f64,
    wrist_to_tool_local: Vector3<f64>,
}

pub(super) fn solve(
    prepared: &PreparedKinematics<'_>,
    workspace: &mut IkWorkspace,
    target: ArmPose,
    seed: &[f64; MAX_JOINTS],
    config: IkSolveConfig,
) -> Result<KernelOutcome, ArmAlgorithmError> {
    let parameters = parameters(prepared)?;
    let target_pose = target.to_isometry();
    let target_wrist =
        target_pose.translation.vector - target_pose.rotation * parameters.wrist_to_tool_local;
    let wrist = parameters.basis.transpose() * target_wrist;
    let mut accumulator = CandidateAccumulator::new(*seed);

    let joint_1 = [
        (-wrist.y).atan2(-wrist.x),
        normalize((-wrist.y).atan2(-wrist.x) + PI),
    ];
    for joint_1 in joint_1 {
        let radial = wrist.x * joint_1.cos() + wrist.y * joint_1.sin();
        let height = wrist.z - parameters.shoulder_height;
        let distance_squared = radial * radial + height * height;
        let denominator = 2.0 * parameters.upper_arm_length * parameters.forearm_length;
        if denominator <= 1.0e-15 {
            continue;
        }
        let cosine = (distance_squared
            - parameters.upper_arm_length.powi(2)
            - parameters.forearm_length.powi(2))
            / denominator;
        if cosine.abs() > 1.0 {
            continue;
        }
        let elbow = cosine.acos();
        for sign in [1.0, -1.0] {
            let joint_3 = normalize(sign * elbow - parameters.forearm_phi);
            let arm_x = -parameters.upper_arm_length
                - parameters.forearm_length * (joint_3 + parameters.forearm_phi).cos();
            let arm_z = parameters.forearm_length * (joint_3 + parameters.forearm_phi).sin();
            let joint_2 = normalize(arm_z.atan2(arm_x) - height.atan2(radial));
            let mut partial = [0.0; MAX_JOINTS];
            partial[..3].copy_from_slice(&[joint_1, joint_2, joint_3]);
            if prepared.validate_values(&partial[..6]).is_err() {
                continue;
            }
            let partial_pose = prepared.evaluate(workspace, &partial[..6])?;
            let wrist_rotation =
                (partial_pose.rotation.inverse() * target_pose.rotation).to_rotation_matrix();
            let (wrist_solutions, count) = decompose_yzy(wrist_rotation.matrix());
            for &(joint_4, joint_5, joint_6) in &wrist_solutions[..count] {
                let mut candidate = partial;
                candidate[3] = normalize(joint_4);
                candidate[4] = normalize(joint_5);
                candidate[5] = normalize(joint_6);
                accumulator.consider(prepared, workspace, target, candidate)?;
            }
        }
    }
    accumulator.finish(config, 1)
}

fn parameters(prepared: &PreparedKinematics<'_>) -> Result<Parameters, ArmAlgorithmError> {
    let wrist = prepared
        .wrist_center
        .ok_or(ArmAlgorithmError::UnsupportedGeometry {
            solver: "opw",
            reason: "spherical wrist intersection is unavailable",
        })?;
    let z_axis = prepared.zero_axes[0];
    let y_axis = prepared.zero_axes[1];
    let x_axis = y_axis.cross(&z_axis).normalize();
    let basis = Matrix3::from_columns(&[x_axis, y_axis, z_axis]);
    let shoulder = basis.transpose() * prepared.zero_origins[0];
    let elbow = basis.transpose() * prepared.zero_origins[2];
    let wrist_local = basis.transpose() * wrist;
    let forearm = wrist_local - elbow;
    let wrist_to_tool_world = prepared.zero_tip.translation.vector - wrist;
    Ok(Parameters {
        basis,
        shoulder_height: shoulder.z,
        upper_arm_length: (elbow - shoulder).norm(),
        forearm_length: forearm.norm(),
        forearm_phi: forearm.z.atan2(-forearm.x),
        wrist_to_tool_local: prepared.zero_tip.rotation.inverse() * wrist_to_tool_world,
    })
}

fn decompose_yzy(rotation: &Matrix3<f64>) -> ([(f64, f64, f64); 2], usize) {
    let cosine = rotation[(1, 1)].clamp(-1.0, 1.0);
    let sine_squared = 1.0 - cosine * cosine;
    if sine_squared > 1.0e-10 {
        let sine = sine_squared.sqrt();
        let positive = (
            rotation[(2, 1)].atan2(-rotation[(0, 1)]),
            sine.atan2(cosine),
            rotation[(1, 2)].atan2(rotation[(1, 0)]),
        );
        let negative = (
            (-rotation[(2, 1)]).atan2(rotation[(0, 1)]),
            (-sine).atan2(cosine),
            (-rotation[(1, 2)]).atan2(-rotation[(1, 0)]),
        );
        ([positive, negative], 2)
    } else {
        let middle = if cosine > 0.0 { 0.0 } else { PI };
        let sum = rotation[(0, 0)].atan2(-rotation[(2, 0)]);
        ([(sum, middle, 0.0), (0.0, 0.0, 0.0)], 1)
    }
}

fn normalize(angle: f64) -> f64 {
    (angle + PI).rem_euclid(2.0 * PI) - PI
}
