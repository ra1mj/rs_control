#![allow(dead_code)]

use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use message_model::{ArmJointId, AsciiId};

const SYNTHETIC_6DOF: &str = include_str!("../../../arm-model/tests/fixtures/synthetic_6dof.urdf");
const MIXED_MULTI_GROUP: &str =
    include_str!("../../../arm-model/tests/fixtures/mixed_multi_group.urdf");

pub fn id(value: &str) -> ArmJointId {
    AsciiId::parse(value).unwrap()
}

pub fn synthetic_model() -> ArmModel {
    let joints = (1..=6)
        .map(|index| id(&format!("joint_{index}")))
        .collect::<Vec<_>>();
    let spec = ArmProfileSpec {
        schema_version: ARM_PROFILE_SCHEMA_VERSION,
        name: id("synthetic_6dof"),
        class: ArmProfileClass::HostFixture,
        groups: vec![KinematicGroupSpec {
            id: id("arm"),
            base_link: id("base_link"),
            tip_link: id("tool0"),
            active_joint_order: joints.clone(),
        }],
        acceleration_limits: joints
            .iter()
            .copied()
            .map(|joint_id| JointAccelerationLimitSpec {
                joint_id,
                maximum: 4.0,
            })
            .collect(),
        transmissions: joints
            .iter()
            .enumerate()
            .map(|(index, joint_id)| RotaryTransmissionSpec {
                joint_id: *joint_id,
                actuator_id: id(&format!("motor_{}", index + 1)),
                scale_rad_per_joint_unit: 100.0,
                offset_rad: 0.0,
            })
            .collect(),
    };
    ArmModel::compile_urdf(SYNTHETIC_6DOF, &spec).unwrap()
}

pub fn mixed_model() -> ArmModel {
    let active = ["left_yaw", "left_slide", "right_roll"];
    let spec = ArmProfileSpec {
        schema_version: ARM_PROFILE_SCHEMA_VERSION,
        name: id("mixed_multi_group"),
        class: ArmProfileClass::HostFixture,
        groups: vec![
            KinematicGroupSpec {
                id: id("left_arm"),
                base_link: id("base_link"),
                tip_link: id("left_tip"),
                active_joint_order: vec![id("left_yaw"), id("left_slide")],
            },
            KinematicGroupSpec {
                id: id("right_arm"),
                base_link: id("base_link"),
                tip_link: id("right_tip"),
                active_joint_order: vec![id("right_roll")],
            },
        ],
        acceleration_limits: active
            .into_iter()
            .map(|joint| JointAccelerationLimitSpec {
                joint_id: id(joint),
                maximum: 2.0,
            })
            .collect(),
        transmissions: Vec::new(),
    };
    ArmModel::compile_urdf(MIXED_MULTI_GROUP, &spec).unwrap()
}
