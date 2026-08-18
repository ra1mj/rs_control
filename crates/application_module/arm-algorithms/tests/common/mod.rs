#![allow(dead_code)]

pub mod performance;

use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use message_model::{ArmJointId, AsciiId};

const SYNTHETIC_6DOF: &str = include_str!("../../../arm-model/tests/fixtures/synthetic_6dof.urdf");
const MIXED_MULTI_GROUP: &str =
    include_str!("../../../arm-model/tests/fixtures/mixed_multi_group.urdf");
const OPW_SPHERICAL_6DOF: &str = include_str!("fixtures/opw_spherical_6dof.urdf");
const PADEN_SPHERICAL_6DOF: &str = include_str!("fixtures/paden_spherical_6dof.urdf");
const REDUNDANT_SPHERICAL_7DOF: &str = include_str!("fixtures/redundant_spherical_7dof.urdf");

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

pub fn opw_model() -> ArmModel {
    analytical_model(OPW_SPHERICAL_6DOF, "opw_spherical_6dof", 6)
}

pub fn paden_model() -> ArmModel {
    analytical_model(PADEN_SPHERICAL_6DOF, "paden_spherical_6dof", 6)
}

pub fn redundant_model() -> ArmModel {
    analytical_model(REDUNDANT_SPHERICAL_7DOF, "redundant_spherical_7dof", 7)
}

fn analytical_model(urdf: &str, name: &str, dof: usize) -> ArmModel {
    let joints = (1..=dof)
        .map(|index| id(&format!("joint_{index}")))
        .collect::<Vec<_>>();
    let spec = ArmProfileSpec {
        schema_version: ARM_PROFILE_SCHEMA_VERSION,
        name: id(name),
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
        transmissions: Vec::new(),
    };
    ArmModel::compile_urdf(urdf, &spec).unwrap()
}
