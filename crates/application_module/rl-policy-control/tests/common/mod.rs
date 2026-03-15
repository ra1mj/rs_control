use std::fs;
use std::path::{Path, PathBuf};

use arm_control::{ActuatorControlLimits, ArmControlConfig};
use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorId, ArmEffort, ArmJointId, ArmJointObservation, ArmJointSample, ArmPosition,
    ArmVelocity, AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128,
    KinematicGroupId,
};

pub const FIXTURE_URDF: &str = r#"
<robot name="rl_fixture">
  <link name="base"/>
  <link name="link_1"/>
  <link name="tool"/>
  <joint name="joint_1" type="revolute">
    <parent link="base"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="20" velocity="4"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="tool"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="20" velocity="4"/>
  </joint>
</robot>
"#;

pub fn id(value: &str) -> ArmJointId {
    AsciiId::parse(value).unwrap()
}

pub fn fixture_model() -> ArmModel {
    let joint_1 = id("joint_1");
    let joint_2 = id("joint_2");
    ArmModel::compile_urdf(
        FIXTURE_URDF,
        &ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("rl_fixture"),
            class: ArmProfileClass::HostFixture,
            groups: vec![KinematicGroupSpec {
                id: KinematicGroupId::parse("arm").unwrap(),
                base_link: AsciiId::parse("base").unwrap(),
                tip_link: AsciiId::parse("tool").unwrap(),
                active_joint_order: vec![joint_1, joint_2],
            }],
            acceleration_limits: vec![
                JointAccelerationLimitSpec {
                    joint_id: joint_1,
                    maximum: 8.0,
                },
                JointAccelerationLimitSpec {
                    joint_id: joint_2,
                    maximum: 8.0,
                },
            ],
            transmissions: vec![
                RotaryTransmissionSpec {
                    joint_id: joint_1,
                    actuator_id: ArmActuatorId::parse("act_1").unwrap(),
                    scale_rad_per_joint_unit: 1.0,
                    offset_rad: 0.0,
                },
                RotaryTransmissionSpec {
                    joint_id: joint_2,
                    actuator_id: ArmActuatorId::parse("act_2").unwrap(),
                    scale_rad_per_joint_unit: 1.0,
                    offset_rad: 0.0,
                },
            ],
        },
    )
    .unwrap()
}

pub fn fixture_control(model: &ArmModel) -> ArmControlConfig {
    ArmControlConfig::new(
        model,
        KinematicGroupId::parse("arm").unwrap(),
        11,
        AsciiId::parse("arm_state").unwrap(),
        Id128::new([7; 16]).unwrap(),
        model
            .transmissions()
            .iter()
            .map(|transmission| {
                ActuatorControlLimits::new(
                    transmission.binding(),
                    2.0,
                    4.0,
                    10.0,
                    20.0,
                    0.5,
                    10.0,
                    20.0,
                    50.0,
                )
                .unwrap()
            })
            .collect(),
    )
    .unwrap()
}

pub fn observation(
    model: &ArmModel,
    sequence: u64,
    source_time_ns: u64,
    valid_for_ns: u64,
) -> ArmJointObservation {
    let values = [(0.1, 0.2, 0.3), (-0.1, -0.2, -0.3)];
    let group = model
        .group(KinematicGroupId::parse("arm").unwrap())
        .unwrap();
    let mut samples = ArrayVec::new();
    for (descriptor, (position, velocity, effort)) in
        group.joint_order().joints().iter().zip(values)
    {
        samples
            .try_push(
                ArmJointSample::new(
                    *descriptor,
                    ArmPosition::angular_rad(position).unwrap(),
                    ArmVelocity::angular_radps(velocity).unwrap(),
                    ArmEffort::torque_nm(effort).unwrap(),
                )
                .unwrap(),
            )
            .unwrap();
    }
    ArmJointObservation::new(
        Header::new(HeaderFields {
            source_id: AsciiId::parse("arm_state").unwrap(),
            boot_id: Id128::new([7; 16]).unwrap(),
            sequence,
            source_monotonic_ns: source_time_ns,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap(),
        model.profile_id(),
        KinematicGroupId::parse("arm").unwrap(),
        samples,
    )
    .unwrap()
}

pub fn checked_fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("mit_identity")
}

pub fn copy_fixture(destination: &Path) {
    fs::copy(
        checked_fixture().join("manifest.toml"),
        destination.join("manifest.toml"),
    )
    .unwrap();
    fs::copy(
        checked_fixture().join("model.onnx"),
        destination.join("model.onnx"),
    )
    .unwrap();
}
