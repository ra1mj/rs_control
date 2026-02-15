use contract_codec::{
    DamiaoMitMappingContext, arm_mit_to_damiao, decode_arm_group_identity,
    decode_arm_trajectory_feedback, decode_arm_trajectory_goal, decode_arm_trajectory_result,
    encode_arm_group_identity, encode_arm_trajectory_feedback, encode_arm_trajectory_goal,
    encode_arm_trajectory_result,
};
use message_model::{
    ArmAcceleration, ArmActuatorBinding, ArmActuatorOrder, ArmGroupIdentity, ArmJointDescriptor,
    ArmJointKind, ArmJointOrder, ArmPosition, ArmProfileId, ArmTrajectoryPoint, ArmVelocity,
    AsciiId, ExecuteArmTrajectoryFeedback, ExecuteArmTrajectoryGoal, ExecuteArmTrajectoryResult,
    RotaryMitSetpoint,
};
use motor_damiao::RuntimeLimits;
use prost::Message;
use proto_generated::robot::v1 as wire;

fn id(value: &str) -> message_model::ArmJointId {
    AsciiId::parse(value).unwrap()
}

fn profile(value: u8) -> ArmProfileId {
    ArmProfileId::new([value; 32]).unwrap()
}

fn goal() -> ExecuteArmTrajectoryGoal {
    let order = ArmJointOrder::new(
        [
            ArmJointDescriptor::new(id("yaw"), ArmJointKind::Revolute),
            ArmJointDescriptor::new(id("slide"), ArmJointKind::Prismatic),
        ]
        .into_iter()
        .collect(),
    )
    .unwrap();
    let points = [(1_u64, [0.1, 0.2]), (2_u64, [0.2, 0.3])]
        .into_iter()
        .map(|(time, values)| {
            ArmTrajectoryPoint::new(
                time,
                &order,
                [
                    ArmPosition::angular_rad(values[0]).unwrap(),
                    ArmPosition::linear_m(values[1]).unwrap(),
                ]
                .into_iter()
                .collect(),
                [
                    ArmVelocity::angular_radps(0.0).unwrap(),
                    ArmVelocity::linear_mps(0.0).unwrap(),
                ]
                .into_iter()
                .collect(),
                [
                    ArmAcceleration::angular_radps2(0.0).unwrap(),
                    ArmAcceleration::linear_mps2(0.0).unwrap(),
                ]
                .into_iter()
                .collect(),
            )
            .unwrap()
        })
        .collect();
    ExecuteArmTrajectoryGoal::new(profile(1), id("arm"), order, points).unwrap()
}

#[test]
fn arm_trajectory_round_trips_all_units_and_payloads() {
    let goal = goal();
    assert_eq!(
        decode_arm_trajectory_goal(&encode_arm_trajectory_goal(&goal).unwrap()).unwrap(),
        goal
    );
    let feedback = ExecuteArmTrajectoryFeedback::new(1, 0.5).unwrap();
    assert_eq!(
        decode_arm_trajectory_feedback(&encode_arm_trajectory_feedback(feedback).unwrap()).unwrap(),
        feedback
    );
    let result = ExecuteArmTrajectoryResult::new(2);
    assert_eq!(
        decode_arm_trajectory_result(&encode_arm_trajectory_result(result)).unwrap(),
        result
    );
}

#[test]
fn arm_group_identity_round_trips_stable_joint_and_actuator_orders() {
    let joint_order = goal().order().clone();
    let actuator_order = ArmActuatorOrder::new(
        [
            ArmActuatorBinding::new(id("yaw"), id("motor_1")),
            ArmActuatorBinding::new(id("slide"), id("motor_2")),
        ]
        .into_iter()
        .collect(),
    )
    .unwrap();
    let identity =
        ArmGroupIdentity::new(profile(1), id("arm"), joint_order, Some(actuator_order)).unwrap();
    assert_eq!(
        decode_arm_group_identity(&encode_arm_group_identity(&identity)).unwrap(),
        identity
    );

    let mut malformed =
        wire::ArmGroupIdentity::decode(encode_arm_group_identity(&identity).as_slice()).unwrap();
    malformed.actuator_order.swap(0, 1);
    assert!(decode_arm_group_identity(&malformed.encode_to_vec()).is_err());
}

#[test]
fn arm_trajectory_rejects_identity_enum_dimension_and_nonfinite_input() {
    let mut wire = wire::ExecuteArmTrajectoryGoal {
        profile_id: vec![1; 31],
        group_id: "arm".to_owned(),
        joint_order: vec![wire::ArmJointDescriptor {
            joint_id: "yaw".to_owned(),
            kind: wire::ArmJointKind::Revolute as i32,
        }],
        points: vec![wire::ArmTrajectoryPoint {
            time_from_start_ns: 1,
            positions_si: vec![0.0],
            velocities_si: vec![0.0],
            accelerations_si: vec![0.0],
        }],
    };
    assert!(decode_arm_trajectory_goal(&wire.encode_to_vec()).is_err());
    wire.profile_id = vec![1; 32];
    wire.joint_order[0].kind = 999;
    assert!(decode_arm_trajectory_goal(&wire.encode_to_vec()).is_err());
    wire.joint_order[0].kind = wire::ArmJointKind::Revolute as i32;
    wire.points[0].positions_si.clear();
    assert!(decode_arm_trajectory_goal(&wire.encode_to_vec()).is_err());
    wire.points[0].positions_si.push(f64::NAN);
    assert!(decode_arm_trajectory_goal(&wire.encode_to_vec()).is_err());
    assert!(
        decode_arm_trajectory_feedback(
            &wire::ExecuteArmTrajectoryFeedback {
                active_point: 0,
                progress: f64::NAN,
            }
            .encode_to_vec(),
        )
        .is_err()
    );
}

#[test]
fn semantic_mit_mapping_checks_context_f32_and_runtime_limits() {
    let binding = ArmActuatorBinding::new(id("yaw"), id("motor_1"));
    let limits = RuntimeLimits::new(12.5, 30.0, 10.0).unwrap();
    let context = DamiaoMitMappingContext::new(profile(1), id("arm"), binding, limits);
    let command = arm_mit_to_damiao(
        profile(1),
        id("arm"),
        binding,
        RotaryMitSetpoint::new(0.1, 0.2, 20.0, 1.0, 0.5).unwrap(),
        context,
    )
    .unwrap();
    assert_eq!(command.position, 0.1_f32);
    assert_eq!(command.kp, 20.0_f32);

    assert!(
        arm_mit_to_damiao(
            profile(2),
            id("arm"),
            binding,
            RotaryMitSetpoint::new(0.1, 0.2, 20.0, 1.0, 0.5).unwrap(),
            context,
        )
        .is_err()
    );
    assert!(
        arm_mit_to_damiao(
            profile(1),
            id("arm"),
            binding,
            RotaryMitSetpoint::new(13.0, 0.2, 20.0, 1.0, 0.5).unwrap(),
            context,
        )
        .is_err()
    );
    assert!(
        arm_mit_to_damiao(
            profile(1),
            id("arm"),
            binding,
            RotaryMitSetpoint::new(f64::MAX, 0.2, 20.0, 1.0, 0.5).unwrap(),
            context,
        )
        .is_err()
    );
}
