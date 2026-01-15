mod common;

use arm_algorithms::{
    ArmAlgorithmError,
    kinematics::KinematicChain,
    reactive::{
        ArmTwist, ButterworthFilter, EmaFilter, JointVelocityFilter, NoFilter, RmpConfig,
        ServoConfig,
    },
};
use arrayvec::ArrayVec;
use message_model::{ArmJointKind, ArmJointVector, ArmVelocity, MAX_JOINTS};

use common::{id, synthetic_model};

fn zero_velocities(chain: KinematicChain<'_>) -> ArmJointVector<ArmVelocity> {
    let mut values = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for descriptor in chain.group().joint_order().joints() {
        values.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(0.0)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(0.0),
            }
            .unwrap(),
        );
    }
    ArmJointVector::new(chain.group().joint_order().clone(), values).unwrap()
}

#[test]
fn twist_pose_and_joint_jog_servo_are_bounded() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let positions = chain
        .positions_from_values(&[0.2, -0.3, 0.1, 0.1, -0.2, 0.15])
        .unwrap();
    let mut no_filter = NoFilter;
    let zero = chain
        .twist_servo(
            &positions,
            ArmTwist::new([0.0; 3], [0.0; 3]).unwrap(),
            ServoConfig::host_default(),
            &mut no_filter,
        )
        .unwrap();
    assert!(
        zero.values()
            .iter()
            .all(|value| value.value_si().abs() < 1.0e-12)
    );

    let current = chain.forward(&positions).unwrap().tool_pose();
    let tracked = chain
        .pose_tracking_servo(
            &positions,
            current,
            ServoConfig::host_default(),
            &mut no_filter,
        )
        .unwrap();
    assert!(
        tracked
            .values()
            .iter()
            .all(|value| value.value_si().abs() < 1.0e-10)
    );
    let jog = chain
        .joint_jog(&zero_velocities(chain), 0.5, &mut no_filter)
        .unwrap();
    assert_eq!(jog.values().len(), 6);
}

#[test]
fn filters_are_exact_dof_finite_and_deterministic() {
    let mut ema = EmaFilter::new(2, 0.5).unwrap();
    let mut first = [1.0, -1.0];
    ema.filter(&mut first).unwrap();
    let mut second = [0.0, 0.0];
    ema.filter(&mut second).unwrap();
    assert_eq!(second, [0.5, -0.5]);
    assert!(ema.filter(&mut [0.0]).is_err());

    let mut butterworth = ButterworthFilter::new(2, 100.0, 10.0).unwrap();
    let mut sample = [1.0, -1.0];
    butterworth.filter(&mut sample).unwrap();
    assert!(sample.iter().all(|value| value.is_finite()));
    assert!(ButterworthFilter::new(2, 100.0, 50.0).is_err());
}

#[test]
fn rmp_combines_reach_damping_limits_and_rejects_collision_policy() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let positions = chain.positions_from_values(&[0.0; 6]).unwrap();
    let target = chain
        .positions_from_values(&[0.1, -0.1, 0.1, 0.05, -0.05, 0.1])
        .unwrap();
    let acceleration = chain
        .rmp_joint_acceleration(
            &positions,
            &zero_velocities(chain),
            &target,
            RmpConfig::host_default(),
        )
        .unwrap();
    assert_eq!(acceleration.values().len(), 6);
    assert!(
        acceleration
            .values()
            .iter()
            .all(|value| value.value_si().is_finite())
    );
    assert!(matches!(
        chain.require_collision_rmp(),
        Err(ArmAlgorithmError::CollisionCapabilityUnavailable { .. })
    ));
}
