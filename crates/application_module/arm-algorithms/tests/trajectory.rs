mod common;

use arm_algorithms::{
    ArmAlgorithmError, TrajectoryViolationKind,
    kinematics::KinematicChain,
    trajectory::{
        ArmTrajectory, TrajectoryDeviationKind, TrajectoryDeviationLimits, TrajectorySampleBuffer,
        TrajectorySampling,
    },
};
use arrayvec::ArrayVec;
use message_model::{ArmJointVector, ArmPosition, MAX_JOINTS};

use common::{id, synthetic_model};

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

#[test]
fn trapezoidal_generation_is_bounded_and_converts_to_action() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let goal = chain
        .positions_from_values(&[0.4, -0.3, 0.2, 0.1, -0.2, 0.25])
        .unwrap();
    let trajectory =
        ArmTrajectory::trapezoidal(chain, &start, &goal, TrajectorySampling::new(33).unwrap())
            .unwrap();
    assert_eq!(trajectory.points().len(), 33);
    assert_eq!(trajectory.points()[0].time_from_start_ns(), 1);
    assert!(
        trajectory
            .points()
            .windows(2)
            .all(|pair| pair[0].time_from_start_ns() < pair[1].time_from_start_ns())
    );
    assert_eq!(trajectory.points()[0].positions_si(), &[0.0; 6]);
    for (actual, expected) in trajectory
        .points()
        .last()
        .unwrap()
        .positions_si()
        .iter()
        .zip([0.4, -0.3, 0.2, 0.1, -0.2, 0.25])
    {
        assert_near(*actual, expected, 1.0e-12);
    }
    let action = trajectory.to_action_goal().unwrap();
    assert_eq!(action.points().len(), 33);

    let mut output = TrajectorySampleBuffer::new();
    trajectory
        .sample_into(trajectory.duration_ns() / 2, &mut output)
        .unwrap();
    assert_eq!(output.positions().len(), 6);

    let observed_velocities = ArmJointVector::new(
        trajectory.order().clone(),
        output.velocities().iter().copied().collect(),
    )
    .unwrap();
    let mut displaced = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
    for (index, value) in output.positions().iter().copied().enumerate() {
        displaced.push(if index == 0 {
            ArmPosition::angular_rad(value.value_si() + 0.2).unwrap()
        } else {
            value
        });
    }
    let observed_positions = ArmJointVector::new(trajectory.order().clone(), displaced).unwrap();
    let limits = TrajectoryDeviationLimits::new(6, vec![0.1; 6], vec![0.1; 6]).unwrap();
    let deviation = trajectory
        .check_deviation(
            trajectory.duration_ns() / 2,
            &observed_positions,
            &observed_velocities,
            &limits,
            &mut output,
        )
        .unwrap()
        .unwrap();
    assert_eq!(deviation.kind(), TrajectoryDeviationKind::Position);
    assert_eq!(deviation.index(), 0);
}

#[test]
fn jerk_limited_profile_obeys_limits_and_is_deterministic() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let goal = chain
        .positions_from_values(&[0.8, -0.6, 0.4, 0.2, -0.3, 0.5])
        .unwrap();
    let sampling = TrajectorySampling::new(65).unwrap();
    let first = ArmTrajectory::jerk_limited(chain, &start, &goal, &[20.0; 6], sampling).unwrap();
    let second = ArmTrajectory::jerk_limited(chain, &start, &goal, &[20.0; 6], sampling).unwrap();
    assert_eq!(first, second);
    for point in first.points() {
        assert!(point.jerks_si().iter().all(|value| value.abs() <= 20.0));
    }
}

#[test]
fn cubic_and_path_parameterization_report_exact_violations_and_capacity() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let goal = chain
        .positions_from_values(&[0.2, -0.2, 0.2, 0.1, -0.1, 0.2])
        .unwrap();
    let cubic = ArmTrajectory::cubic_clamped(
        chain,
        &start,
        &goal,
        &[0.0; 6],
        &[0.0; 6],
        1_000_000_000,
        TrajectorySampling::new(17).unwrap(),
    )
    .unwrap();
    assert_eq!(cubic.points().len(), 17);

    let path = ArmTrajectory::parameterize_path(
        chain,
        &[start.clone(), goal.clone(), start.clone()],
        TrajectorySampling::new(9).unwrap(),
    )
    .unwrap();
    assert_eq!(path.points().len(), 17);

    let reverse =
        ArmTrajectory::trapezoidal(chain, &goal, &start, TrajectorySampling::new(9).unwrap())
            .unwrap();
    let blended = ArmTrajectory::blend_sequence(
        chain,
        &path,
        &reverse,
        1_000_000_000,
        TrajectorySampling::new(9).unwrap(),
    )
    .unwrap();
    assert!(blended.points().len() > path.points().len());

    let too_fast = ArmTrajectory::cubic_clamped(
        chain,
        &start,
        &goal,
        &[100.0; 6],
        &[100.0; 6],
        10_000_000,
        TrajectorySampling::new(8).unwrap(),
    );
    assert!(matches!(
        too_fast,
        Err(ArmAlgorithmError::TrajectoryViolation {
            kind: TrajectoryViolationKind::Velocity,
            ..
        })
    ));
    assert!(TrajectorySampling::new(1).is_err());
    assert!(TrajectorySampling::new(message_model::MAX_TRAJECTORY_POINTS + 1).is_err());
}
