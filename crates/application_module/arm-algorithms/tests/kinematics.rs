use arm_algorithms::{
    ArmAlgorithmError,
    kinematics::{DampedLeastSquaresConfig, IkSolverKind, KinematicChain, WorkspaceSamplingConfig},
};
use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use message_model::{ArmJointId, AsciiId};

const SYNTHETIC_6DOF: &str = include_str!("../../arm-model/tests/fixtures/synthetic_6dof.urdf");

fn id(value: &str) -> ArmJointId {
    AsciiId::parse(value).unwrap()
}

fn synthetic_model() -> ArmModel {
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

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

#[test]
fn zero_pose_and_jacobian_match_golden_geometry() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let zero = chain.positions_from_values(&[0.0; 6]).unwrap();
    let forward = chain.forward(&zero).unwrap();
    assert_eq!(forward.link_poses().len(), 8);
    let translation = forward.tool_pose().translation_m();
    assert_near(translation[0], 0.91, 1.0e-12);
    assert_near(translation[1], 0.0, 1.0e-12);
    assert_near(translation[2], 0.1, 1.0e-12);

    let jacobian = chain.jacobian(&zero).unwrap();
    assert_eq!(jacobian.columns(), 6);
    assert_near(jacobian.get(1, 0).unwrap(), 0.91, 1.0e-12);
    assert_near(jacobian.get(2, 1).unwrap(), -0.66, 1.0e-12);
    assert_near(jacobian.get(2, 2).unwrap(), -0.44, 1.0e-12);
    assert_near(jacobian.get(2, 4).unwrap(), -0.14, 1.0e-12);
    assert_near(jacobian.get(5, 0).unwrap(), 1.0, 1.0e-12);
    assert!(jacobian.analyze(1.0e-9).unwrap().is_singular());
}

#[test]
fn dls_recovers_a_reachable_pose_and_reports_unavailable_solvers() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let target_positions = chain
        .positions_from_values(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.15])
        .unwrap();
    let target = chain.forward(&target_positions).unwrap().tool_pose();
    let initial = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let solution = chain
        .solve_ik(
            IkSolverKind::Auto,
            target,
            &initial,
            DampedLeastSquaresConfig::host_default(),
        )
        .unwrap();
    assert_eq!(solution.solver(), IkSolverKind::DampedLeastSquares);
    assert!(solution.position_error_m() <= 1.0e-5);
    assert!(solution.orientation_error_rad() <= 1.0e-5);

    assert!(matches!(
        chain.solve_ik(
            IkSolverKind::Opw,
            target,
            &initial,
            DampedLeastSquaresConfig::host_default(),
        ),
        Err(ArmAlgorithmError::UnsupportedCapability { capability: "opw" })
    ));
}

#[test]
fn dimensions_limits_and_workspace_sampling_fail_closed_and_repeat() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    assert!(matches!(
        chain.positions_from_values(&[0.0; 5]),
        Err(ArmAlgorithmError::DimensionMismatch { .. })
    ));
    let outside = chain.positions_from_values(&[3.1, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!(outside.is_ok());
    assert!(matches!(
        chain.forward(&outside.unwrap()),
        Err(ArmAlgorithmError::JointLimit { .. })
    ));

    let config =
        WorkspaceSamplingConfig::new([9; 32], 8, [-std::f64::consts::PI, std::f64::consts::PI])
            .unwrap();
    let first = chain.sample_workspace(config).unwrap();
    let second = chain.sample_workspace(config).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.samples().len(), 8);
    assert!(WorkspaceSamplingConfig::new([0; 32], 0, [-1.0, 1.0]).is_err());
}
