mod common;

use arm_algorithms::{
    ArmAlgorithmError,
    grasp::{
        GraspKind, GripperDescription, PrimitiveObject, PrimitiveShape,
        evaluate_grasp_reachability, generate_grasp_candidates, require_grasp_collision_filter,
    },
    kinematics::{ArmPose, DampedLeastSquaresConfig, KinematicChain},
};

use common::{id, synthetic_model};

#[test]
fn primitive_candidates_are_deterministic_ranked_and_collision_unchecked() {
    let object = PrimitiveObject::new(
        id("part"),
        ArmPose::new([0.4, 0.0, 0.4], [0.0, 0.0, 0.0, 1.0]).unwrap(),
        PrimitiveShape::Box {
            half_extents_m: [0.03, 0.02, 0.04],
        },
    )
    .unwrap();
    let gripper = GripperDescription::new(0.1, 0.08, Some(0.01)).unwrap();

    let first = generate_grasp_candidates(object, gripper).unwrap();
    let second = generate_grasp_candidates(object, gripper).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.first().unwrap().kind(), GraspKind::Suction);
    assert_eq!(first.first().unwrap().score(), 0.9);
    assert!(
        first
            .windows(2)
            .all(|pair| pair[0].score() >= pair[1].score())
    );
    assert!(matches!(
        require_grasp_collision_filter(),
        Err(ArmAlgorithmError::CollisionCapabilityUnavailable {
            surface: "grasp_filter"
        })
    ));
}

#[test]
fn invalid_geometry_and_tool_dimensions_fail_closed() {
    assert!(matches!(
        PrimitiveShape::Sphere { radius_m: 0.0 }.validate(),
        Err(ArmAlgorithmError::NonFinite {
            field: "grasp.primitive_shape"
        })
    ));
    assert!(GripperDescription::new(f64::NAN, 0.08, None).is_err());
    assert!(GripperDescription::new(0.1, 0.08, Some(-0.01)).is_err());
}

#[test]
fn reachability_preserves_every_unchecked_candidate() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let initial = chain.positions_from_values(&[0.0; 6]).unwrap();
    let object = PrimitiveObject::new(
        id("part"),
        chain.forward(&initial).unwrap().tool_pose(),
        PrimitiveShape::Sphere { radius_m: 0.01 },
    )
    .unwrap();
    let candidates = generate_grasp_candidates(
        object,
        GripperDescription::new(0.1, 0.05, Some(0.005)).unwrap(),
    )
    .unwrap();

    let evaluated = evaluate_grasp_reachability(
        chain,
        &candidates,
        &initial,
        DampedLeastSquaresConfig::host_default(),
    );
    assert_eq!(evaluated.len(), candidates.len());
    assert!(
        evaluated
            .iter()
            .zip(&candidates)
            .all(|(reachability, candidate)| reachability.candidate() == candidate)
    );
    assert!(evaluated.iter().any(|value| value.solution().is_some()));
}
