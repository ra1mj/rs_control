mod common;

use allocation_counter::measure;
use arm_algorithms::{
    ArmAlgorithmError,
    adaptive_gravity::{AdaptiveGravityConfig, AdaptiveGravityEstimator, AdaptiveSampleContext},
    dynamics::PreparedGravity,
    kinematics::{KinematicChain, PreparedKinematics},
};
use common::{id, synthetic_model};

#[test]
fn prepared_fk_and_gravity_have_zero_warm_allocations() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let positions = chain
        .positions_from_values(&[0.1, -0.2, 0.15, 0.05, -0.1, 0.2])
        .unwrap();
    let prepared_fk = PreparedKinematics::new(chain, 1).unwrap();
    let mut fk_workspace = prepared_fk.workspace(64).unwrap();
    let prepared_gravity = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 1).unwrap();
    let mut gravity_workspace = prepared_gravity.workspace();
    let mut regressor = [0.0; 24];
    prepared_fk.forward(&positions, &mut fk_workspace).unwrap();
    prepared_gravity
        .payload_regressor(&positions, &mut gravity_workspace, &mut regressor)
        .unwrap();

    let fk_allocations = measure(|| {
        std::hint::black_box(prepared_fk.forward(&positions, &mut fk_workspace).unwrap());
    });
    let gravity_allocations = measure(|| {
        std::hint::black_box(
            prepared_gravity
                .gravity_compensation(&positions, &mut gravity_workspace)
                .unwrap(),
        );
        prepared_gravity
            .payload_regressor(&positions, &mut gravity_workspace, &mut regressor)
            .unwrap();
    });

    let mut estimator =
        AdaptiveGravityEstimator::new(&prepared_gravity, AdaptiveGravityConfig::host_default())
            .unwrap();
    estimator.start_collection();
    let context = AdaptiveSampleContext {
        generation: 1,
        fresh: true,
        settled: true,
        saturated: false,
        contact_free: true,
    };
    let rows = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 1.0, 1.0],
    ];
    let batch_regressor = rows.into_iter().flatten().collect::<Vec<_>>();
    let theta = [2.0, 0.2, -0.1, 0.4];
    let residual = rows.map(|row| {
        row.into_iter()
            .zip(theta)
            .map(|(left, right)| left * right)
            .sum()
    });
    estimator
        .observe_window(context, &batch_regressor, &residual, 1.0)
        .unwrap();
    estimator.solve_batch().unwrap();
    estimator
        .validate_candidate(&batch_regressor, &residual, 1.0e-9)
        .unwrap();
    estimator.activate().unwrap();
    let mut adaptive_output = [0.0; 6];
    let adaptive_allocations = measure(|| {
        estimator
            .evaluate(&regressor, &mut adaptive_output)
            .unwrap();
        assert!(estimator.update_rls(context, rows[0], residual[0]).unwrap());
    });
    assert_eq!(fk_allocations.count_total, 0);
    assert_eq!(gravity_allocations.count_total, 0);
    assert_eq!(adaptive_allocations.count_total, 0);

    let outside = chain
        .positions_from_values(&[3.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    assert!(matches!(
        prepared_gravity.gravity_compensation(&outside, &mut gravity_workspace),
        Err(ArmAlgorithmError::JointLimit { .. })
    ));
    let other_generation = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 2).unwrap();
    assert!(matches!(
        other_generation.gravity_compensation(&positions, &mut gravity_workspace),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 2,
            actual: 1
        })
    ));
}
