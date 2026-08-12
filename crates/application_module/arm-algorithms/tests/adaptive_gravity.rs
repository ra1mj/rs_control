mod common;

use arm_algorithms::{
    adaptive_gravity::{
        AdaptiveGravityConfig, AdaptiveGravityEstimator, AdaptiveGravityState,
        AdaptiveSampleContext,
    },
    dynamics::PreparedGravity,
    kinematics::KinematicChain,
};

use common::{id, synthetic_model};

fn context(generation: u64) -> AdaptiveSampleContext {
    AdaptiveSampleContext {
        generation,
        fresh: true,
        settled: true,
        saturated: false,
        contact_free: true,
    }
}

#[test]
fn bounded_batch_qr_validates_activates_and_freezes_rls() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let prepared = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 9).unwrap();
    let mut estimator =
        AdaptiveGravityEstimator::new(&prepared, AdaptiveGravityConfig::host_default()).unwrap();
    estimator.start_collection();

    let rows = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 1.0, 1.0],
    ];
    let theta = [2.0, 0.2, -0.1, 0.4];
    let regressor = rows.into_iter().flatten().collect::<Vec<_>>();
    let residual = rows
        .iter()
        .map(|row| row.iter().zip(theta).map(|(a, b)| a * b).sum())
        .collect::<Vec<f64>>();
    assert!(
        estimator
            .observe_window(context(9), &regressor, &residual, 1.0)
            .unwrap()
    );
    assert!(
        !estimator
            .observe_window(context(9), &regressor, &residual, 1.0)
            .unwrap()
    );
    assert_eq!(estimator.counters().insufficient_excitation, 1);
    let candidate = estimator.solve_batch().unwrap();
    assert!((candidate.mass_kg() - 2.0).abs() < 1.0e-12);
    assert!(
        estimator
            .validate_candidate(&regressor, &residual, 1.0e-9)
            .unwrap()
            < 1.0e-12
    );
    estimator.activate().unwrap();
    assert_eq!(estimator.state(), AdaptiveGravityState::Active);

    let stale = AdaptiveSampleContext {
        fresh: false,
        ..context(9)
    };
    assert!(!estimator.update_rls(stale, rows[0], residual[0]).unwrap());
    assert_eq!(estimator.state(), AdaptiveGravityState::Frozen);
    assert_eq!(estimator.counters().stale, 1);
    assert!(
        estimator
            .update_rls(context(9), rows[0], residual[0])
            .unwrap()
    );
    assert_eq!(estimator.state(), AdaptiveGravityState::Active);
}

#[test]
fn payload_parameter_encoding_rejects_overflowed_first_moments() {
    assert!(
        arm_algorithms::adaptive_gravity::PayloadParameters::from_mass_cog(
            f64::MAX,
            [f64::MAX, 0.0, 0.0],
        )
        .is_err()
    );
}

#[test]
fn identity_and_observability_fail_closed() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let prepared = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 3).unwrap();
    let mut estimator =
        AdaptiveGravityEstimator::new(&prepared, AdaptiveGravityConfig::host_default()).unwrap();
    estimator.start_collection();
    assert!(
        !estimator
            .observe_window(context(4), &[0.0; 24], &[0.0; 6], 1.0)
            .unwrap()
    );
    assert_eq!(estimator.counters().identity_mismatch, 1);
    let mut stale = context(3);
    stale.fresh = false;
    assert!(
        !estimator
            .observe_window(stale, &[0.0; 24], &[0.0; 6], 1.0)
            .unwrap()
    );
    let mut non_finite = [0.0; 24];
    non_finite[0] = f64::NAN;
    assert!(
        !estimator
            .observe_window(context(3), &non_finite, &[0.0; 6], 1.0)
            .unwrap()
    );
    assert_eq!(estimator.counters().stale, 1);
    assert_eq!(estimator.counters().non_finite, 1);
    estimator
        .observe_window(context(3), &[1.0; 24], &[1.0; 6], 1.0)
        .unwrap();
    assert!(estimator.solve_batch().is_err());
    assert_eq!(estimator.state(), AdaptiveGravityState::Rejected);
    assert!(
        AdaptiveGravityConfig::host_default()
            .with_payload_bounds([0.0, 1.0], [[-0.1, 0.1]; 3])
            .is_err()
    );
}
