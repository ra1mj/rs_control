mod common;

use std::{hint::black_box, time::Instant};

use arm_algorithms::{
    adaptive_gravity::{AdaptiveGravityConfig, AdaptiveGravityEstimator, AdaptiveSampleContext},
    dynamics::PreparedGravity,
    kinematics::{KinematicChain, PreparedKinematics},
};
use common::{id, synthetic_model};

const SAMPLES: usize = 2_000;

fn p95(mut values: Vec<u128>) -> u128 {
    values.sort_unstable();
    values[(values.len() * 95).div_ceil(100) - 1]
}

#[test]
#[ignore = "release-only isolated host latency gate"]
fn prepared_gravity_and_adaptation_p95_meet_50us_gate() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let positions = chain
        .positions_from_values(&[0.1, -0.2, 0.15, 0.05, -0.1, 0.2])
        .unwrap();
    let prepared_fk = PreparedKinematics::new(chain, 1).unwrap();
    let mut fk_workspace = prepared_fk.workspace(64).unwrap();
    let prepared = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 1).unwrap();
    let mut workspace = prepared.workspace();
    let mut regressor = [0.0; 24];
    prepared
        .payload_regressor(&positions, &mut workspace, &mut regressor)
        .unwrap();
    let mut estimator =
        AdaptiveGravityEstimator::new(&prepared, AdaptiveGravityConfig::host_default()).unwrap();
    estimator.start_collection();
    let context = AdaptiveSampleContext {
        generation: 1,
        fresh: true,
        settled: true,
        saturated: false,
        contact_free: true,
    };
    let batch_rows = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 1.0, 0.0, 0.0],
        [1.0, 0.0, 1.0, 1.0],
    ];
    let batch_regressor = batch_rows.into_iter().flatten().collect::<Vec<_>>();
    let theta = [2.0, 0.2, -0.1, 0.4];
    let residual = batch_rows.map(|row| {
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

    let mut fk_times = Vec::with_capacity(SAMPLES);
    let mut gravity_times = Vec::with_capacity(SAMPLES);
    let mut regressor_times = Vec::with_capacity(SAMPLES);
    let mut evaluation_times = Vec::with_capacity(SAMPLES);
    let mut rls_times = Vec::with_capacity(SAMPLES);
    let mut output = [0.0; 6];
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(prepared_fk.forward(&positions, &mut fk_workspace).unwrap());
        fk_times.push(started.elapsed().as_nanos());
        let started = Instant::now();
        black_box(
            prepared
                .gravity_compensation(&positions, &mut workspace)
                .unwrap(),
        );
        gravity_times.push(started.elapsed().as_nanos());
        let started = Instant::now();
        prepared
            .payload_regressor(&positions, &mut workspace, &mut regressor)
            .unwrap();
        regressor_times.push(started.elapsed().as_nanos());
        let started = Instant::now();
        estimator.evaluate(&regressor, &mut output).unwrap();
        evaluation_times.push(started.elapsed().as_nanos());
        let started = Instant::now();
        black_box(
            estimator
                .update_rls(context, batch_rows[0], residual[0])
                .unwrap(),
        );
        rls_times.push(started.elapsed().as_nanos());
    }
    let fk_p95 = p95(fk_times);
    let gravity_p95 = p95(gravity_times);
    let regressor_p95 = p95(regressor_times);
    let evaluation_p95 = p95(evaluation_times);
    let rls_p95 = p95(rls_times);
    eprintln!(
        "prepared FK/gravity/regressor/adaptive/RLS P95: {fk_p95}/{gravity_p95}/{regressor_p95}/{evaluation_p95}/{rls_p95} ns"
    );
    for value in [fk_p95, gravity_p95, regressor_p95, evaluation_p95, rls_p95] {
        assert!(value <= 50_000);
    }
    assert_eq!(estimator.counters().accepted_rls_updates, SAMPLES as u32);
}
