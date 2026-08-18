mod common;

use std::{hint::black_box, time::Instant};

use allocation_counter::measure;
use arm_algorithms::{
    adaptive_gravity::{AdaptiveGravityConfig, AdaptiveGravityEstimator, AdaptiveSampleContext},
    dynamics::PreparedGravity,
    kinematics::{KinematicChain, PreparedKinematics},
};
use common::{id, performance::report, synthetic_model};

const SAMPLES: usize = 2_000;

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
    let fk_allocations = measure(|| {
        black_box(prepared_fk.forward(&positions, &mut fk_workspace).unwrap());
    });
    let gravity_allocations = measure(|| {
        black_box(
            prepared
                .gravity_compensation(&positions, &mut workspace)
                .unwrap(),
        );
    });
    let regressor_allocations = measure(|| {
        prepared
            .payload_regressor(&positions, &mut workspace, &mut regressor)
            .unwrap();
    });
    let evaluation_allocations = measure(|| {
        estimator.evaluate(&regressor, &mut output).unwrap();
    });
    let rls_allocations = measure(|| {
        black_box(
            estimator
                .update_rls(context, batch_rows[0], residual[0])
                .unwrap(),
        );
    });
    let fk_report = report(
        "prepared_forward_kinematics",
        &mut fk_times,
        fk_allocations,
        fk_workspace.allocated_bytes(),
        chain.dof() as u64,
    );
    let gravity_report = report(
        "prepared_gravity_compensation",
        &mut gravity_times,
        gravity_allocations,
        workspace.allocated_bytes(),
        chain.dof() as u64,
    );
    let regressor_report = report(
        "prepared_payload_regressor",
        &mut regressor_times,
        regressor_allocations,
        workspace.allocated_bytes(),
        regressor.len() as u64,
    );
    let evaluation_report = report(
        "adaptive_gravity_evaluate",
        &mut evaluation_times,
        evaluation_allocations,
        std::mem::size_of_val(&estimator),
        4,
    );
    let rls_report = report(
        "adaptive_gravity_rls",
        &mut rls_times,
        rls_allocations,
        std::mem::size_of_val(&estimator),
        u64::from(estimator.counters().accepted_rls_updates),
    );
    for report in [
        fk_report,
        gravity_report,
        regressor_report,
        evaluation_report,
        rls_report,
    ] {
        assert!(report.p95_ns() <= 50_000);
    }
    assert_eq!(
        estimator.counters().accepted_rls_updates,
        (SAMPLES + 1) as u32
    );
}
