mod common;

use std::{hint::black_box, time::Instant};

use arm_algorithms::kinematics::{
    IkCache, IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics,
};
use common::{id, opw_model, synthetic_model};

const SAMPLES: usize = 2_000;

fn percentile_95(mut samples_ns: Vec<u128>) -> u128 {
    samples_ns.sort_unstable();
    samples_ns[(samples_ns.len() * 95).div_ceil(100) - 1]
}

#[test]
#[ignore = "release-only isolated host latency gate"]
fn warm_ik_p95_meets_microsecond_targets() {
    let opw_model = opw_model();
    let opw_chain = KinematicChain::new(&opw_model, id("arm")).unwrap();
    let opw = PreparedKinematics::new(opw_chain, 1).unwrap();
    let opw_values = opw_chain
        .positions_from_values(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.15])
        .unwrap();
    let opw_target = opw_chain.forward(&opw_values).unwrap().tool_pose();
    let opw_initial = opw_chain.positions_from_values(&[0.0; 6]).unwrap();
    let config = IkSolveConfig::new(250, 1.0e-5, 1.0e-5, 1.0e-3, 0.2, [31; 32], 32, 2, 16).unwrap();
    let mut opw_workspace = opw.workspace(32).unwrap();
    let mut cache = IkCache::new(&opw, 4, 64 * 1_024, 1.0e-7, 1.0e-7).unwrap();
    opw.solve_cached(
        IkSolverKind::Opw,
        opw_target,
        &opw_initial,
        config,
        &mut opw_workspace,
        &mut cache,
    )
    .unwrap();

    let mut analytical = Vec::with_capacity(SAMPLES);
    let mut cached = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            opw.solve(
                IkSolverKind::Opw,
                opw_target,
                &opw_initial,
                config,
                &mut opw_workspace,
            )
            .unwrap(),
        );
        analytical.push(started.elapsed().as_nanos());

        let started = Instant::now();
        black_box(cache.lookup(&opw, opw_target).unwrap().unwrap());
        cached.push(started.elapsed().as_nanos());
    }

    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let numerical = PreparedKinematics::new(chain, 2).unwrap();
    let target_values = chain
        .positions_from_values(&[0.12, -0.18, 0.22, 0.06, -0.08, 0.05])
        .unwrap();
    let target = chain.forward(&target_values).unwrap().tool_pose();
    let initial = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let mut numerical_workspace = numerical.workspace(32).unwrap();
    let mut dls = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            numerical
                .solve(
                    IkSolverKind::DampedLeastSquares,
                    target,
                    &initial,
                    config,
                    &mut numerical_workspace,
                )
                .unwrap(),
        );
        dls.push(started.elapsed().as_nanos());
    }

    let analytical_p95 = percentile_95(analytical);
    let cache_p95 = percentile_95(cached);
    let numerical_p95 = percentile_95(dls);
    eprintln!(
        "IK host Release P95: analytical={analytical_p95} ns cache={cache_p95} ns numerical={numerical_p95} ns"
    );
    assert!(analytical_p95 <= 50_000, "analytical P95 exceeded 50 us");
    assert!(cache_p95 <= 50_000, "cache P95 exceeded 50 us");
    assert!(numerical_p95 <= 500_000, "numerical P95 exceeded 500 us");
}
