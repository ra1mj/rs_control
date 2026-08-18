mod common;

use std::{hint::black_box, time::Instant};

use allocation_counter::measure;
use arm_algorithms::kinematics::{
    IkCache, IkSolution, IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics,
};
use common::{id, opw_model, paden_model, performance::report, redundant_model, synthetic_model};

const SAMPLES: usize = 2_000;

fn report_solver(
    kernel: &str,
    prepared_bytes: usize,
    mut solve: impl FnMut() -> IkSolution,
) -> common::performance::HostPerformanceReport {
    let first = solve();
    let allocations = measure(|| {
        black_box(solve());
    });
    assert_eq!(
        allocations.count_total, 0,
        "{kernel} warm solve allocated {} bytes",
        allocations.bytes_total
    );
    let mut samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(solve());
        samples.push(started.elapsed().as_nanos());
    }
    let work = first.work();
    report(
        kernel,
        &mut samples,
        allocations,
        prepared_bytes,
        u64::from(work.iterations())
            + u64::from(work.evaluations())
            + u64::from(work.generations())
            + u64::from(work.candidates()),
    )
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

    let generic_config =
        IkSolveConfig::new(250, 5.0e-3, 1.0e-3, 1.0e-3, 0.2, [41; 32], 50, 2, 36).unwrap();
    let mut fabrik_workspace = numerical.workspace(50).unwrap();
    let fabrik_report = report_solver("ik_fabrik", fabrik_workspace.allocated_bytes(), || {
        numerical
            .solve(
                IkSolverKind::Fabrik,
                target,
                &initial,
                generic_config,
                &mut fabrik_workspace,
            )
            .unwrap()
    });
    let mut sqp_workspace = numerical.workspace(50).unwrap();
    let sqp_report = report_solver("ik_sqp", sqp_workspace.allocated_bytes(), || {
        numerical
            .solve(
                IkSolverKind::Sqp,
                target,
                &initial,
                generic_config,
                &mut sqp_workspace,
            )
            .unwrap()
    });

    let bio = PreparedKinematics::new(chain, 3).unwrap();
    let bio_initial = chain.positions_from_values(&[0.0; 6]).unwrap();
    let bio_target = chain.forward(&bio_initial).unwrap().tool_pose();
    let bio_config =
        IkSolveConfig::new(16, 1.0e-8, 1.0e-8, 1.0e-3, 0.2, [42; 32], 32, 2, 8).unwrap();
    let mut bio_workspace = bio.workspace(32).unwrap();
    let bio_report = report_solver("ik_bio", bio_workspace.allocated_bytes(), || {
        bio.solve(
            IkSolverKind::BioIk,
            bio_target,
            &bio_initial,
            bio_config,
            &mut bio_workspace,
        )
        .unwrap()
    });

    let paden_model = paden_model();
    let paden_chain = KinematicChain::new(&paden_model, id("arm")).unwrap();
    let paden = PreparedKinematics::new(paden_chain, 4).unwrap();
    let paden_target_positions = paden_chain
        .positions_from_values(&[0.15, -0.25, 0.35, 0.2, -0.15, 0.1])
        .unwrap();
    let paden_target = paden_chain
        .forward(&paden_target_positions)
        .unwrap()
        .tool_pose();
    let paden_initial = paden_chain.positions_from_values(&[0.0; 6]).unwrap();
    let analytical_config =
        IkSolveConfig::new(64, 1.0e-8, 1.0e-8, 1.0e-3, 0.2, [43; 32], 32, 2, 17).unwrap();
    let mut paden_workspace = paden.workspace(32).unwrap();
    let paden_report = report_solver("ik_paden_kahan", paden_workspace.allocated_bytes(), || {
        paden
            .solve(
                IkSolverKind::PadenKahan,
                paden_target,
                &paden_initial,
                analytical_config,
                &mut paden_workspace,
            )
            .unwrap()
    });

    let redundant_model = redundant_model();
    let redundant_chain = KinematicChain::new(&redundant_model, id("arm")).unwrap();
    let redundant = PreparedKinematics::new(redundant_chain, 5).unwrap();
    let redundant_target_positions = redundant_chain
        .positions_from_values(&[0.1, -0.2, 0.3, 0.25, 0.15, -0.2, 0.1])
        .unwrap();
    let redundant_target = redundant_chain
        .forward(&redundant_target_positions)
        .unwrap()
        .tool_pose();
    let redundant_initial = redundant_chain.positions_from_values(&[0.0; 7]).unwrap();
    let mut redundant_workspace = redundant.workspace(32).unwrap();
    let redundant_report = report_solver(
        "ik_redundant_seven_dof",
        redundant_workspace.allocated_bytes(),
        || {
            redundant
                .solve(
                    IkSolverKind::RedundantSevenDof,
                    redundant_target,
                    &redundant_initial,
                    analytical_config,
                    &mut redundant_workspace,
                )
                .unwrap()
        },
    );

    let analytical_result = opw
        .solve(
            IkSolverKind::Opw,
            opw_target,
            &opw_initial,
            config,
            &mut opw_workspace,
        )
        .unwrap();
    let analytical_allocations = measure(|| {
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
    });
    let cache_allocations = measure(|| {
        black_box(cache.lookup(&opw, opw_target).unwrap().unwrap());
    });
    let numerical_result = numerical
        .solve(
            IkSolverKind::DampedLeastSquares,
            target,
            &initial,
            config,
            &mut numerical_workspace,
        )
        .unwrap();
    let numerical_allocations = measure(|| {
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
    });
    assert_eq!(analytical_allocations.count_total, 0);
    assert_eq!(cache_allocations.count_total, 0);
    assert_eq!(numerical_allocations.count_total, 0);
    let analytical_report = report(
        "ik_opw",
        &mut analytical,
        analytical_allocations,
        opw_workspace.allocated_bytes(),
        u64::from(analytical_result.work().evaluations()),
    );
    let cache_report = report(
        "ik_cache_lookup",
        &mut cached,
        cache_allocations,
        opw_workspace.allocated_bytes(),
        1,
    );
    let numerical_report = report(
        "ik_damped_least_squares",
        &mut dls,
        numerical_allocations,
        numerical_workspace.allocated_bytes(),
        u64::from(numerical_result.work().evaluations()),
    );
    assert!(
        analytical_report.p95_ns() <= 50_000,
        "analytical P95 exceeded 50 us"
    );
    assert!(cache_report.p95_ns() <= 50_000, "cache P95 exceeded 50 us");
    assert!(
        numerical_report.p95_ns() <= 500_000,
        "numerical P95 exceeded 500 us"
    );
    for (name, report) in [
        ("fabrik", fabrik_report),
        ("sqp", sqp_report),
        ("bio", bio_report),
        ("paden_kahan", paden_report),
        ("redundant_seven_dof", redundant_report),
    ] {
        assert!(
            report.p95_ns() <= 500_000,
            "{name} P95 exceeded 500 us: {} ns",
            report.p95_ns()
        );
    }
}
