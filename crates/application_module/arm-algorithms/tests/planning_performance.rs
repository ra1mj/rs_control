mod common;

use std::{hint::black_box, time::Instant};

use allocation_counter::measure;
use arm_algorithms::{
    kinematics::{KinematicChain, PreparedKinematics},
    planning::{
        ArmStateValidator, PlannerConfig, PlannerKind, PlannerWorkspace, PreparedPlanning,
        PreparedRoadmap, PrmQueryOptions,
    },
};
use common::{id, performance::report, synthetic_model};

const SAMPLES: usize = 1_000;
const FAMILY_SAMPLES: usize = 250;

struct CentralBox;

impl ArmStateValidator for CentralBox {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.15 && state_si[1].abs() < 0.15 {
            Err("central_box")
        } else {
            Ok(())
        }
    }
}

#[test]
#[ignore = "release-only isolated host latency gate"]
fn warm_planning_p95_meets_microsecond_targets() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 50).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 1).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let direct_goal = chain
        .positions_from_values(&[-0.4, -0.3, 0.1, 0.05, 0.0, 0.0])
        .unwrap();
    let config =
        PlannerConfig::new([101; 32], 2_000, 128, 512, 50_000_000, 0.2, 0.04, 0.15).unwrap();
    let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    let roadmap = PreparedRoadmap::build(
        &prepared,
        config,
        PrmQueryOptions::new(12, false).unwrap(),
        &CentralBox,
        &mut workspace,
    )
    .unwrap();
    roadmap
        .query(
            &prepared,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();

    let mut prm_samples = Vec::with_capacity(SAMPLES);
    let mut direct_samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            roadmap
                .query(
                    &prepared,
                    &start,
                    &goal,
                    config,
                    &CentralBox,
                    &mut workspace,
                )
                .unwrap(),
        );
        prm_samples.push(started.elapsed().as_nanos());

        let started = Instant::now();
        black_box(
            prepared
                .plan(
                    PlannerKind::RrtConnect,
                    &start,
                    &direct_goal,
                    config,
                    &CentralBox,
                    &mut workspace,
                )
                .unwrap(),
        );
        direct_samples.push(started.elapsed().as_nanos());
    }
    let prm_result = roadmap
        .query(
            &prepared,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();
    let prm_allocations = measure(|| {
        black_box(
            roadmap
                .query(
                    &prepared,
                    &start,
                    &goal,
                    config,
                    &CentralBox,
                    &mut workspace,
                )
                .unwrap(),
        );
    });
    let direct_result = prepared
        .plan(
            PlannerKind::RrtConnect,
            &start,
            &direct_goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();
    let direct_allocations = measure(|| {
        black_box(
            prepared
                .plan(
                    PlannerKind::RrtConnect,
                    &start,
                    &direct_goal,
                    config,
                    &CentralBox,
                    &mut workspace,
                )
                .unwrap(),
        );
    });
    let prepared_bytes = workspace.allocated_bytes();
    let prm_report = report(
        "prepared_prm_query",
        &mut prm_samples,
        prm_allocations,
        prepared_bytes,
        u64::from(prm_result.statistics().iterations()),
    );
    let direct_report = report(
        "planner_direct_edge",
        &mut direct_samples,
        direct_allocations,
        prepared_bytes,
        u64::from(direct_result.statistics().iterations()),
    );
    assert!(
        prm_report.p95_ns() <= 200_000,
        "prepared PRM P95 exceeded 200 us"
    );
    assert!(
        direct_report.p95_ns() <= 1_000_000,
        "nominal planner P95 exceeded 1 ms"
    );

    for planner in [
        PlannerKind::RrtConnect,
        PlannerKind::RrtStar,
        PlannerKind::BidirectionalRrtStar,
        PlannerKind::BiTrrt,
        PlannerKind::Est,
        PlannerKind::Kpiece,
    ] {
        let mut samples = Vec::with_capacity(FAMILY_SAMPLES);
        for _ in 0..FAMILY_SAMPLES {
            let started = Instant::now();
            black_box(
                prepared
                    .plan(planner, &start, &goal, config, &CentralBox, &mut workspace)
                    .unwrap(),
            );
            samples.push(started.elapsed().as_nanos());
        }
        let result = prepared
            .plan(planner, &start, &goal, config, &CentralBox, &mut workspace)
            .unwrap();
        let allocations = measure(|| {
            black_box(
                prepared
                    .plan(planner, &start, &goal, config, &CentralBox, &mut workspace)
                    .unwrap(),
            );
        });
        let kernel = format!("planner_{}", planner.name());
        let performance = report(
            &kernel,
            &mut samples,
            allocations,
            workspace.allocated_bytes(),
            u64::from(result.statistics().iterations()),
        );
        assert!(
            performance.p95_ns() <= 1_000_000,
            "{} P95 exceeded 1 ms",
            planner.name()
        );
    }
}
