mod common;

use std::{hint::black_box, time::Instant};

use allocation_counter::measure;
use arm_algorithms::{
    kinematics::{IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics},
    planning::{
        ArmStateValidator, CartesianConfig, IrisConfig, JointLimitsOnly, ManifoldProjector,
        OptimizationConfig, OptimizationWorkspace, PlannerConfig, PlannerWorkspace,
        PreparedCartesianPlanner, PreparedChomp, PreparedConstraintRegions, PreparedPlanning,
        PreparedRegionGraph, PreparedStomp, RegionConstraintProvider, TrajectoryCost,
    },
};
use common::{id, performance::report, synthetic_model};

const SAMPLES: usize = 500;
const OPTIMIZER_SAMPLES: usize = 200;

struct CentralBox {
    generation: u64,
}

impl ArmStateValidator for CentralBox {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.15 && state_si[1].abs() < 0.15 {
            Err("central_box")
        } else {
            Ok(())
        }
    }
}

impl RegionConstraintProvider for CentralBox {
    fn generation(&self) -> u64 {
        self.generation
    }

    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        ArmStateValidator::validate(self, state_si)
    }

    fn certify_box(&self, lower: &[f64], upper: &[f64]) -> Result<bool, &'static str> {
        Ok(upper[0] <= -0.15 || lower[0] >= 0.15 || upper[1] <= -0.15 || lower[1] >= 0.15)
    }
}

struct UpperHalf;

impl ArmStateValidator for UpperHalf {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        (state_si[1] >= 0.2).then_some(()).ok_or("upper_half")
    }
}

struct BowlCost;

impl TrajectoryCost for BowlCost {
    fn generation(&self) -> u64 {
        31
    }

    fn evaluate(&self, state_si: &[f64]) -> Result<f64, &'static str> {
        Ok((state_si[1] - 0.4).powi(2))
    }

    fn gradient(&self, state_si: &[f64], output: &mut [f64]) -> Result<(), &'static str> {
        output.fill(0.0);
        output[1] = 2.0 * (state_si[1] - 0.4);
        Ok(())
    }
}

struct FixedSecondJoint;

impl ManifoldProjector for FixedSecondJoint {
    fn generation(&self) -> u64 {
        31
    }

    fn project(&self, input: &[f64], output: &mut [f64]) -> Result<u32, &'static str> {
        output.copy_from_slice(input);
        output[1] = 0.5;
        Ok(1)
    }
}

struct CentralQ0Q2;

impl ArmStateValidator for CentralQ0Q2 {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.15 && state_si[2].abs() < 0.15 {
            Err("central_q0_q2")
        } else {
            Ok(())
        }
    }
}

#[test]
#[ignore = "release-only isolated host latency gate"]
fn prepared_optimization_planners_meet_microsecond_targets() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 130).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 31).unwrap();
    let region_provider = CentralBox { generation: 31 };
    let region_values = [
        [-0.8, -0.8, 0.0, 0.0, 0.0, 0.0],
        [-0.7, -0.3, 0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.25, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.25, 0.0, 0.0, 0.0, 0.0],
        [0.7, 0.3, 0.0, 0.0, 0.0, 0.0],
        [0.8, 0.8, 0.0, 0.0, 0.0, 0.0],
    ];
    let region_seeds = region_values
        .iter()
        .map(|values| chain.positions_from_values(values).unwrap())
        .collect::<Vec<_>>();
    let regions = PreparedConstraintRegions::build(
        &prepared,
        &region_seeds,
        IrisConfig::new(16, 0.1, 1.0e-4, 16).unwrap(),
        &region_provider,
    )
    .unwrap();
    let planner_config = PlannerConfig::new(
        [131; 32], 20_000, 20_000, 20_000, 50_000_000, 0.2, 0.04, 0.15,
    )
    .unwrap();
    let mut planner_workspace = PlannerWorkspace::new(&prepared, planner_config).unwrap();
    let graph = PreparedRegionGraph::build(
        &prepared,
        &regions,
        planner_config,
        &region_provider,
        &mut planner_workspace,
    )
    .unwrap();
    graph
        .query(
            &prepared,
            &region_seeds[0],
            region_seeds.last().unwrap(),
            planner_config,
            &region_provider,
            &mut planner_workspace,
        )
        .unwrap();

    let mut gcs_query = Vec::with_capacity(SAMPLES);
    let mut gcs_build_query = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            graph
                .query(
                    &prepared,
                    &region_seeds[0],
                    region_seeds.last().unwrap(),
                    planner_config,
                    &region_provider,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
        gcs_query.push(started.elapsed().as_nanos());

        let started = Instant::now();
        let rebuilt = PreparedRegionGraph::build(
            &prepared,
            &regions,
            planner_config,
            &region_provider,
            &mut planner_workspace,
        )
        .unwrap();
        black_box(
            rebuilt
                .query(
                    &prepared,
                    &region_seeds[0],
                    region_seeds.last().unwrap(),
                    planner_config,
                    &region_provider,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
        gcs_build_query.push(started.elapsed().as_nanos());
    }

    let optimization_seed_values = [
        [-0.8, 0.4, 0.0, 0.0, 0.0, 0.0],
        [-0.55, 0.8, 0.0, 0.0, 0.0, 0.0],
        [-0.3, 0.7, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.9, 0.0, 0.0, 0.0, 0.0],
        [0.3, 0.7, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.8, 0.0, 0.0, 0.0, 0.0],
        [0.8, 0.4, 0.0, 0.0, 0.0, 0.0],
    ];
    let optimization_seed = optimization_seed_values
        .iter()
        .map(|values| chain.positions_from_values(values).unwrap())
        .collect::<Vec<_>>();
    let chomp_config =
        OptimizationConfig::new(32, 7, 0.05, 1.0e-8, 8, 0.15, 10.0, [132; 32]).unwrap();
    let stomp_config =
        OptimizationConfig::new(32, 7, 0.05, 1.0e-8, 16, 0.25, 10.0, [133; 32]).unwrap();
    let chomp = PreparedChomp::new(&prepared, chomp_config).unwrap();
    let stomp = PreparedStomp::new(&prepared, stomp_config).unwrap();
    let mut chomp_workspace = OptimizationWorkspace::new(&prepared, chomp_config).unwrap();
    let mut stomp_workspace = OptimizationWorkspace::new(&prepared, stomp_config).unwrap();
    let objective = BowlCost;
    chomp
        .optimize(
            &prepared,
            &optimization_seed,
            planner_config,
            chomp_config,
            &objective,
            &JointLimitsOnly,
            &mut chomp_workspace,
        )
        .unwrap();
    stomp
        .optimize(
            &prepared,
            &optimization_seed,
            planner_config,
            stomp_config,
            &objective,
            &UpperHalf,
            &mut stomp_workspace,
        )
        .unwrap();
    let mut chomp_samples = Vec::with_capacity(OPTIMIZER_SAMPLES);
    let mut stomp_samples = Vec::with_capacity(OPTIMIZER_SAMPLES);
    for _ in 0..OPTIMIZER_SAMPLES {
        let started = Instant::now();
        black_box(
            chomp
                .optimize(
                    &prepared,
                    &optimization_seed,
                    planner_config,
                    chomp_config,
                    &objective,
                    &JointLimitsOnly,
                    &mut chomp_workspace,
                )
                .unwrap(),
        );
        chomp_samples.push(started.elapsed().as_nanos());
        let started = Instant::now();
        black_box(
            stomp
                .optimize(
                    &prepared,
                    &optimization_seed,
                    planner_config,
                    stomp_config,
                    &objective,
                    &UpperHalf,
                    &mut stomp_workspace,
                )
                .unwrap(),
        );
        stomp_samples.push(started.elapsed().as_nanos());
    }

    let cartesian_start = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let cartesian_target_positions = chain
        .positions_from_values(&[0.12, -0.18, 0.22, 0.06, -0.08, 0.05])
        .unwrap();
    let cartesian_target = chain
        .forward(&cartesian_target_positions)
        .unwrap()
        .tool_pose();
    let cartesian_config = CartesianConfig::new(
        100.0,
        10.0,
        1.0,
        0.03,
        false,
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::new(250, 5.0e-3, 1.0e-3, 1.0e-3, 0.2, [134; 32], 50, 2, 36).unwrap(),
    )
    .unwrap();
    let mut cartesian = PreparedCartesianPlanner::new(&prepared, 16, 50).unwrap();
    cartesian
        .plan(
            &prepared,
            &cartesian_start,
            cartesian_target,
            cartesian_config,
            &JointLimitsOnly,
        )
        .unwrap();
    let mut cartesian_samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            cartesian
                .plan(
                    &prepared,
                    &cartesian_start,
                    cartesian_target,
                    cartesian_config,
                    &JointLimitsOnly,
                )
                .unwrap(),
        );
        cartesian_samples.push(started.elapsed().as_nanos());
    }

    let constrained_start = chain
        .positions_from_values(&[-0.8, 0.5, -0.8, 0.0, 0.0, 0.0])
        .unwrap();
    let constrained_goal = chain
        .positions_from_values(&[0.8, 0.5, 0.8, 0.0, 0.0, 0.0])
        .unwrap();
    prepared
        .plan_constrained(
            &constrained_start,
            &constrained_goal,
            planner_config,
            &FixedSecondJoint,
            &CentralQ0Q2,
            &mut planner_workspace,
        )
        .unwrap();
    let mut constrained_samples = Vec::with_capacity(OPTIMIZER_SAMPLES);
    for _ in 0..OPTIMIZER_SAMPLES {
        let started = Instant::now();
        black_box(
            prepared
                .plan_constrained(
                    &constrained_start,
                    &constrained_goal,
                    planner_config,
                    &FixedSecondJoint,
                    &CentralQ0Q2,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
        constrained_samples.push(started.elapsed().as_nanos());
    }

    let gcs_query_result = graph
        .query(
            &prepared,
            &region_seeds[0],
            region_seeds.last().unwrap(),
            planner_config,
            &region_provider,
            &mut planner_workspace,
        )
        .unwrap();
    let gcs_query_allocations = measure(|| {
        black_box(
            graph
                .query(
                    &prepared,
                    &region_seeds[0],
                    region_seeds.last().unwrap(),
                    planner_config,
                    &region_provider,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
    });
    let gcs_build_query_allocations = measure(|| {
        let rebuilt = PreparedRegionGraph::build(
            &prepared,
            &regions,
            planner_config,
            &region_provider,
            &mut planner_workspace,
        )
        .unwrap();
        black_box(
            rebuilt
                .query(
                    &prepared,
                    &region_seeds[0],
                    region_seeds.last().unwrap(),
                    planner_config,
                    &region_provider,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
    });
    let chomp_result = chomp
        .optimize(
            &prepared,
            &optimization_seed,
            planner_config,
            chomp_config,
            &objective,
            &JointLimitsOnly,
            &mut chomp_workspace,
        )
        .unwrap();
    let chomp_allocations = measure(|| {
        black_box(
            chomp
                .optimize(
                    &prepared,
                    &optimization_seed,
                    planner_config,
                    chomp_config,
                    &objective,
                    &JointLimitsOnly,
                    &mut chomp_workspace,
                )
                .unwrap(),
        );
    });
    let stomp_result = stomp
        .optimize(
            &prepared,
            &optimization_seed,
            planner_config,
            stomp_config,
            &objective,
            &UpperHalf,
            &mut stomp_workspace,
        )
        .unwrap();
    let stomp_allocations = measure(|| {
        black_box(
            stomp
                .optimize(
                    &prepared,
                    &optimization_seed,
                    planner_config,
                    stomp_config,
                    &objective,
                    &UpperHalf,
                    &mut stomp_workspace,
                )
                .unwrap(),
        );
    });
    let cartesian_result = cartesian
        .plan(
            &prepared,
            &cartesian_start,
            cartesian_target,
            cartesian_config,
            &JointLimitsOnly,
        )
        .unwrap();
    let cartesian_allocations = measure(|| {
        black_box(
            cartesian
                .plan(
                    &prepared,
                    &cartesian_start,
                    cartesian_target,
                    cartesian_config,
                    &JointLimitsOnly,
                )
                .unwrap(),
        );
    });
    let constrained_result = prepared
        .plan_constrained(
            &constrained_start,
            &constrained_goal,
            planner_config,
            &FixedSecondJoint,
            &CentralQ0Q2,
            &mut planner_workspace,
        )
        .unwrap();
    let constrained_allocations = measure(|| {
        black_box(
            prepared
                .plan_constrained(
                    &constrained_start,
                    &constrained_goal,
                    planner_config,
                    &FixedSecondJoint,
                    &CentralQ0Q2,
                    &mut planner_workspace,
                )
                .unwrap(),
        );
    });
    let planner_bytes = planner_workspace.allocated_bytes();
    let gcs_query_report = report(
        "prepared_gcs_query",
        &mut gcs_query,
        gcs_query_allocations,
        planner_bytes,
        u64::from(gcs_query_result.statistics().iterations()),
    );
    let gcs_build_query_report = report(
        "prepared_gcs_build_query",
        &mut gcs_build_query,
        gcs_build_query_allocations,
        planner_bytes,
        0,
    );
    let chomp_report = report(
        "prepared_chomp",
        &mut chomp_samples,
        chomp_allocations,
        chomp.allocated_bytes() + chomp_workspace.allocated_bytes(),
        u64::from(chomp_result.statistics().iterations()),
    );
    let stomp_report = report(
        "prepared_stomp",
        &mut stomp_samples,
        stomp_allocations,
        stomp.allocated_bytes() + stomp_workspace.allocated_bytes(),
        u64::from(stomp_result.statistics().iterations()),
    );
    let cartesian_report = report(
        "prepared_cartesian",
        &mut cartesian_samples,
        cartesian_allocations,
        cartesian.allocated_bytes(),
        cartesian_result.achieved_steps() as u64,
    );
    let constrained_report = report(
        "prepared_constrained_rrt",
        &mut constrained_samples,
        constrained_allocations,
        planner_bytes,
        u64::from(constrained_result.statistics().iterations()),
    );
    assert!(
        gcs_query_report.p95_ns() <= 200_000,
        "prepared GCS P95 exceeded 200 us"
    );
    for (name, report) in [
        ("gcs_build_query", gcs_build_query_report),
        ("chomp", chomp_report),
        ("stomp", stomp_report),
        ("cartesian", cartesian_report),
        ("constrained_rrt", constrained_report),
    ] {
        assert!(
            report.p95_ns() <= 1_000_000,
            "{name} P95 exceeded 1 ms: {} ns",
            report.p95_ns()
        );
    }
}
