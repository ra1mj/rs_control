mod common;

use allocation_counter::measure;
use arm_algorithms::{
    ArmAlgorithmError,
    kinematics::{ArmPose, IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics},
    planning::{
        ArmStateValidator, CartesianConfig, CartesianTermination, IrisConfig, JointLimitsOnly,
        ManifoldProjector, OptimizationConfig, OptimizationWorkspace, PlannerConfig,
        PlannerEvidence, PlannerKind, PlannerWorkspace, PreparedCartesianPlanner, PreparedChomp,
        PreparedConstraintRegions, PreparedPlanning, PreparedRegionGraph, PreparedStomp,
        RegionConstraintProvider, SmoothnessOnly, TrajectoryCost,
    },
};
use common::{id, synthetic_model};

struct UpperHalf;

impl ArmStateValidator for UpperHalf {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[1] >= 0.2 {
            Ok(())
        } else {
            Err("upper_half")
        }
    }
}

struct BowlCost {
    generation: u64,
}

impl TrajectoryCost for BowlCost {
    fn generation(&self) -> u64 {
        self.generation
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

#[test]
fn prepared_chomp_and_stomp_improve_distinct_objectives_deterministically() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 101).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 17).unwrap();
    let seed_values = [
        [-0.8, 0.4, 0.0, 0.0, 0.0, 0.0],
        [-0.55, 0.8, 0.0, 0.0, 0.0, 0.0],
        [-0.3, 0.7, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.9, 0.0, 0.0, 0.0, 0.0],
        [0.3, 0.7, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.8, 0.0, 0.0, 0.0, 0.0],
        [0.8, 0.4, 0.0, 0.0, 0.0, 0.0],
    ];
    let seed = seed_values
        .iter()
        .map(|values| chain.positions_from_values(values).unwrap())
        .collect::<Vec<_>>();
    let planner =
        PlannerConfig::new([51; 32], 2_000, 2_000, 512, 50_000_000, 0.2, 0.04, 0.1).unwrap();
    let chomp_config =
        OptimizationConfig::new(64, 7, 0.05, 1.0e-8, 16, 0.15, 10.0, [52; 32]).unwrap();
    let cost = BowlCost { generation: 17 };
    let chomp = PreparedChomp::new(&prepared, chomp_config).unwrap();
    let mut chomp_workspace = OptimizationWorkspace::new(&prepared, chomp_config).unwrap();
    let chomp_plan = chomp
        .optimize(
            &prepared,
            &seed,
            planner,
            chomp_config,
            &cost,
            &UpperHalf,
            &mut chomp_workspace,
        )
        .unwrap();
    assert_eq!(chomp_plan.waypoints().first(), seed.first());
    assert_eq!(chomp_plan.waypoints().last(), seed.last());
    assert!(matches!(
        chomp_plan.statistics().evidence(),
        PlannerEvidence::Chomp {
            initial_cost,
            final_cost,
            accepted_updates: 1..,
            ..
        } if final_cost < initial_cost
    ));

    let stomp_config =
        OptimizationConfig::new(64, 7, 0.05, 1.0e-8, 32, 0.25, 10.0, [53; 32]).unwrap();
    let stomp = PreparedStomp::new(&prepared, stomp_config).unwrap();
    let mut first_workspace = OptimizationWorkspace::new(&prepared, stomp_config).unwrap();
    let mut second_workspace = OptimizationWorkspace::new(&prepared, stomp_config).unwrap();
    let first = stomp
        .optimize(
            &prepared,
            &seed,
            planner,
            stomp_config,
            &cost,
            &UpperHalf,
            &mut first_workspace,
        )
        .unwrap();
    let second = stomp
        .optimize(
            &prepared,
            &seed,
            planner,
            stomp_config,
            &cost,
            &UpperHalf,
            &mut second_workspace,
        )
        .unwrap();
    assert_eq!(first, second);
    assert_eq!(first.waypoints().first(), seed.first());
    assert_eq!(first.waypoints().last(), seed.last());
    assert!(matches!(
        first.statistics().evidence(),
        PlannerEvidence::Stomp {
            initial_cost,
            final_cost,
            evaluated_rollouts: 1..,
            accepted_updates: 1..,
        } if final_cost < initial_cost
    ));
}

#[test]
fn optimizers_report_non_convergence_for_an_already_minimal_seed() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 102).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 18).unwrap();
    let seed = (0..7)
        .map(|index| {
            let alpha = index as f64 / 6.0;
            chain
                .positions_from_values(&[-0.6 + 1.2 * alpha, 0.4, 0.0, 0.0, 0.0, 0.0])
                .unwrap()
        })
        .collect::<Vec<_>>();
    let planner = PlannerConfig::host_default([61; 32]);
    let objective = SmoothnessOnly::new(18).unwrap();
    for (kind, seed_byte) in [(PlannerKind::Chomp, 62), (PlannerKind::Stomp, 63)] {
        let config =
            OptimizationConfig::new(8, 7, 0.05, 1.0e-9, 8, 0.1, 10.0, [seed_byte; 32]).unwrap();
        let mut workspace = OptimizationWorkspace::new(&prepared, config).unwrap();
        let result = match kind {
            PlannerKind::Chomp => PreparedChomp::new(&prepared, config).unwrap().optimize(
                &prepared,
                &seed,
                planner,
                config,
                &objective,
                &UpperHalf,
                &mut workspace,
            ),
            PlannerKind::Stomp => PreparedStomp::new(&prepared, config).unwrap().optimize(
                &prepared,
                &seed,
                planner,
                config,
                &objective,
                &UpperHalf,
                &mut workspace,
            ),
            _ => unreachable!(),
        };
        assert!(
            matches!(
                &result,
                Err(ArmAlgorithmError::OptimizationNonConvergent { planner, .. })
                    if *planner == kind.name()
            ),
            "{kind:?}: {result:?}"
        );
    }
}

struct CentralBoxProvider {
    generation: u64,
}

impl RegionConstraintProvider for CentralBoxProvider {
    fn generation(&self) -> u64 {
        self.generation
    }

    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.15 && state_si[1].abs() < 0.15 {
            Err("central_box")
        } else {
            Ok(())
        }
    }

    fn certify_box(&self, lower: &[f64], upper: &[f64]) -> Result<bool, &'static str> {
        Ok(upper[0] <= -0.15 || lower[0] >= 0.15 || upper[1] <= -0.15 || lower[1] >= 0.15)
    }
}

impl ArmStateValidator for CentralBoxProvider {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        RegionConstraintProvider::validate(self, state_si)
    }
}

#[test]
fn iris_certifies_regions_and_prepared_gcs_reuses_exact_context() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 103).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 19).unwrap();
    let provider = CentralBoxProvider { generation: 19 };
    let seed_values = [
        [-0.8, -0.8, 0.0, 0.0, 0.0, 0.0],
        [-0.7, -0.3, 0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.25, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.25, 0.0, 0.0, 0.0, 0.0],
        [0.7, 0.3, 0.0, 0.0, 0.0, 0.0],
        [0.8, 0.8, 0.0, 0.0, 0.0, 0.0],
    ];
    let seeds = seed_values
        .iter()
        .map(|values| chain.positions_from_values(values).unwrap())
        .collect::<Vec<_>>();
    let regions = PreparedConstraintRegions::build(
        &prepared,
        &seeds,
        IrisConfig::new(16, 0.1, 1.0e-4, 16).unwrap(),
        &provider,
    )
    .unwrap();
    assert_eq!(regions.regions().len(), seeds.len());
    for region in regions.regions() {
        for corner_mask in 0..(1_usize << prepared.dof()) {
            let point = (0..prepared.dof())
                .map(|joint| {
                    if corner_mask & (1 << joint) == 0 {
                        region.lower_bounds_si()[joint]
                    } else {
                        region.upper_bounds_si()[joint]
                    }
                })
                .collect::<Vec<_>>();
            assert!(region.contains_values(&point));
            RegionConstraintProvider::validate(&provider, &point).unwrap();
        }
    }

    let config = PlannerConfig::new([71; 32], 2_000, 256, 512, 50_000_000, 0.2, 0.04, 0.1).unwrap();
    let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    let graph =
        PreparedRegionGraph::build(&prepared, &regions, config, &provider, &mut workspace).unwrap();
    assert_eq!(graph.region_count(), seeds.len());
    assert!(graph.edge_count() > 0);
    let start = &seeds[0];
    let goal = seeds.last().unwrap();
    let first = graph
        .query(&prepared, start, goal, config, &provider, &mut workspace)
        .unwrap();
    let second = graph
        .query(&prepared, start, goal, config, &provider, &mut workspace)
        .unwrap();
    assert_eq!(first, second);
    assert!(matches!(
        first.statistics().evidence(),
        PlannerEvidence::Gcs {
            regions: 1..,
            edges: 1..,
            expanded_regions: 1..,
            refined_waypoints: 3..,
        }
    ));
    let mut returned_waypoints = 0_usize;
    let allocations = measure(|| {
        let plan = graph
            .query(&prepared, start, goal, config, &provider, &mut workspace)
            .unwrap();
        returned_waypoints = plan.waypoints().len();
        std::hint::black_box(plan);
    });
    assert!(allocations.count_total > 0);
    assert!(
        allocations.count_total <= (returned_waypoints + 1) as u64,
        "prepared GCS scratch grew after warm-up"
    );

    let changed = PreparedPlanning::new(&kinematics, 20).unwrap();
    let mut changed_workspace = PlannerWorkspace::new(&changed, config).unwrap();
    assert!(matches!(
        graph.query(
            &changed,
            start,
            goal,
            config,
            &provider,
            &mut changed_workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 19,
            actual: 20,
        })
    ));
}

#[test]
fn blocked_paths_execute_each_optimization_and_constraint_family() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 106).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 23).unwrap();
    let provider = CentralBoxProvider { generation: 23 };
    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    for planner in [
        PlannerKind::Gcs,
        PlannerKind::Iris,
        PlannerKind::Chomp,
        PlannerKind::Stomp,
        PlannerKind::ConstrainedRrt,
    ] {
        let config = PlannerConfig::new(
            [planner as u8 + 101; 32],
            20_000,
            20_000,
            20_000,
            50_000_000,
            0.2,
            0.04,
            0.15,
        )
        .unwrap();
        let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
        let plan = prepared
            .plan(planner, &start, &goal, config, &provider, &mut workspace)
            .unwrap_or_else(|error| panic!("{} failed: {error}", planner.name()));
        assert_eq!(plan.planner(), planner);
        assert!(matches!(
            (planner, plan.statistics().evidence()),
            (PlannerKind::Gcs, PlannerEvidence::Gcs { regions: 1.., .. })
                | (
                    PlannerKind::Iris,
                    PlannerEvidence::Iris { regions: 1.., .. }
                )
                | (
                    PlannerKind::Chomp,
                    PlannerEvidence::Chomp {
                        accepted_updates: 1..,
                        ..
                    },
                )
                | (
                    PlannerKind::Stomp,
                    PlannerEvidence::Stomp {
                        evaluated_rollouts: 1..,
                        accepted_updates: 1..,
                        ..
                    },
                )
                | (
                    PlannerKind::ConstrainedRrt,
                    PlannerEvidence::ConstrainedRrt {
                        projection_calls: 1..,
                        ..
                    },
                )
        ));
    }
}

struct FixedSecondJoint {
    generation: u64,
}

impl ManifoldProjector for FixedSecondJoint {
    fn generation(&self) -> u64 {
        self.generation
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

struct RejectProjection;

impl ManifoldProjector for RejectProjection {
    fn generation(&self) -> u64 {
        21
    }

    fn project(&self, _input: &[f64], _output: &mut [f64]) -> Result<u32, &'static str> {
        Err("no_projection")
    }
}

struct SeparatingQ0;

impl ArmStateValidator for SeparatingQ0 {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.2 {
            Err("separating_q0")
        } else {
            Ok(())
        }
    }
}

#[test]
fn constrained_rrt_projects_every_extension_and_reports_failure() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 104).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 21).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, 0.5, -0.8, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.5, 0.8, 0.0, 0.0, 0.0])
        .unwrap();
    let config = PlannerConfig::new(
        [81; 32], 20_000, 20_000, 20_000, 50_000_000, 0.2, 0.04, 0.15,
    )
    .unwrap();
    let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    let plan = prepared
        .plan_constrained(
            &start,
            &goal,
            config,
            &FixedSecondJoint { generation: 21 },
            &CentralQ0Q2,
            &mut workspace,
        )
        .unwrap();
    assert!(matches!(
        plan.statistics().evidence(),
        PlannerEvidence::ConstrainedRrt {
            projection_calls: 1..,
            projection_iterations: 1..,
            connect_attempts: 1..,
            ..
        }
    ));
    for waypoint in plan.waypoints() {
        assert!((waypoint.values()[1].value_si() - 0.5).abs() <= 1.0e-12);
    }

    let mut failed_workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    assert!(matches!(
        prepared.plan_constrained(
            &start,
            &goal,
            config,
            &RejectProjection,
            &CentralQ0Q2,
            &mut failed_workspace,
        ),
        Err(ArmAlgorithmError::ProjectionFailed {
            sample_index: 0,
            reason: "no_projection",
            ..
        })
    ));

    let exhausted_config =
        PlannerConfig::new([82; 32], 200, 200, 400, 1_000_000, 0.1, 0.03, 0.1).unwrap();
    let mut exhausted_workspace = PlannerWorkspace::new(&prepared, exhausted_config).unwrap();
    assert!(matches!(
        prepared.plan_constrained(
            &start,
            &goal,
            exhausted_config,
            &FixedSecondJoint { generation: 21 },
            &SeparatingQ0,
            &mut exhausted_workspace,
        ),
        Err(ArmAlgorithmError::WorkBudgetExhausted {
            planner: "constrained_rrt",
            ..
        })
    ));
}

#[test]
fn cartesian_planning_uses_prepared_ik_and_keeps_partial_paths_diagnostic() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 105).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 22).unwrap();
    let start = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let target_positions = chain
        .positions_from_values(&[0.12, -0.18, 0.22, 0.06, -0.08, 0.05])
        .unwrap();
    let target = chain.forward(&target_positions).unwrap().tool_pose();
    let ik = IkSolveConfig::new(250, 5.0e-3, 1.0e-3, 1.0e-3, 0.2, [91; 32], 50, 2, 36).unwrap();
    let config = CartesianConfig::new(
        0.02,
        0.1,
        1.0,
        0.03,
        false,
        IkSolverKind::DampedLeastSquares,
        ik,
    )
    .unwrap();
    let mut cartesian = PreparedCartesianPlanner::new(&prepared, 128, 50).unwrap();
    let complete = cartesian
        .plan(&prepared, &start, target, config, &JointLimitsOnly)
        .unwrap();
    assert!(complete.is_complete());
    assert_eq!(complete.termination(), CartesianTermination::Complete);
    assert_eq!(complete.waypoints().len(), complete.requested_steps() + 1);
    assert_eq!(complete.fraction(), 1.0);

    let partial_ik = CartesianConfig::new(
        100.0,
        10.0,
        1.0,
        0.03,
        true,
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::new(3, 1.0e-5, 1.0e-5, 1.0e-3, 0.2, [92; 32], 8, 2, 4).unwrap(),
    )
    .unwrap();
    let unreachable = ArmPose::new([10.0, -8.0, 12.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
    let partial = cartesian
        .plan(&prepared, &start, unreachable, partial_ik, &JointLimitsOnly)
        .unwrap();
    assert_eq!(partial.termination(), CartesianTermination::IkFailure);
    assert!(!partial.is_complete());
    assert_eq!(partial.waypoints().len(), 1);

    let jump_limited = CartesianConfig::new(
        100.0,
        10.0,
        1.0e-8,
        0.03,
        true,
        IkSolverKind::DampedLeastSquares,
        ik,
    )
    .unwrap();
    let jump = cartesian
        .plan(&prepared, &start, target, jump_limited, &JointLimitsOnly)
        .unwrap();
    assert_eq!(jump.termination(), CartesianTermination::JointJump);
    assert_eq!(jump.waypoints().len(), 1);
}

#[test]
fn every_prepared_artifact_rejects_constraint_generation_changes() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 107).unwrap();
    let original = PreparedPlanning::new(&kinematics, 40).unwrap();
    let changed = PreparedPlanning::new(&kinematics, 41).unwrap();
    let planner_config =
        PlannerConfig::new([111; 32], 200, 64, 128, 1_000_000, 0.2, 0.04, 0.1).unwrap();
    let optimization =
        OptimizationConfig::new(8, 7, 0.05, 1.0e-8, 8, 0.1, 10.0, [112; 32]).unwrap();
    let chomp = PreparedChomp::new(&original, optimization).unwrap();
    let stomp = PreparedStomp::new(&original, optimization).unwrap();
    let mut changed_optimization = OptimizationWorkspace::new(&changed, optimization).unwrap();
    let changed_cost = SmoothnessOnly::new(41).unwrap();
    assert!(matches!(
        chomp.optimize(
            &changed,
            &[],
            planner_config,
            optimization,
            &changed_cost,
            &JointLimitsOnly,
            &mut changed_optimization,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 40,
            actual: 41,
        })
    ));
    assert!(matches!(
        stomp.optimize(
            &changed,
            &[],
            planner_config,
            optimization,
            &changed_cost,
            &JointLimitsOnly,
            &mut changed_optimization,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 40,
            actual: 41,
        })
    ));

    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let provider = CentralBoxProvider { generation: 40 };
    let regions = PreparedConstraintRegions::build(
        &original,
        &[start.clone(), goal.clone()],
        IrisConfig::new(4, 0.05, 1.0e-4, 8).unwrap(),
        &provider,
    )
    .unwrap();
    let mut original_workspace = PlannerWorkspace::new(&original, planner_config).unwrap();
    let graph = PreparedRegionGraph::build(
        &original,
        &regions,
        planner_config,
        &provider,
        &mut original_workspace,
    )
    .unwrap();
    let mut changed_workspace = PlannerWorkspace::new(&changed, planner_config).unwrap();
    assert!(matches!(
        graph.query(
            &changed,
            &start,
            &goal,
            planner_config,
            &JointLimitsOnly,
            &mut changed_workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 40,
            actual: 41,
        })
    ));
    assert!(matches!(
        PreparedRegionGraph::build(
            &changed,
            &regions,
            planner_config,
            &JointLimitsOnly,
            &mut changed_workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 40,
            actual: 41,
        })
    ));

    let mut cartesian = PreparedCartesianPlanner::new(&original, 16, 8).unwrap();
    let target = chain.forward(&start).unwrap().tool_pose();
    assert!(matches!(
        cartesian.plan(
            &changed,
            &start,
            target,
            CartesianConfig::host_default([113; 32]),
            &JointLimitsOnly,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 40,
            actual: 41,
        })
    ));
    assert!(matches!(
        changed.plan_constrained(
            &start,
            &goal,
            planner_config,
            &FixedSecondJoint { generation: 40 },
            &JointLimitsOnly,
            &mut changed_workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 41,
            actual: 40,
        })
    ));
}
