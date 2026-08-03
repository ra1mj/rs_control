mod common;

use allocation_counter::measure;
use arm_algorithms::{
    ArmAlgorithmError, CollisionUnchecked,
    kinematics::{KinematicChain, PreparedKinematics},
    planning::{
        ArmPlan, ArmStateValidator, JointLimitsOnly, PlannerConfig, PlannerEvidence, PlannerKind,
        PlannerWorkspace, PlanningTopology, PreparedPlanning, PreparedRoadmap, PrmQueryOptions,
        plan_joint_space,
    },
};

use common::{id, synthetic_model};

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

struct SeparatingSlab;

impl ArmStateValidator for SeparatingSlab {
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str> {
        if state_si[0].abs() < 0.2 {
            Err("separating_slab")
        } else {
            Ok(())
        }
    }
}

fn assert_unchecked(_: &ArmPlan<CollisionUnchecked>) {}

#[test]
fn all_planner_surfaces_return_bounded_collision_unchecked_direct_paths() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain
        .positions_from_values(&[-0.5, -0.4, 0.1, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[-0.2, -0.1, 0.2, 0.1, -0.1, 0.1])
        .unwrap();
    for planner in [
        PlannerKind::RrtConnect,
        PlannerKind::RrtStar,
        PlannerKind::BidirectionalRrtStar,
        PlannerKind::BiTrrt,
        PlannerKind::Est,
        PlannerKind::Kpiece,
        PlannerKind::Prm,
        PlannerKind::Gcs,
        PlannerKind::Iris,
        PlannerKind::Chomp,
        PlannerKind::Stomp,
        PlannerKind::ConstrainedRrt,
    ] {
        let plan = plan_joint_space(
            chain,
            planner,
            &start,
            &goal,
            PlannerConfig::host_default([planner as u8 + 1; 32]),
            &JointLimitsOnly,
        )
        .unwrap();
        assert_unchecked(&plan);
        assert_eq!(plan.planner(), planner);
        assert!(plan.waypoints().len() >= 2);
    }
}

#[test]
fn sampling_plan_is_seed_deterministic_and_shortcut_preserves_constraints() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let config = PlannerConfig::new(
        [7; 32],
        20_000,
        20_000,
        40_000,
        5_000_000_000,
        0.2,
        0.04,
        0.1,
    )
    .unwrap();
    let first = plan_joint_space(
        chain,
        PlannerKind::RrtConnect,
        &start,
        &goal,
        config,
        &CentralBox,
    )
    .unwrap();
    let second = plan_joint_space(
        chain,
        PlannerKind::RrtConnect,
        &start,
        &goal,
        config,
        &CentralBox,
    )
    .unwrap();
    assert_eq!(first, second);
    let changed_seed = PlannerConfig::new(
        [8; 32],
        20_000,
        20_000,
        40_000,
        5_000_000_000,
        0.2,
        0.04,
        0.1,
    )
    .unwrap();
    let changed = plan_joint_space(
        chain,
        PlannerKind::RrtConnect,
        &start,
        &goal,
        changed_seed,
        &CentralBox,
    )
    .unwrap();
    assert_eq!(changed.seed(), &[8; 32]);
    assert_ne!(first, changed);
    let shortened = first.shortcut(chain, 0.04, &CentralBox).unwrap();
    assert!(shortened.waypoints().len() <= first.waypoints().len());
}

#[test]
fn blocked_direct_path_executes_each_distinct_sampling_family() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    for planner in [
        PlannerKind::RrtConnect,
        PlannerKind::RrtStar,
        PlannerKind::BidirectionalRrtStar,
        PlannerKind::BiTrrt,
        PlannerKind::Est,
        PlannerKind::Kpiece,
        PlannerKind::Prm,
    ] {
        let seed = match planner {
            PlannerKind::RrtStar => 1,
            PlannerKind::BidirectionalRrtStar => 2,
            _ => planner as u8 + 20,
        };
        let config = PlannerConfig::new(
            [seed; 32], 20_000, 20_000, 20_000, 50_000_000, 0.2, 0.04, 0.15,
        )
        .unwrap();
        let plan = plan_joint_space(chain, planner, &start, &goal, config, &CentralBox)
            .unwrap_or_else(|error| panic!("{} failed: {error}", planner.name()));
        assert_eq!(plan.planner(), planner);
        match (planner, plan.statistics().evidence()) {
            (
                PlannerKind::RrtConnect,
                PlannerEvidence::RrtConnect {
                    connect_attempts: 1..,
                },
            ) => {}
            (
                PlannerKind::RrtStar,
                PlannerEvidence::RrtStar {
                    parent_candidates: 1..,
                    rewires: 1..,
                    best_cost,
                    ..
                },
            ) if best_cost.is_finite() => {}
            (
                PlannerKind::BidirectionalRrtStar,
                PlannerEvidence::BidirectionalRrtStar {
                    parent_candidates: 1..,
                    rewires: _,
                    connection_candidates: 1..,
                    best_cost,
                },
            ) if best_cost.is_finite() => {}
            (
                PlannerKind::BiTrrt,
                PlannerEvidence::BiTrrt {
                    transitions: 1..,
                    final_temperature,
                    ..
                },
            ) if final_temperature.is_finite() => {}
            (
                PlannerKind::Est,
                PlannerEvidence::Est {
                    density_evaluations: 1..,
                    sparse_selections: 1..,
                },
            ) => {}
            (
                PlannerKind::Kpiece,
                PlannerEvidence::Kpiece {
                    cells: 1..,
                    border_selections: 1..,
                    successful_expansions: 1..,
                    ..
                },
            ) => {}
            (
                PlannerKind::Prm,
                PlannerEvidence::Prm {
                    roadmap_nodes: 2..,
                    roadmap_edges: 1..,
                    nodes_expanded: 1..,
                    ..
                },
            ) => {}
            (_, evidence) => panic!("unexpected family evidence: {evidence:?}"),
        }
        for waypoint in plan.waypoints() {
            let values = waypoint
                .values()
                .iter()
                .map(|position| position.value_si())
                .collect::<Vec<_>>();
            CentralBox.validate(&values).unwrap();
        }
    }
}

#[test]
fn prepared_prm_reuses_graph_and_rejects_constraint_generation_changes() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 41).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 7).unwrap();
    let config = PlannerConfig::new([91; 32], 4_000, 192, 512, 50_000_000, 0.2, 0.04, 0.1).unwrap();
    let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    let roadmap = PreparedRoadmap::build(
        &prepared,
        config,
        PrmQueryOptions::new(12, true).unwrap(),
        &CentralBox,
        &mut workspace,
    )
    .unwrap();
    assert!(roadmap.statistics().accepted_nodes() > 100);
    assert!(roadmap.statistics().edges() > 0);

    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let first = roadmap
        .query(
            &prepared,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();
    let second = roadmap
        .query(
            &prepared,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();
    assert_eq!(first, second);
    assert!(matches!(
        first.statistics().evidence(),
        PlannerEvidence::Prm {
            lazy_edge_checks: 1..,
            ..
        }
    ));

    let changed = PreparedPlanning::new(&kinematics, 8).unwrap();
    let mut changed_workspace = PlannerWorkspace::new(&changed, config).unwrap();
    assert!(matches!(
        roadmap.query(
            &changed,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut changed_workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 7,
            actual: 8,
        })
    ));
}

#[test]
fn prepared_workspace_warm_kernel_does_not_grow_scratch_storage() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let kinematics = PreparedKinematics::new(chain, 42).unwrap();
    let prepared = PreparedPlanning::new(&kinematics, 9).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, -0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.8, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let config =
        PlannerConfig::new([33; 32], 2_000, 512, 1_024, 50_000_000, 0.2, 0.04, 0.15).unwrap();
    let required_bytes = PlannerWorkspace::required_bytes(config).unwrap();
    assert!(matches!(
        prepared.workspace_with_byte_limit(config, required_bytes - 1),
        Err(ArmAlgorithmError::Capacity {
            field: "planner.workspace_bytes",
            ..
        })
    ));
    let mut workspace = PlannerWorkspace::new(&prepared, config).unwrap();
    prepared
        .plan(
            PlannerKind::RrtConnect,
            &start,
            &goal,
            config,
            &CentralBox,
            &mut workspace,
        )
        .unwrap();

    let mut returned_waypoints = 0;
    let allocations = measure(|| {
        let plan = prepared
            .plan(
                PlannerKind::RrtConnect,
                &start,
                &goal,
                config,
                &CentralBox,
                &mut workspace,
            )
            .unwrap();
        returned_waypoints = plan.waypoints().len();
        std::hint::black_box(plan);
    });
    assert!(allocations.count_total > 0);
    assert!(
        allocations.count_total <= (returned_waypoints + 1) as u64,
        "scratch growth added allocations beyond the owned result"
    );
}

#[test]
fn impossible_constraints_exhaust_budget_and_deferred_topologies_are_typed() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain
        .positions_from_values(&[-0.8, -0.4, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let goal = chain
        .positions_from_values(&[0.8, 0.4, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let config = PlannerConfig::new([3; 32], 100, 100, 200, 1_000_000, 0.1, 0.03, 0.1).unwrap();
    let invalid = chain
        .positions_from_values(&[0.0, 0.4, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    assert!(matches!(
        plan_joint_space(
            chain,
            PlannerKind::RrtConnect,
            &invalid,
            &goal,
            config,
            &SeparatingSlab,
        ),
        Err(ArmAlgorithmError::ConstraintViolation {
            sample_index: 0,
            ..
        })
    ));
    assert!(matches!(
        plan_joint_space(
            chain,
            PlannerKind::RrtConnect,
            &start,
            &invalid,
            config,
            &SeparatingSlab,
        ),
        Err(ArmAlgorithmError::ConstraintViolation {
            sample_index: 1,
            ..
        })
    ));
    for planner in [
        PlannerKind::RrtConnect,
        PlannerKind::RrtStar,
        PlannerKind::BidirectionalRrtStar,
        PlannerKind::BiTrrt,
        PlannerKind::Est,
        PlannerKind::Kpiece,
        PlannerKind::Prm,
    ] {
        assert!(matches!(
            plan_joint_space(chain, planner, &start, &goal, config, &SeparatingSlab),
            Err(ArmAlgorithmError::WorkBudgetExhausted {
                planner: exhausted,
                ..
            }) if exhausted == planner.name()
        ));
    }
    assert!(matches!(
        PlanningTopology::CoordinatedDualArm.require_supported(),
        Err(ArmAlgorithmError::UnsupportedCapability { .. })
    ));
    assert!(matches!(
        PlanningTopology::FloatingBase.require_supported(),
        Err(ArmAlgorithmError::UnsupportedCapability { .. })
    ));
    assert!(matches!(
        PlanningTopology::ParallelMechanism.require_supported(),
        Err(ArmAlgorithmError::UnsupportedCapability { .. })
    ));
}
