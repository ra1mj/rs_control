mod common;

use arm_algorithms::{
    ArmAlgorithmError, CollisionUnchecked,
    kinematics::KinematicChain,
    planning::{
        ArmPlan, ArmStateValidator, JointLimitsOnly, PlannerConfig, PlannerKind, PlanningTopology,
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
    let shortened = first.shortcut(chain, 0.04, &CentralBox).unwrap();
    assert!(shortened.waypoints().len() <= first.waypoints().len());
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
    assert!(matches!(
        plan_joint_space(
            chain,
            PlannerKind::RrtConnect,
            &start,
            &goal,
            config,
            &SeparatingSlab,
        ),
        Err(ArmAlgorithmError::WorkBudgetExhausted { .. })
    ));
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
