mod common;

use allocation_counter::measure;
use arm_algorithms::{
    ArmAlgorithmError,
    kinematics::{
        DampedLeastSquaresConfig, IkCache, IkSolveConfig, IkSolverKind, KinematicChain,
        PreparedKinematics, WorkspaceSamplingConfig,
    },
};
use common::{id, mixed_model, opw_model, paden_model, redundant_model, synthetic_model};

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

#[test]
fn prepared_generic_kernels_are_distinct_and_context_bound() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let prepared = PreparedKinematics::new(chain, 7).unwrap();
    let mut workspace = prepared.workspace(64).unwrap();
    let target_positions = chain
        .positions_from_values(&[0.12, -0.18, 0.22, 0.06, -0.08, 0.05])
        .unwrap();
    let target = chain.forward(&target_positions).unwrap().tool_pose();
    let initial = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let config = IkSolveConfig::new(250, 5.0e-3, 1.0e-3, 1.0e-3, 0.2, [11; 32], 50, 2, 36).unwrap();

    for solver in [
        IkSolverKind::DampedLeastSquares,
        IkSolverKind::Sqp,
        IkSolverKind::Fabrik,
    ] {
        let solution = prepared
            .solve(solver, target, &initial, config, &mut workspace)
            .unwrap_or_else(|error| panic!("{solver:?} failed: {error:?}"));
        assert_eq!(solution.solver(), solver);
        assert!(solution.work().evaluations() > 0);
        let repeated = prepared
            .solve(solver, target, &initial, config, &mut workspace)
            .unwrap_or_else(|error| panic!("{solver:?} repeat failed: {error:?}"));
        assert_eq!(solution, repeated);
    }

    let other_generation = PreparedKinematics::new(chain, 8).unwrap();
    assert!(matches!(
        other_generation.solve(
            IkSolverKind::DampedLeastSquares,
            target,
            &initial,
            config,
            &mut workspace,
        ),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 8,
            actual: 7,
        })
    ));
}

#[test]
fn solver_admission_and_unreachable_failures_are_typed() {
    assert!(matches!(
        IkSolveConfig::new(0, 1.0e-5, 1.0e-5, 1.0e-3, 0.2, [0; 32], 8, 2, 4),
        Err(ArmAlgorithmError::InvalidBudget {
            field: "ik.max_iterations"
        })
    ));
    assert!(matches!(
        IkSolveConfig::new(8, f64::NAN, 1.0e-5, 1.0e-3, 0.2, [0; 32], 8, 2, 4),
        Err(ArmAlgorithmError::NonFinite {
            field: "ik.position_tolerance_m"
        })
    ));

    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let prepared = PreparedKinematics::new(chain, 30).unwrap();
    let initial = chain.positions_from_values(&[0.0; 6]).unwrap();
    let outside = chain
        .positions_from_values(&[3.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        .unwrap();
    let unreachable =
        arm_algorithms::kinematics::ArmPose::new([10.0, -8.0, 12.0], [0.0, 0.0, 0.0, 1.0]).unwrap();
    let config = IkSolveConfig::new(3, 1.0e-5, 1.0e-5, 1.0e-3, 0.2, [5; 32], 8, 2, 4).unwrap();
    let mut workspace = prepared.workspace(8).unwrap();
    assert!(matches!(
        prepared.solve(
            IkSolverKind::DampedLeastSquares,
            unreachable,
            &outside,
            config,
            &mut workspace,
        ),
        Err(ArmAlgorithmError::JointLimit { .. })
    ));
    for solver in [
        IkSolverKind::DampedLeastSquares,
        IkSolverKind::Fabrik,
        IkSolverKind::Sqp,
        IkSolverKind::BioIk,
    ] {
        assert!(matches!(
            prepared.solve(solver, unreachable, &initial, config, &mut workspace),
            Err(ArmAlgorithmError::NonConvergent { .. })
        ));
    }

    let opw_model = opw_model();
    let opw_chain = KinematicChain::new(&opw_model, id("arm")).unwrap();
    let opw = PreparedKinematics::new(opw_chain, 31).unwrap();
    let opw_initial = opw_chain.positions_from_values(&[0.0; 6]).unwrap();
    let mut opw_workspace = opw.workspace(8).unwrap();
    assert!(matches!(
        opw.solve(
            IkSolverKind::Opw,
            unreachable,
            &opw_initial,
            config,
            &mut opw_workspace,
        ),
        Err(ArmAlgorithmError::NonConvergent { .. })
    ));
}

#[test]
fn analytical_kernels_admit_only_matching_axis_conventions() {
    let config = IkSolveConfig::new(64, 1.0e-8, 1.0e-8, 1.0e-3, 0.2, [23; 32], 32, 2, 17).unwrap();

    let opw_model = opw_model();
    let opw_chain = KinematicChain::new(&opw_model, id("arm")).unwrap();
    let opw = PreparedKinematics::new(opw_chain, 3).unwrap();
    assert!(opw.geometry().supports_opw());
    assert!(!opw.geometry().supports_paden_kahan());
    let opw_target_values = [0.2, -0.3, 0.4, 0.1, -0.2, 0.15];
    let opw_target_positions = opw_chain.positions_from_values(&opw_target_values).unwrap();
    let opw_target = opw_chain
        .forward(&opw_target_positions)
        .unwrap()
        .tool_pose();
    let opw_initial = opw_chain.positions_from_values(&[0.0; 6]).unwrap();
    let mut opw_workspace = opw.workspace(32).unwrap();
    let opw_solution = opw
        .solve(
            IkSolverKind::Opw,
            opw_target,
            &opw_initial,
            config,
            &mut opw_workspace,
        )
        .unwrap();
    assert_eq!(opw_solution.solver(), IkSolverKind::Opw);
    assert!(opw_solution.work().candidates() <= 8);
    assert_eq!(
        opw_solution,
        opw.solve(
            IkSolverKind::Opw,
            opw_target,
            &opw_initial,
            config,
            &mut opw_workspace,
        )
        .unwrap()
    );

    let paden_model = paden_model();
    let paden_chain = KinematicChain::new(&paden_model, id("arm")).unwrap();
    let paden = PreparedKinematics::new(paden_chain, 4).unwrap();
    assert!(!paden.geometry().supports_opw());
    assert!(paden.geometry().supports_paden_kahan());
    let paden_target_positions = paden_chain
        .positions_from_values(&[0.15, -0.25, 0.35, 0.2, -0.15, 0.1])
        .unwrap();
    let paden_target = paden_chain
        .forward(&paden_target_positions)
        .unwrap()
        .tool_pose();
    let paden_initial = paden_chain.positions_from_values(&[0.0; 6]).unwrap();
    let mut paden_workspace = paden.workspace(32).unwrap();
    let paden_solution = paden
        .solve(
            IkSolverKind::PadenKahan,
            paden_target,
            &paden_initial,
            config,
            &mut paden_workspace,
        )
        .unwrap();
    assert_eq!(paden_solution.solver(), IkSolverKind::PadenKahan);
    assert_eq!(
        paden_solution,
        paden
            .solve(
                IkSolverKind::PadenKahan,
                paden_target,
                &paden_initial,
                config,
                &mut paden_workspace,
            )
            .unwrap()
    );

    let seven_model = redundant_model();
    let seven_chain = KinematicChain::new(&seven_model, id("arm")).unwrap();
    let seven = PreparedKinematics::new(seven_chain, 5).unwrap();
    assert!(seven.geometry().supports_seven_axis_analysis());
    let seven_target_positions = seven_chain
        .positions_from_values(&[0.1, -0.2, 0.3, 0.25, 0.15, -0.2, 0.1])
        .unwrap();
    let seven_target = seven_chain
        .forward(&seven_target_positions)
        .unwrap()
        .tool_pose();
    let seven_initial = seven_chain.positions_from_values(&[0.0; 7]).unwrap();
    let mut seven_workspace = seven.workspace(32).unwrap();
    let seven_solution = seven
        .solve(
            IkSolverKind::RedundantSevenDof,
            seven_target,
            &seven_initial,
            config,
            &mut seven_workspace,
        )
        .unwrap();
    assert_eq!(seven_solution.solver(), IkSolverKind::RedundantSevenDof);
    assert_eq!(
        seven_solution,
        seven
            .solve(
                IkSolverKind::RedundantSevenDof,
                seven_target,
                &seven_initial,
                config,
                &mut seven_workspace,
            )
            .unwrap()
    );

    let near_miss_model = synthetic_model();
    let near_miss_chain = KinematicChain::new(&near_miss_model, id("arm")).unwrap();
    let near_miss = PreparedKinematics::new(near_miss_chain, 6).unwrap();
    assert!(!near_miss.geometry().supports_six_axis_analysis());
}

#[test]
fn bio_ik_is_seed_repeatable_and_cache_batch_storage_is_bounded() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let prepared = PreparedKinematics::new(chain, 10).unwrap();
    let zero = chain.positions_from_values(&[0.0; 6]).unwrap();
    let target = chain.forward(&zero).unwrap().tool_pose();
    let config = IkSolveConfig::new(16, 1.0e-8, 1.0e-8, 1.0e-3, 0.2, [77; 32], 32, 2, 8).unwrap();
    let mut first_workspace = prepared.workspace(32).unwrap();
    let required_bytes = first_workspace.allocated_bytes();
    assert!(matches!(
        prepared.workspace_with_byte_limit(32, required_bytes - 1),
        Err(ArmAlgorithmError::Capacity {
            field: "ik.workspace_bytes",
            ..
        })
    ));
    assert!(
        prepared
            .workspace_with_byte_limit(32, required_bytes)
            .is_ok()
    );
    let mut second_workspace = prepared.workspace(32).unwrap();
    let first = prepared
        .solve(
            IkSolverKind::BioIk,
            target,
            &zero,
            config,
            &mut first_workspace,
        )
        .unwrap();
    let second = prepared
        .solve(
            IkSolverKind::BioIk,
            target,
            &zero,
            config,
            &mut second_workspace,
        )
        .unwrap();
    assert_eq!(first, second);
    assert_eq!(first.work().generations(), 1);
    assert_eq!(first.work().candidates(), 32);

    let opw_model = opw_model();
    let opw_chain = KinematicChain::new(&opw_model, id("arm")).unwrap();
    let opw = PreparedKinematics::new(opw_chain, 11).unwrap();
    let values = opw_chain
        .positions_from_values(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.15])
        .unwrap();
    let opw_target = opw_chain.forward(&values).unwrap().tool_pose();
    let initial = opw_chain.positions_from_values(&[0.0; 6]).unwrap();
    let mut workspace = opw.workspace(32).unwrap();
    let mut cache = IkCache::new(&opw, 1, 64 * 1_024, 1.0e-7, 1.0e-7).unwrap();
    let first = opw
        .solve_cached(
            IkSolverKind::Opw,
            opw_target,
            &initial,
            config,
            &mut workspace,
            &mut cache,
        )
        .unwrap();
    assert!(!first.work().cache_hit());
    let hit = opw
        .solve_cached(
            IkSolverKind::Opw,
            opw_target,
            &initial,
            config,
            &mut workspace,
            &mut cache,
        )
        .unwrap();
    assert!(hit.work().cache_hit());
    assert_eq!(cache.len(), 1);
    let different_generation = PreparedKinematics::new(opw_chain, 12).unwrap();
    assert!(matches!(
        cache.lookup(&different_generation, opw_target),
        Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: 12,
            actual: 11,
        })
    ));

    let targets = [opw_target, opw_target];
    let initials = [initial.clone(), initial];
    let mut output = Vec::with_capacity(2);
    opw.solve_batch(
        IkSolverKind::Opw,
        &targets,
        &initials,
        config,
        &mut workspace,
        &mut output,
    )
    .unwrap();
    assert_eq!(output.len(), 2);
    let mut insufficient = Vec::new();
    assert!(matches!(
        opw.solve_batch(
            IkSolverKind::Opw,
            &targets,
            &initials,
            config,
            &mut workspace,
            &mut insufficient,
        ),
        Err(ArmAlgorithmError::Capacity {
            field: "ik.batch_output",
            ..
        })
    ));
}

#[test]
fn generic_admission_supports_mixed_dof_and_warm_solves_do_not_allocate() {
    let model = mixed_model();
    let left_chain = KinematicChain::new(&model, id("left_arm")).unwrap();
    let prepared = PreparedKinematics::new(left_chain, 20).unwrap();
    assert!(!prepared.geometry().all_rotary());
    let initial = left_chain.positions_from_values(&[0.0, 0.0]).unwrap();
    let target = left_chain.forward(&initial).unwrap().tool_pose();
    let config = IkSolveConfig::new(8, 1.0e-8, 1.0e-8, 1.0e-3, 0.2, [9; 32], 8, 2, 4).unwrap();
    let mut workspace = prepared.workspace(8).unwrap();
    for solver in [
        IkSolverKind::DampedLeastSquares,
        IkSolverKind::Sqp,
        IkSolverKind::BioIk,
    ] {
        let solution = prepared
            .solve(solver, target, &initial, config, &mut workspace)
            .unwrap();
        assert_eq!(solution.solver(), solver);
    }
    assert!(matches!(
        prepared.solve(
            IkSolverKind::Fabrik,
            target,
            &initial,
            config,
            &mut workspace,
        ),
        Err(ArmAlgorithmError::UnsupportedGeometry {
            solver: "fabrik",
            ..
        })
    ));

    let wrong_order = KinematicChain::new(&model, id("right_arm"))
        .unwrap()
        .positions_from_values(&[0.0])
        .unwrap();
    assert!(matches!(
        prepared.solve(
            IkSolverKind::DampedLeastSquares,
            target,
            &wrong_order,
            config,
            &mut workspace,
        ),
        Err(ArmAlgorithmError::JointOrderMismatch { .. })
    ));

    let allocations = measure(|| {
        let solution = prepared
            .solve(
                IkSolverKind::DampedLeastSquares,
                target,
                &initial,
                config,
                &mut workspace,
            )
            .unwrap();
        std::hint::black_box(solution);
    });
    assert_eq!(allocations.count_total, 0);
}

#[test]
fn zero_pose_and_jacobian_match_golden_geometry() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let zero = chain.positions_from_values(&[0.0; 6]).unwrap();
    let forward = chain.forward(&zero).unwrap();
    assert_eq!(forward.link_poses().len(), 8);
    let translation = forward.tool_pose().translation_m();
    assert_near(translation[0], 0.91, 1.0e-12);
    assert_near(translation[1], 0.0, 1.0e-12);
    assert_near(translation[2], 0.1, 1.0e-12);

    let jacobian = chain.jacobian(&zero).unwrap();
    assert_eq!(jacobian.columns(), 6);
    assert_near(jacobian.get(1, 0).unwrap(), 0.91, 1.0e-12);
    assert_near(jacobian.get(2, 1).unwrap(), -0.66, 1.0e-12);
    assert_near(jacobian.get(2, 2).unwrap(), -0.44, 1.0e-12);
    assert_near(jacobian.get(2, 4).unwrap(), -0.14, 1.0e-12);
    assert_near(jacobian.get(5, 0).unwrap(), 1.0, 1.0e-12);
    assert!(jacobian.analyze(1.0e-9).unwrap().is_singular());
}

#[test]
fn dls_recovers_a_reachable_pose_and_reports_unavailable_solvers() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let target_positions = chain
        .positions_from_values(&[0.2, -0.3, 0.4, 0.1, -0.2, 0.15])
        .unwrap();
    let target = chain.forward(&target_positions).unwrap().tool_pose();
    let initial = chain
        .positions_from_values(&[0.05, -0.05, 0.05, 0.02, -0.02, 0.01])
        .unwrap();
    let solution = chain
        .solve_ik(
            IkSolverKind::Auto,
            target,
            &initial,
            DampedLeastSquaresConfig::host_default(),
        )
        .unwrap();
    assert_eq!(solution.solver(), IkSolverKind::DampedLeastSquares);
    assert!(solution.position_error_m() <= 1.0e-5);
    assert!(solution.orientation_error_rad() <= 1.0e-5);

    assert!(matches!(
        chain.solve_ik(
            IkSolverKind::Opw,
            target,
            &initial,
            DampedLeastSquaresConfig::host_default(),
        ),
        Err(ArmAlgorithmError::UnsupportedGeometry { solver: "opw", .. })
    ));
}

#[test]
fn dimensions_limits_and_workspace_sampling_fail_closed_and_repeat() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    assert!(matches!(
        chain.positions_from_values(&[0.0; 5]),
        Err(ArmAlgorithmError::DimensionMismatch { .. })
    ));
    let outside = chain.positions_from_values(&[3.1, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!(outside.is_ok());
    assert!(matches!(
        chain.forward(&outside.unwrap()),
        Err(ArmAlgorithmError::JointLimit { .. })
    ));

    let config =
        WorkspaceSamplingConfig::new([9; 32], 8, [-std::f64::consts::PI, std::f64::consts::PI])
            .unwrap();
    let first = chain.sample_workspace(config).unwrap();
    let second = chain.sample_workspace(config).unwrap();
    assert_eq!(first, second);
    assert_eq!(first.samples().len(), 8);
    assert!(WorkspaceSamplingConfig::new([0; 32], 0, [-1.0, 1.0]).is_err());
}
