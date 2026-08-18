mod common;

use arm_algorithms::{
    ArmAlgorithmError,
    grasp::{
        GripperDescription, PrimitiveObject, PrimitiveShape, evaluate_grasp_reachability,
        generate_grasp_candidates,
    },
    kinematics::{
        ArmPose, DampedLeastSquaresConfig, IkSolveConfig, IkSolverKind, KinematicChain,
        PreparedKinematics,
    },
    planning::{CartesianConfig, JointLimitsOnly, PlannerConfig, PlannerKind, PreparedPlanning},
    task::{
        ArmTaskCompiler, ArmTaskCursor, ArmTaskProgram, ArmTaskProposal, ArmTaskStage,
        ArmTaskState, GripperCommand, MAX_ARM_TASK_STAGES, TaskCompileContext, TaskCompilePolicy,
    },
};

use common::{id, synthetic_model};

#[test]
fn task_program_is_bound_to_profile_group_and_order() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let target = chain.positions_from_values(&[0.0; 6]).unwrap();
    let stages = vec![
        ArmTaskStage::MoveJoint(Box::new(target)),
        ArmTaskStage::MovePose(ArmPose::identity()),
        ArmTaskStage::Gripper(GripperCommand::Open { width_m: 0.05 }),
        ArmTaskStage::Hold { duration_ns: 1 },
    ];

    let program = ArmTaskProgram::new(chain, stages).unwrap();
    assert_eq!(program.profile_id(), model.profile_id());
    assert_eq!(program.group_id(), id("arm"));
    assert_eq!(program.order(), chain.group().joint_order());
}

#[test]
fn cancel_acceptance_is_nonterminal_until_cleanup_completes() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let program = ArmTaskProgram::new(chain, vec![ArmTaskStage::Hold { duration_ns: 1 }]).unwrap();
    let mut cursor = ArmTaskCursor::new(&program);
    assert_eq!(cursor.state(), ArmTaskState::Pending);

    cursor.start().unwrap();
    assert!(cursor.active_stage().is_some());
    cursor.accept_cancel().unwrap();
    assert_eq!(cursor.state(), ArmTaskState::CancelAccepted);
    assert!(cursor.complete_stage().is_err());
    cursor.complete_cancel().unwrap();
    assert_eq!(cursor.state(), ArmTaskState::Cancelled);
}

#[test]
fn stage_progression_and_invalid_programs_are_deterministic() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let program = ArmTaskProgram::new(
        chain,
        vec![
            ArmTaskStage::Hold { duration_ns: 1 },
            ArmTaskStage::Hold { duration_ns: 2 },
        ],
    )
    .unwrap();
    let mut cursor = ArmTaskCursor::new(&program);
    cursor.start().unwrap();
    cursor.complete_stage().unwrap();
    assert_eq!(cursor.state(), ArmTaskState::Running);
    cursor.complete_stage().unwrap();
    assert_eq!(cursor.state(), ArmTaskState::Succeeded);
    assert!(cursor.complete_stage().is_err());

    assert!(matches!(
        ArmTaskProgram::new(chain, Vec::new()),
        Err(ArmAlgorithmError::Capacity {
            field: "arm_task.stages",
            actual: 0,
            maximum: MAX_ARM_TASK_STAGES
        })
    ));
    assert!(ArmTaskProgram::new(chain, vec![ArmTaskStage::Hold { duration_ns: 0 }]).is_err());
    assert!(
        ArmTaskProgram::new(
            chain,
            vec![ArmTaskStage::Recovery(Box::new(vec![ArmTaskStage::Hold {
                duration_ns: 0
            }]))]
        )
        .is_err()
    );
    assert!(
        GripperCommand::Close {
            maximum_effort_n: f64::INFINITY
        }
        .validate()
        .is_err()
    );
}

#[test]
fn task_compiler_binds_every_stage_to_prepared_context_without_authority() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let target = chain
        .positions_from_values(&[0.03, -0.02, 0.01, 0.0, 0.0, 0.0])
        .unwrap();
    let current_pose = chain.forward(&start).unwrap().tool_pose();
    let object = PrimitiveObject::new(
        id("part"),
        current_pose,
        PrimitiveShape::Sphere { radius_m: 0.01 },
    )
    .unwrap();
    let candidates = generate_grasp_candidates(
        object,
        GripperDescription::new(0.1, 0.05, Some(0.005)).unwrap(),
    )
    .unwrap();
    let candidate = evaluate_grasp_reachability(
        chain,
        &candidates,
        &start,
        DampedLeastSquaresConfig::host_default(),
    )
    .into_iter()
    .find(|candidate| candidate.solution().is_some())
    .unwrap()
    .candidate()
    .clone();
    let program = ArmTaskProgram::new(
        chain,
        vec![
            ArmTaskStage::MoveJoint(Box::new(target)),
            ArmTaskStage::MovePose(current_pose),
            ArmTaskStage::Cartesian(Box::new(vec![current_pose])),
            ArmTaskStage::Pick {
                object_id: id("part"),
                candidate: Box::new(candidate),
            },
            ArmTaskStage::Place {
                object_id: id("part"),
                pose: current_pose,
            },
            ArmTaskStage::Gripper(GripperCommand::Open { width_m: 0.04 }),
            ArmTaskStage::Hold { duration_ns: 1_000 },
        ],
    )
    .unwrap();
    let prepared_kinematics = PreparedKinematics::new(chain, 41).unwrap();
    let prepared = PreparedPlanning::new(&prepared_kinematics, 19).unwrap();
    let policy = TaskCompilePolicy::new(
        PlannerKind::RrtConnect,
        PlannerConfig::host_default([11; 32]),
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::host_default([12; 32]),
        CartesianConfig::host_default([13; 32]),
        16,
    )
    .unwrap();
    let compiler = ArmTaskCompiler::new(&program, &prepared, policy).unwrap();
    let context = compiler.context().clone();
    let mut workspace = compiler.workspace().unwrap();
    let mut cursor = ArmTaskCursor::new(&program);
    cursor.start().unwrap();

    for expected_stage in 0..program.stages().len() {
        let request = cursor.begin_compile(context.clone()).unwrap();
        assert_eq!(request.stage_index(), expected_stage);
        let compiled = compiler
            .compile(request, &start, &JointLimitsOnly, &mut workspace)
            .unwrap();
        match (expected_stage, compiled.proposal()) {
            (0 | 1, ArmTaskProposal::Motion(plan)) => {
                assert_eq!(plan.profile_id(), model.profile_id())
            }
            (2, ArmTaskProposal::Cartesian(plans)) => assert_eq!(plans.len(), 1),
            (
                3,
                ArmTaskProposal::Pick {
                    candidate,
                    approach,
                    ..
                },
            ) => {
                assert_eq!(candidate.object_id(), id("part"));
                assert_eq!(approach.group_id(), id("arm"));
            }
            (4, ArmTaskProposal::Place { approach, .. }) => {
                assert_eq!(approach.group_id(), id("arm"))
            }
            (5, ArmTaskProposal::Gripper(GripperCommand::Open { width_m })) => {
                assert_eq!(*width_m, 0.04);
            }
            (6, ArmTaskProposal::Hold { duration_ns }) => assert_eq!(*duration_ns, 1_000),
            _ => panic!("stage {expected_stage} returned the wrong proposal kind"),
        }
        cursor.accept_compilation(&compiled, &context).unwrap();
    }
    assert_eq!(cursor.state(), ArmTaskState::Succeeded);
}

#[test]
fn cancelled_or_reloaded_task_result_cannot_advance_the_cursor() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let program = ArmTaskProgram::new(chain, vec![ArmTaskStage::Hold { duration_ns: 1 }]).unwrap();
    let prepared_kinematics = PreparedKinematics::new(chain, 51).unwrap();
    let prepared = PreparedPlanning::new(&prepared_kinematics, 23).unwrap();
    let policy = TaskCompilePolicy::new(
        PlannerKind::RrtConnect,
        PlannerConfig::host_default([21; 32]),
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::host_default([22; 32]),
        CartesianConfig::host_default([23; 32]),
        16,
    )
    .unwrap();
    let compiler = ArmTaskCompiler::new(&program, &prepared, policy).unwrap();
    let context = compiler.context().clone();
    let mut workspace = compiler.workspace().unwrap();
    let mut cursor = ArmTaskCursor::new(&program);
    cursor.start().unwrap();

    let first = cursor.begin_compile(context.clone()).unwrap();
    let late = compiler
        .compile(first, &start, &JointLimitsOnly, &mut workspace)
        .unwrap();
    cursor.invalidate_compilation().unwrap();
    assert!(matches!(
        cursor.accept_compilation(&late, &context),
        Err(ArmAlgorithmError::StaleTaskResult { .. })
    ));
    assert_eq!(cursor.state(), ArmTaskState::Running);

    let next = cursor.begin_compile(context.clone()).unwrap();
    assert!(next.request_id().get() > late.request().request_id().get());
    let fresh = compiler
        .compile(next, &start, &JointLimitsOnly, &mut workspace)
        .unwrap();
    cursor.accept_compilation(&fresh, &context).unwrap();
    assert_eq!(cursor.state(), ArmTaskState::Succeeded);

    let reloaded_kinematics = PreparedKinematics::new(chain, 52).unwrap();
    let reloaded = PreparedPlanning::new(&reloaded_kinematics, 23).unwrap();
    let reloaded_context = TaskCompileContext::new(&program, &reloaded).unwrap();
    assert_ne!(context, reloaded_context);

    let reloaded_compiler = ArmTaskCompiler::new(&program, &reloaded, policy).unwrap();
    let mut reloaded_workspace = reloaded_compiler.workspace().unwrap();
    let mut original_cursor = ArmTaskCursor::new(&program);
    original_cursor.start().unwrap();
    let original_request = original_cursor.begin_compile(context.clone()).unwrap();
    assert!(matches!(
        reloaded_compiler.compile(
            original_request.clone(),
            &start,
            &JointLimitsOnly,
            &mut reloaded_workspace,
        ),
        Err(ArmAlgorithmError::StaleTaskResult { .. })
    ));
    assert!(matches!(
        compiler.compile(
            original_request,
            &start,
            &JointLimitsOnly,
            &mut reloaded_workspace,
        ),
        Err(ArmAlgorithmError::StaleTaskResult { .. })
    ));
}

#[test]
fn task_compiler_rejects_cartesian_workspace_capacity_without_growth() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let current = chain.forward(&start).unwrap().tool_pose();
    let [x, y, z] = current.translation_m();
    let far_pose = ArmPose::new([x + 1.0, y, z], current.quaternion_xyzw()).unwrap();
    let program = ArmTaskProgram::new(
        chain,
        vec![ArmTaskStage::Cartesian(Box::new(vec![far_pose]))],
    )
    .unwrap();
    let kinematics = PreparedKinematics::new(chain, 61).unwrap();
    let planning = PreparedPlanning::new(&kinematics, 29).unwrap();
    let compact_policy = TaskCompilePolicy::new(
        PlannerKind::RrtConnect,
        PlannerConfig::host_default([31; 32]),
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::host_default([32; 32]),
        CartesianConfig::host_default([33; 32]),
        2,
    )
    .unwrap();
    let broad_policy = TaskCompilePolicy::new(
        PlannerKind::RrtConnect,
        PlannerConfig::host_default([31; 32]),
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::host_default([32; 32]),
        CartesianConfig::host_default([33; 32]),
        32,
    )
    .unwrap();
    let compact_compiler = ArmTaskCompiler::new(&program, &planning, compact_policy).unwrap();
    let broad_compiler = ArmTaskCompiler::new(&program, &planning, broad_policy).unwrap();
    let mut workspace = compact_compiler.workspace().unwrap();
    let prepared_bytes = workspace.allocated_bytes();
    let mut cursor = ArmTaskCursor::new(&program);
    cursor.start().unwrap();
    let request = cursor
        .begin_compile(broad_compiler.context().clone())
        .unwrap();

    assert!(matches!(
        broad_compiler.compile(request, &start, &JointLimitsOnly, &mut workspace),
        Err(ArmAlgorithmError::Capacity {
            field: "planner.cartesian_waypoints",
            maximum: 2,
            ..
        })
    ));
    assert_eq!(workspace.allocated_bytes(), prepared_bytes);
}
