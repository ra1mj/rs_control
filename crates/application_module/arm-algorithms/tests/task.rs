mod common;

use arm_algorithms::{
    ArmAlgorithmError,
    kinematics::{ArmPose, KinematicChain},
    task::{
        ArmTaskCursor, ArmTaskProgram, ArmTaskStage, ArmTaskState, GripperCommand,
        MAX_ARM_TASK_STAGES,
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
