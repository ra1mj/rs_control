mod common;

use std::{hint::black_box, time::Instant};

use arm_algorithms::{
    kinematics::{IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics},
    planning::{CartesianConfig, JointLimitsOnly, PlannerConfig, PlannerKind, PreparedPlanning},
    reactive::{PreparedRmp, RmpConfig},
    task::{ArmTaskCompiler, ArmTaskCursor, ArmTaskProgram, ArmTaskStage, TaskCompilePolicy},
};
use arrayvec::ArrayVec;
use common::{id, performance::report, synthetic_model};
use message_model::{ArmJointKind, ArmJointVector, ArmVelocity, MAX_JOINTS};

const SAMPLES: usize = 2_000;

fn zero_velocities(chain: KinematicChain<'_>) -> ArmJointVector<ArmVelocity> {
    let mut values = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for descriptor in chain.group().joint_order().joints() {
        values.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(0.0)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(0.0),
            }
            .unwrap(),
        );
    }
    ArmJointVector::new(chain.group().joint_order().clone(), values).unwrap()
}

#[test]
#[ignore = "release-only isolated host latency gate"]
fn prepared_reactive_and_task_compilation_report_host_only_release_evidence() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain
        .positions_from_values(&[0.1, -0.2, 0.15, 0.05, -0.1, 0.2])
        .unwrap();
    let target = chain
        .positions_from_values(&[0.15, -0.15, 0.1, 0.05, -0.05, 0.15])
        .unwrap();
    let velocities = zero_velocities(chain);
    let kinematics = PreparedKinematics::new(chain, 71).unwrap();

    let rmp = PreparedRmp::new(
        &kinematics,
        RmpConfig::host_default()
            .with_singularity(3.0, 0.1)
            .unwrap(),
    );
    let mut rmp_workspace = rmp.workspace().unwrap();
    let rmp_first = rmp
        .accelerate(&start, &velocities, &target, &mut rmp_workspace)
        .unwrap();
    let rmp_allocations = allocation_counter::measure(|| {
        black_box(
            rmp.accelerate(&start, &velocities, &target, &mut rmp_workspace)
                .unwrap(),
        );
    });
    assert_eq!(rmp_allocations.count_total, 0);
    let mut rmp_samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            rmp.accelerate(&start, &velocities, &target, &mut rmp_workspace)
                .unwrap(),
        );
        rmp_samples.push(started.elapsed().as_nanos());
    }
    let rmp_report = report(
        "prepared_rmp_accelerate",
        &mut rmp_samples,
        rmp_allocations,
        rmp_workspace.allocated_bytes(),
        u64::from(rmp_first.singularity_rotations()),
    );
    assert!(
        rmp_report.p95_ns() <= 50_000,
        "prepared RMP P95 exceeded 50 us"
    );

    let planning = PreparedPlanning::new(&kinematics, 37).unwrap();
    let policy = TaskCompilePolicy::new(
        PlannerKind::RrtConnect,
        PlannerConfig::host_default([71; 32]),
        IkSolverKind::DampedLeastSquares,
        IkSolveConfig::host_default([72; 32]),
        CartesianConfig::host_default([73; 32]),
        16,
    )
    .unwrap();
    let motion_program = ArmTaskProgram::new(
        chain,
        vec![ArmTaskStage::MoveJoint(Box::new(target.clone()))],
    )
    .unwrap();
    let motion_compiler = ArmTaskCompiler::new(&motion_program, &planning, policy).unwrap();
    let mut motion_workspace = motion_compiler.workspace().unwrap();
    let prepared_motion_bytes = motion_workspace.allocated_bytes();
    let mut motion_cursor = ArmTaskCursor::new(&motion_program);
    motion_cursor.start().unwrap();
    let motion_request = motion_cursor
        .begin_compile(motion_compiler.context().clone())
        .unwrap();
    let motion_first = motion_compiler
        .compile(
            motion_request.clone(),
            &start,
            &JointLimitsOnly,
            &mut motion_workspace,
        )
        .unwrap();
    let motion_work = match motion_first.proposal() {
        arm_algorithms::task::ArmTaskProposal::Motion(plan) => {
            u64::from(plan.statistics().iterations())
        }
        _ => unreachable!("MoveJoint must compile to a motion proposal"),
    };
    let motion_allocations = allocation_counter::measure(|| {
        black_box(
            motion_compiler
                .compile(
                    motion_request.clone(),
                    &start,
                    &JointLimitsOnly,
                    &mut motion_workspace,
                )
                .unwrap(),
        );
    });
    let mut motion_samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            motion_compiler
                .compile(
                    motion_request.clone(),
                    &start,
                    &JointLimitsOnly,
                    &mut motion_workspace,
                )
                .unwrap(),
        );
        motion_samples.push(started.elapsed().as_nanos());
    }
    assert_eq!(motion_workspace.allocated_bytes(), prepared_motion_bytes);
    let motion_report = report(
        "task_compile_move_joint",
        &mut motion_samples,
        motion_allocations,
        prepared_motion_bytes,
        motion_work,
    );
    assert!(
        motion_report.p95_ns() <= 1_000_000,
        "task MoveJoint compile P95 exceeded 1 ms"
    );

    let hold_program =
        ArmTaskProgram::new(chain, vec![ArmTaskStage::Hold { duration_ns: 1 }]).unwrap();
    let hold_compiler = ArmTaskCompiler::new(&hold_program, &planning, policy).unwrap();
    let mut hold_workspace = hold_compiler.workspace().unwrap();
    let prepared_hold_bytes = hold_workspace.allocated_bytes();
    let mut hold_cursor = ArmTaskCursor::new(&hold_program);
    hold_cursor.start().unwrap();
    let hold_request = hold_cursor
        .begin_compile(hold_compiler.context().clone())
        .unwrap();
    let hold_allocations = allocation_counter::measure(|| {
        black_box(
            hold_compiler
                .compile(
                    hold_request.clone(),
                    &start,
                    &JointLimitsOnly,
                    &mut hold_workspace,
                )
                .unwrap(),
        );
    });
    assert_eq!(hold_allocations.count_total, 0);
    let mut hold_samples = Vec::with_capacity(SAMPLES);
    for _ in 0..SAMPLES {
        let started = Instant::now();
        black_box(
            hold_compiler
                .compile(
                    hold_request.clone(),
                    &start,
                    &JointLimitsOnly,
                    &mut hold_workspace,
                )
                .unwrap(),
        );
        hold_samples.push(started.elapsed().as_nanos());
    }
    assert_eq!(hold_workspace.allocated_bytes(), prepared_hold_bytes);
    let hold_report = report(
        "task_compile_hold",
        &mut hold_samples,
        hold_allocations,
        prepared_hold_bytes,
        0,
    );
    assert!(
        hold_report.p95_ns() <= 50_000,
        "task Hold compile P95 exceeded 50 us"
    );
}
