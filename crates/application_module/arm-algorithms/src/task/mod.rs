//! Bounded arm task stages with explicit cancellation progression.

use message_model::{
    ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId,
    MAX_TRAJECTORY_POINTS,
};

use crate::{
    ArmAlgorithmError, CollisionUnchecked,
    grasp::{GraspCandidate, GraspObjectId},
    kinematics::{ArmPose, IkSolveConfig, IkSolverKind, IkWorkspace, KinematicChain},
    planning::{
        ArmPlan, ArmStateValidator, CartesianConfig, CartesianPlan, PlannerConfig, PlannerKind,
        PlannerWorkspace, PreparedCartesianPlanner, PreparedPlanning,
    },
};

/// Maximum stages in one task program or recovery branch.
pub const MAX_ARM_TASK_STAGES: usize = 64;
/// Maximum Cartesian poses in one stage.
pub const MAX_CARTESIAN_STAGE_POSES: usize = 256;

/// Validated gripper intent independent of a hardware protocol.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum GripperCommand {
    /// Open to a finite nonnegative width in metres.
    Open { width_m: f64 },
    /// Close with a finite positive effort ceiling.
    Close { maximum_effort_n: f64 },
}

impl GripperCommand {
    /// Validates semantic SI parameters.
    pub fn validate(self) -> Result<Self, ArmAlgorithmError> {
        let valid = match self {
            Self::Open { width_m } => width_m.is_finite() && width_m >= 0.0,
            Self::Close { maximum_effort_n } => {
                maximum_effort_n.is_finite() && maximum_effort_n > 0.0
            }
        };
        if valid {
            Ok(self)
        } else {
            Err(ArmAlgorithmError::NonFinite {
                field: "arm_task.gripper",
            })
        }
    }
}

/// One bounded project-owned arm task stage.
#[derive(Clone, Debug, PartialEq)]
pub enum ArmTaskStage {
    /// Move to a stable-order joint target.
    MoveJoint(Box<ArmJointVector<ArmPosition>>),
    /// Move to one Cartesian pose.
    MovePose(ArmPose),
    /// Follow a bounded Cartesian pose sequence.
    Cartesian(Box<Vec<ArmPose>>),
    /// Open/close the configured gripper.
    Gripper(GripperCommand),
    /// Pick using a collision-unchecked ranked grasp.
    Pick {
        /// Target object.
        object_id: GraspObjectId,
        /// Selected candidate.
        candidate: Box<GraspCandidate<CollisionUnchecked>>,
    },
    /// Place an object at one pose.
    Place {
        /// Target object.
        object_id: GraspObjectId,
        /// Desired object/tool pose.
        pose: ArmPose,
    },
    /// Hold without granting or extending authority.
    Hold {
        /// Strictly positive hold duration.
        duration_ns: u64,
    },
    /// Bounded recovery branch.
    Recovery(Box<Vec<ArmTaskStage>>),
}

/// Profile/group/order-bound task program.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmTaskProgram {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    stages: Vec<ArmTaskStage>,
}

impl ArmTaskProgram {
    /// Validates every nested stage and exact joint context.
    pub fn new(
        chain: KinematicChain<'_>,
        stages: Vec<ArmTaskStage>,
    ) -> Result<Self, ArmAlgorithmError> {
        validate_stages(chain, &stages, "arm_task.stages")?;
        Ok(Self {
            profile_id: chain.model().profile_id(),
            group_id: chain.group().id(),
            order: chain.group().joint_order().clone(),
            stages,
        })
    }

    /// Returns immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns exact group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns bounded stages.
    #[must_use]
    pub fn stages(&self) -> &[ArmTaskStage] {
        &self.stages
    }
}

/// Task execution lifecycle preserving cancel acceptance versus completion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ArmTaskState {
    /// Program has not started.
    Pending,
    /// One stage may be active.
    Running,
    /// Cancellation was accepted but is not terminal.
    CancelAccepted,
    /// Cancellation cleanup completed.
    Cancelled,
    /// All stages completed.
    Succeeded,
    /// A stage failed.
    Failed,
}

/// Pure cursor over a validated task program.
#[derive(Clone, Debug)]
pub struct ArmTaskCursor<'program> {
    program: &'program ArmTaskProgram,
    state: ArmTaskState,
    next_stage: usize,
    compile_generation: u64,
    next_request_id: u64,
    active_request: Option<TaskCompileRequest>,
}

impl<'program> ArmTaskCursor<'program> {
    /// Creates a pending cursor.
    #[must_use]
    pub const fn new(program: &'program ArmTaskProgram) -> Self {
        Self {
            program,
            state: ArmTaskState::Pending,
            next_stage: 0,
            compile_generation: 1,
            next_request_id: 1,
            active_request: None,
        }
    }

    /// Returns lifecycle state.
    #[must_use]
    pub const fn state(&self) -> ArmTaskState {
        self.state
    }

    /// Starts the program.
    pub fn start(&mut self) -> Result<(), ArmAlgorithmError> {
        if self.state != ArmTaskState::Pending {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "task can start only from pending",
            });
        }
        self.state = ArmTaskState::Running;
        Ok(())
    }

    /// Returns the next stage while running.
    #[must_use]
    pub fn active_stage(&self) -> Option<&'program ArmTaskStage> {
        (self.state == ArmTaskState::Running)
            .then(|| self.program.stages.get(self.next_stage))
            .flatten()
    }

    /// Marks the active stage complete and advances.
    pub fn complete_stage(&mut self) -> Result<(), ArmAlgorithmError> {
        if self.state != ArmTaskState::Running || self.next_stage >= self.program.stages.len() {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "no running task stage can complete",
            });
        }
        if self.active_request.is_some() {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "active task compilation must be accepted or invalidated",
            });
        }
        self.next_stage += 1;
        if self.next_stage == self.program.stages.len() {
            self.state = ArmTaskState::Succeeded;
        }
        Ok(())
    }

    /// Accepts cancellation without claiming terminal cancellation.
    pub fn accept_cancel(&mut self) -> Result<(), ArmAlgorithmError> {
        if self.state != ArmTaskState::Running {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "cancel can be accepted only while running",
            });
        }
        self.invalidate_active_request()?;
        self.state = ArmTaskState::CancelAccepted;
        Ok(())
    }

    /// Completes cancellation cleanup.
    pub fn complete_cancel(&mut self) -> Result<(), ArmAlgorithmError> {
        if self.state != ArmTaskState::CancelAccepted {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "cancel completion requires accepted cancellation",
            });
        }
        self.state = ArmTaskState::Cancelled;
        Ok(())
    }

    /// Marks a running or cancel-cleanup stage failed.
    pub fn fail(&mut self) -> Result<(), ArmAlgorithmError> {
        if !matches!(
            self.state,
            ArmTaskState::Running | ArmTaskState::CancelAccepted
        ) {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "only active task execution can fail",
            });
        }
        self.invalidate_active_request()?;
        self.state = ArmTaskState::Failed;
        Ok(())
    }

    /// Starts one generation-bound asynchronous stage compilation.
    pub fn begin_compile(
        &mut self,
        context: TaskCompileContext,
    ) -> Result<TaskCompileRequest, ArmAlgorithmError> {
        if self.state != ArmTaskState::Running || self.next_stage >= self.program.stages.len() {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "task compilation requires one running stage",
            });
        }
        if self.active_request.is_some() {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "task stage already has an active compilation request",
            });
        }
        context.require_program(self.program)?;
        let request_id = TaskRequestId::new(self.next_request_id)?;
        self.next_request_id =
            self.next_request_id
                .checked_add(1)
                .ok_or(ArmAlgorithmError::InvalidBudget {
                    field: "arm_task.request_id",
                })?;
        let request = TaskCompileRequest {
            request_id,
            stage_index: self.next_stage,
            generation: self.compile_generation,
            context,
        };
        self.active_request = Some(request.clone());
        Ok(request)
    }

    /// Accepts a correlated compilation result and advances exactly one stage.
    pub fn accept_compilation(
        &mut self,
        result: &CompiledTaskStage,
        current_context: &TaskCompileContext,
    ) -> Result<(), ArmAlgorithmError> {
        let active = self
            .active_request
            .as_ref()
            .ok_or(ArmAlgorithmError::StaleTaskResult {
                reason: "no task compilation is active",
            })?;
        if active != result.request() {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "task request identity or generation differs",
            });
        }
        if &active.context != current_context {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "task profile or prepared generation changed",
            });
        }
        self.active_request = None;
        self.complete_stage()
    }

    /// Invalidates pending work after cancellation, profile reload or a task
    /// execution boundary change. A late result can no longer advance state.
    pub fn invalidate_compilation(&mut self) -> Result<(), ArmAlgorithmError> {
        self.invalidate_active_request()
    }

    fn invalidate_active_request(&mut self) -> Result<(), ArmAlgorithmError> {
        self.active_request = None;
        self.compile_generation =
            self.compile_generation
                .checked_add(1)
                .ok_or(ArmAlgorithmError::InvalidBudget {
                    field: "arm_task.compile_generation",
                })?;
        Ok(())
    }
}

/// Nonzero identity for one task-stage compilation attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TaskRequestId(u64);

impl TaskRequestId {
    /// Validates that the externally visible request sequence is nonzero.
    pub fn new(value: u64) -> Result<Self, ArmAlgorithmError> {
        if value == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "arm_task.request_id",
            });
        }
        Ok(Self(value))
    }

    /// Returns the stable request number.
    #[must_use]
    pub const fn get(self) -> u64 {
        self.0
    }
}

/// Exact prepared profile and planning identity accepted by task compilation.
#[derive(Clone, Debug, PartialEq)]
pub struct TaskCompileContext {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    model_generation: u64,
    constraint_generation: u64,
}

impl TaskCompileContext {
    /// Captures the immutable identity of one prepared planning context.
    pub fn new(
        program: &ArmTaskProgram,
        planning: &PreparedPlanning<'_, '_>,
    ) -> Result<Self, ArmAlgorithmError> {
        let context = Self {
            profile_id: planning.profile_id(),
            group_id: planning.group_id(),
            order: planning.order().clone(),
            model_generation: planning.model_generation(),
            constraint_generation: planning.constraint_generation(),
        };
        context.require_program(program)?;
        Ok(context)
    }

    /// Returns the model generation bound into the context.
    #[must_use]
    pub const fn model_generation(&self) -> u64 {
        self.model_generation
    }

    /// Returns the constraint generation bound into the context.
    #[must_use]
    pub const fn constraint_generation(&self) -> u64 {
        self.constraint_generation
    }

    fn require_program(&self, program: &ArmTaskProgram) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != program.profile_id || self.group_id != program.group_id {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "task profile or group differs from prepared context",
            });
        }
        if self.order != program.order {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        Ok(())
    }

    fn require_planning(
        &self,
        planning: &PreparedPlanning<'_, '_>,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != planning.profile_id()
            || self.group_id != planning.group_id()
            || &self.order != planning.order()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "task prepared profile, group or order changed",
            });
        }
        if self.model_generation != planning.model_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.model_generation,
                actual: planning.model_generation(),
            });
        }
        if self.constraint_generation != planning.constraint_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.constraint_generation,
                actual: planning.constraint_generation(),
            });
        }
        Ok(())
    }
}

/// One correlated task-stage compilation request.
#[derive(Clone, Debug, PartialEq)]
pub struct TaskCompileRequest {
    request_id: TaskRequestId,
    stage_index: usize,
    generation: u64,
    context: TaskCompileContext,
}

impl TaskCompileRequest {
    /// Returns the stable request identity.
    #[must_use]
    pub const fn request_id(&self) -> TaskRequestId {
        self.request_id
    }

    /// Returns the exact stage selected by this request.
    #[must_use]
    pub const fn stage_index(&self) -> usize {
        self.stage_index
    }

    /// Returns the cancellation/reload generation.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns the required prepared context.
    #[must_use]
    pub const fn context(&self) -> &TaskCompileContext {
        &self.context
    }
}

/// Immutable solver and bounded-storage policy for a task compiler.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TaskCompilePolicy {
    planner: PlannerKind,
    planner_config: PlannerConfig,
    pose_solver: IkSolverKind,
    pose_config: IkSolveConfig,
    cartesian_config: CartesianConfig,
    maximum_cartesian_waypoints: usize,
}

impl TaskCompilePolicy {
    /// Validates one deterministic planning and Cartesian policy.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        planner: PlannerKind,
        planner_config: PlannerConfig,
        pose_solver: IkSolverKind,
        pose_config: IkSolveConfig,
        cartesian_config: CartesianConfig,
        maximum_cartesian_waypoints: usize,
    ) -> Result<Self, ArmAlgorithmError> {
        if !(2..=MAX_TRAJECTORY_POINTS).contains(&maximum_cartesian_waypoints) {
            return Err(ArmAlgorithmError::Capacity {
                field: "arm_task.cartesian_workspace",
                actual: maximum_cartesian_waypoints,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        Ok(Self {
            planner,
            planner_config,
            pose_solver,
            pose_config,
            cartesian_config,
            maximum_cartesian_waypoints,
        })
    }
}

/// Bounded result of compiling one task stage. It contains no lease, deadline
/// or hardware command and remains collision unchecked.
#[derive(Clone, Debug, PartialEq)]
pub enum ArmTaskProposal {
    /// Joint-space plan for a MoveJoint or MovePose stage.
    Motion(ArmPlan<CollisionUnchecked>),
    /// Bounded Cartesian segments for a Cartesian stage.
    Cartesian(Vec<CartesianPlan<CollisionUnchecked>>),
    /// Pick approach plan plus the original collision-unchecked grasp.
    Pick {
        /// Object identity selected by the stage.
        object_id: GraspObjectId,
        /// Candidate retained as collision-unchecked evidence.
        candidate: Box<GraspCandidate<CollisionUnchecked>>,
        /// Bounded approach plan.
        approach: ArmPlan<CollisionUnchecked>,
    },
    /// Place approach plan and object identity.
    Place {
        /// Object identity selected by the stage.
        object_id: GraspObjectId,
        /// Bounded approach plan.
        approach: ArmPlan<CollisionUnchecked>,
    },
    /// Semantic gripper intent with no device command bytes.
    Gripper(GripperCommand),
    /// Semantic hold duration with no authority effect.
    Hold {
        /// Validated positive duration.
        duration_ns: u64,
    },
}

/// One compiler result correlated to the request that produced it.
#[derive(Clone, Debug, PartialEq)]
pub struct CompiledTaskStage {
    request: TaskCompileRequest,
    proposal: ArmTaskProposal,
}

impl CompiledTaskStage {
    /// Returns the request that this result can satisfy.
    #[must_use]
    pub const fn request(&self) -> &TaskCompileRequest {
        &self.request
    }

    /// Returns the collision-unchecked semantic proposal.
    #[must_use]
    pub const fn proposal(&self) -> &ArmTaskProposal {
        &self.proposal
    }
}

/// Prepared compiler for the task program's active stages.
pub struct ArmTaskCompiler<'program, 'planning, 'prepared, 'model> {
    program: &'program ArmTaskProgram,
    planning: &'planning PreparedPlanning<'prepared, 'model>,
    context: TaskCompileContext,
    policy: TaskCompilePolicy,
}

impl<'program, 'planning, 'prepared, 'model>
    ArmTaskCompiler<'program, 'planning, 'prepared, 'model>
{
    /// Binds one task program and deterministic compile policy to prepared state.
    pub fn new(
        program: &'program ArmTaskProgram,
        planning: &'planning PreparedPlanning<'prepared, 'model>,
        policy: TaskCompilePolicy,
    ) -> Result<Self, ArmAlgorithmError> {
        let context = TaskCompileContext::new(program, planning)?;
        Ok(Self {
            program,
            planning,
            context,
            policy,
        })
    }

    /// Returns the context required by task cursor requests.
    #[must_use]
    pub const fn context(&self) -> &TaskCompileContext {
        &self.context
    }

    /// Allocates all reusable planning, IK and Cartesian scratch once.
    pub fn workspace(&self) -> Result<TaskCompileWorkspace, ArmAlgorithmError> {
        TaskCompileWorkspace::new(self.planning, &self.context, self.policy)
    }

    /// Compiles one request into a bounded collision-unchecked proposal.
    pub fn compile(
        &self,
        request: TaskCompileRequest,
        start: &ArmJointVector<ArmPosition>,
        validator: &impl ArmStateValidator,
        workspace: &mut TaskCompileWorkspace,
    ) -> Result<CompiledTaskStage, ArmAlgorithmError> {
        if request.context != self.context {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "task request context differs from compiler",
            });
        }
        self.context.require_planning(self.planning)?;
        workspace.require(self.planning, &self.context)?;
        let stage = self.program.stages.get(request.stage_index).ok_or(
            ArmAlgorithmError::StaleTaskResult {
                reason: "task request stage is outside the program",
            },
        )?;
        let proposal = match stage {
            ArmTaskStage::MoveJoint(target) => ArmTaskProposal::Motion(self.planning.plan(
                self.policy.planner,
                start,
                target,
                self.policy.planner_config,
                validator,
                &mut workspace.planner_workspace,
            )?),
            ArmTaskStage::MovePose(target) => {
                ArmTaskProposal::Motion(self.plan_pose(start, *target, validator, workspace)?)
            }
            ArmTaskStage::Cartesian(poses) => {
                ArmTaskProposal::Cartesian(self.plan_cartesian(start, poses, validator, workspace)?)
            }
            ArmTaskStage::Pick {
                object_id,
                candidate,
            } => ArmTaskProposal::Pick {
                object_id: *object_id,
                candidate: candidate.clone(),
                approach: self.plan_pose(start, candidate.pose(), validator, workspace)?,
            },
            ArmTaskStage::Place { object_id, pose } => ArmTaskProposal::Place {
                object_id: *object_id,
                approach: self.plan_pose(start, *pose, validator, workspace)?,
            },
            ArmTaskStage::Gripper(command) => ArmTaskProposal::Gripper(command.validate()?),
            ArmTaskStage::Hold { duration_ns } => ArmTaskProposal::Hold {
                duration_ns: *duration_ns,
            },
            ArmTaskStage::Recovery(_) => {
                return Err(ArmAlgorithmError::UnsupportedCapability {
                    capability: "task_recovery_branch_dispatch",
                });
            }
        };
        Ok(CompiledTaskStage { request, proposal })
    }

    fn plan_pose(
        &self,
        start: &ArmJointVector<ArmPosition>,
        target: ArmPose,
        validator: &impl ArmStateValidator,
        workspace: &mut TaskCompileWorkspace,
    ) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
        let solution = self.planning.kinematics.solve(
            self.policy.pose_solver,
            target,
            start,
            self.policy.pose_config,
            &mut workspace.ik_workspace,
        )?;
        self.planning.plan(
            self.policy.planner,
            start,
            solution.positions(),
            self.policy.planner_config,
            validator,
            &mut workspace.planner_workspace,
        )
    }

    fn plan_cartesian(
        &self,
        start: &ArmJointVector<ArmPosition>,
        poses: &[ArmPose],
        validator: &impl ArmStateValidator,
        workspace: &mut TaskCompileWorkspace,
    ) -> Result<Vec<CartesianPlan<CollisionUnchecked>>, ArmAlgorithmError> {
        let mut current = start.clone();
        let mut plans = Vec::with_capacity(poses.len());
        for target in poses {
            let plan = workspace.cartesian_workspace.plan(
                self.planning,
                &current,
                *target,
                self.policy.cartesian_config,
                validator,
            )?;
            if !plan.is_complete() {
                return Err(ArmAlgorithmError::ConstraintViolation {
                    sample_index: plan.achieved_steps(),
                    reason: "task_cartesian_stage_incomplete",
                });
            }
            current = plan
                .waypoints()
                .last()
                .cloned()
                .ok_or(ArmAlgorithmError::InvalidModel {
                    reason: "complete Cartesian task stage has no endpoint",
                })?;
            plans.push(plan);
        }
        Ok(plans)
    }
}

/// Preparation-time storage for a single task compiler.
pub struct TaskCompileWorkspace {
    context: TaskCompileContext,
    planner_workspace: PlannerWorkspace,
    ik_workspace: IkWorkspace,
    cartesian_workspace: PreparedCartesianPlanner,
}

impl TaskCompileWorkspace {
    fn new(
        planning: &PreparedPlanning<'_, '_>,
        context: &TaskCompileContext,
        policy: TaskCompilePolicy,
    ) -> Result<Self, ArmAlgorithmError> {
        context.require_planning(planning)?;
        Ok(Self {
            context: context.clone(),
            planner_workspace: planning.workspace(policy.planner_config)?,
            ik_workspace: planning
                .kinematics
                .workspace(policy.pose_config.population_size())?,
            cartesian_workspace: PreparedCartesianPlanner::new(
                planning,
                policy.maximum_cartesian_waypoints,
                policy.cartesian_config.maximum_ik_population(),
            )?,
        })
    }

    fn require(
        &self,
        planning: &PreparedPlanning<'_, '_>,
        context: &TaskCompileContext,
    ) -> Result<(), ArmAlgorithmError> {
        if &self.context != context {
            return Err(ArmAlgorithmError::StaleTaskResult {
                reason: "task compiler workspace context changed",
            });
        }
        self.context.require_planning(planning)
    }

    /// Returns all reusable storage reserved during compiler preparation.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.planner_workspace.allocated_bytes()
            + self.ik_workspace.allocated_bytes()
            + self.cartesian_workspace.allocated_bytes()
    }
}

fn validate_stages(
    chain: KinematicChain<'_>,
    stages: &[ArmTaskStage],
    field: &'static str,
) -> Result<(), ArmAlgorithmError> {
    if stages.is_empty() || stages.len() > MAX_ARM_TASK_STAGES {
        return Err(ArmAlgorithmError::Capacity {
            field,
            actual: stages.len(),
            maximum: MAX_ARM_TASK_STAGES,
        });
    }
    for stage in stages {
        match stage {
            ArmTaskStage::MoveJoint(target) => {
                if target.order() != chain.group().joint_order() {
                    return Err(ArmAlgorithmError::JointOrderMismatch {
                        group: chain.group().id(),
                    });
                }
                chain.forward(target)?;
            }
            ArmTaskStage::MovePose(_) | ArmTaskStage::Place { .. } => {}
            ArmTaskStage::Cartesian(poses) => {
                if poses.is_empty() || poses.len() > MAX_CARTESIAN_STAGE_POSES {
                    return Err(ArmAlgorithmError::Capacity {
                        field: "arm_task.cartesian",
                        actual: poses.len(),
                        maximum: MAX_CARTESIAN_STAGE_POSES,
                    });
                }
            }
            ArmTaskStage::Gripper(command) => {
                command.validate()?;
            }
            ArmTaskStage::Pick {
                object_id,
                candidate,
            } => {
                if *object_id != candidate.object_id() {
                    return Err(ArmAlgorithmError::InvalidModel {
                        reason: "pick object and grasp object differ",
                    });
                }
            }
            ArmTaskStage::Hold { duration_ns } => {
                if *duration_ns == 0 {
                    return Err(ArmAlgorithmError::InvalidTime {
                        field: "arm_task.hold",
                    });
                }
            }
            ArmTaskStage::Recovery(recovery) => {
                validate_stages(chain, recovery, "arm_task.recovery")?;
            }
        }
    }
    Ok(())
}
