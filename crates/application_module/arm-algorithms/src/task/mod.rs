//! Bounded arm task stages with explicit cancellation progression.

use message_model::{ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};

use crate::{
    ArmAlgorithmError, CollisionUnchecked,
    grasp::{GraspCandidate, GraspObjectId},
    kinematics::{ArmPose, KinematicChain},
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
}

impl<'program> ArmTaskCursor<'program> {
    /// Creates a pending cursor.
    #[must_use]
    pub const fn new(program: &'program ArmTaskProgram) -> Self {
        Self {
            program,
            state: ArmTaskState::Pending,
            next_stage: 0,
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
        self.state = ArmTaskState::Failed;
        Ok(())
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
