//! Backend-neutral action payload and lifecycle contracts.

use arrayvec::ArrayVec;

use crate::{
    AsciiId, CommandEnvelope, Header, Id128, SemanticError, Twist2, numeric::require_finite,
};

/// Maximum fully-qualified action or Protobuf type identifier length.
pub const MAX_TYPE_NAME_LEN: usize = 160;
/// Maximum action payload accepted before type-specific decoding.
pub const MAX_ACTION_PAYLOAD_BYTES: usize = 65_536;
/// Maximum points in the first concrete trajectory action.
pub const MAX_TRAJECTORY_POINTS: usize = 256;

/// Fully-qualified registered Protobuf type name.
pub type ProtobufTypeName = AsciiId<MAX_TYPE_NAME_LEN>;
/// Versioned action identifier.
pub type ActionName = AsciiId<MAX_TYPE_NAME_LEN>;

/// Validated chassis velocity command.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisSetpoint {
    /// Local authority and monotonic deadline.
    pub envelope: CommandEnvelope,
    /// Finite body-frame target.
    pub target: Twist2,
}

/// Cross-host authority assertion that is not yet an executable command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandIntent {
    header: Header,
    resource_id: crate::ResourceId,
    owner_id: crate::OwnerId,
    lease_id: Id128,
    session_epoch: u64,
    command_id: Id128,
    requested_ttl_ns: u64,
}

/// Fields supplied to [`CommandIntent::new`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandIntentFields {
    /// Common source, boot, sequence and validity metadata.
    pub header: Header,
    /// Resource asserted by the remote command source.
    pub resource_id: crate::ResourceId,
    /// Owner asserted by the remote command source.
    pub owner_id: crate::OwnerId,
    /// Lease identity asserted by the remote command source.
    pub lease_id: Id128,
    /// Authority-service session asserted by the remote source.
    pub session_epoch: u64,
    /// Command identity used for correlation/idempotency.
    pub command_id: Id128,
    /// Requested lifetime after local receipt.
    pub requested_ttl_ns: u64,
}

impl CommandIntent {
    /// Constructs a structurally valid network command intent.
    pub fn new(fields: CommandIntentFields) -> Result<Self, SemanticError> {
        if fields.session_epoch == 0 {
            return Err(SemanticError::ZeroSessionEpoch);
        }
        if fields.requested_ttl_ns == 0 {
            return Err(SemanticError::InvalidDuration {
                field: "command_intent.requested_ttl_ns",
                maximum_ns: u64::MAX,
            });
        }
        Ok(Self {
            header: fields.header,
            resource_id: fields.resource_id,
            owner_id: fields.owner_id,
            lease_id: fields.lease_id,
            session_epoch: fields.session_epoch,
            command_id: fields.command_id,
            requested_ttl_ns: fields.requested_ttl_ns,
        })
    }

    /// Returns common source metadata.
    #[must_use]
    pub const fn header(self) -> Header {
        self.header
    }

    /// Returns the asserted resource.
    #[must_use]
    pub const fn resource_id(self) -> crate::ResourceId {
        self.resource_id
    }

    /// Returns the asserted owner.
    #[must_use]
    pub const fn owner_id(self) -> crate::OwnerId {
        self.owner_id
    }

    /// Returns the asserted lease identity.
    #[must_use]
    pub const fn lease_id(self) -> Id128 {
        self.lease_id
    }

    /// Returns the asserted authority session.
    #[must_use]
    pub const fn session_epoch(self) -> u64 {
        self.session_epoch
    }

    /// Returns the command correlation identity.
    #[must_use]
    pub const fn command_id(self) -> Id128 {
        self.command_id
    }

    /// Returns the requested lifetime after local receipt.
    #[must_use]
    pub const fn requested_ttl_ns(self) -> u64 {
        self.requested_ttl_ns
    }

    /// Matches local authority and creates an executable local envelope.
    pub fn materialize(
        self,
        active_lease: crate::ResourceLease,
        received_monotonic_ns: u64,
        maximum_ttl_ns: u64,
    ) -> Result<CommandEnvelope, SemanticError> {
        if self.resource_id != active_lease.resource_id() {
            return Err(SemanticError::AuthorityMismatch {
                field: "resource_id",
            });
        }
        if self.owner_id != active_lease.owner_id() {
            return Err(SemanticError::AuthorityMismatch { field: "owner_id" });
        }
        if self.lease_id != active_lease.lease_id() {
            return Err(SemanticError::AuthorityMismatch { field: "lease_id" });
        }
        if self.session_epoch != active_lease.session_epoch() {
            return Err(SemanticError::AuthorityMismatch {
                field: "session_epoch",
            });
        }
        CommandEnvelope::from_bridge_ingress(
            self.header,
            active_lease,
            self.command_id,
            received_monotonic_ns,
            self.requested_ttl_ns,
            maximum_ttl_ns,
        )
    }
}

/// Cross-host chassis command awaiting local authority materialization.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisSetpointIntent {
    intent: CommandIntent,
    target: Twist2,
}

impl ChassisSetpointIntent {
    /// Constructs a typed chassis intent.
    #[must_use]
    pub const fn new(intent: CommandIntent, target: Twist2) -> Self {
        Self { intent, target }
    }

    /// Returns the authority assertion.
    #[must_use]
    pub const fn intent(self) -> CommandIntent {
        self.intent
    }

    /// Returns the requested body velocity.
    #[must_use]
    pub const fn target(self) -> Twist2 {
        self.target
    }

    /// Creates a locally enforceable setpoint after authority checks.
    pub fn materialize(
        self,
        active_lease: crate::ResourceLease,
        received_monotonic_ns: u64,
        maximum_ttl_ns: u64,
    ) -> Result<ChassisSetpoint, SemanticError> {
        Ok(ChassisSetpoint {
            envelope: self.intent.materialize(
                active_lease,
                received_monotonic_ns,
                maximum_ttl_ns,
            )?,
            target: self.target,
        })
    }
}

/// Cross-host action goal awaiting local authority materialization.
#[derive(Clone, Debug, PartialEq)]
pub struct ActionGoalIntent {
    intent: CommandIntent,
    action_name: ActionName,
    goal: RegisteredPayload,
}

impl ActionGoalIntent {
    /// Constructs a typed action intent.
    #[must_use]
    pub const fn new(
        intent: CommandIntent,
        action_name: ActionName,
        goal: RegisteredPayload,
    ) -> Self {
        Self {
            intent,
            action_name,
            goal,
        }
    }

    /// Returns the authority assertion.
    #[must_use]
    pub const fn intent(&self) -> CommandIntent {
        self.intent
    }

    /// Returns the versioned action name.
    #[must_use]
    pub const fn action_name(&self) -> ActionName {
        self.action_name
    }

    /// Returns the registered goal payload.
    #[must_use]
    pub const fn goal(&self) -> &RegisteredPayload {
        &self.goal
    }

    /// Creates a locally enforceable action goal after authority checks.
    pub fn materialize(
        self,
        active_lease: crate::ResourceLease,
        received_monotonic_ns: u64,
        maximum_ttl_ns: u64,
    ) -> Result<ActionGoal, SemanticError> {
        Ok(ActionGoal {
            envelope: self.intent.materialize(
                active_lease,
                received_monotonic_ns,
                maximum_ttl_ns,
            )?,
            action_name: self.action_name,
            goal: self.goal,
        })
    }
}

/// Registered, bounded serialized Protobuf payload.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RegisteredPayload {
    protobuf_type: ProtobufTypeName,
    value: Vec<u8>,
}

impl RegisteredPayload {
    /// Constructs a nonempty type identity with a bounded payload.
    pub fn new(protobuf_type: ProtobufTypeName, value: Vec<u8>) -> Result<Self, SemanticError> {
        if value.len() > MAX_ACTION_PAYLOAD_BYTES {
            return Err(SemanticError::InvalidBound {
                field: "action.payload",
                maximum: MAX_ACTION_PAYLOAD_BYTES,
            });
        }
        Ok(Self {
            protobuf_type,
            value,
        })
    }

    /// Returns the registered type identity.
    #[must_use]
    pub const fn protobuf_type(&self) -> ProtobufTypeName {
        self.protobuf_type
    }

    /// Returns serialized payload bytes.
    #[must_use]
    pub fn value(&self) -> &[u8] {
        &self.value
    }
}

/// Canonical action state exposed on the wire.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionStatus {
    /// Goal accepted but execution has not started.
    Accepted,
    /// Goal is executing.
    Executing,
    /// Goal is canceling after an accepted request.
    Canceling,
    /// Goal succeeded.
    Succeeded,
    /// Goal failed.
    Failed,
    /// Goal completed cancellation.
    Cancelled,
    /// Goal timed out.
    TimedOut,
    /// Goal was rejected before execution.
    Rejected,
}

impl ActionStatus {
    /// Returns whether this state is terminal.
    #[must_use]
    pub const fn is_terminal(self) -> bool {
        matches!(
            self,
            Self::Succeeded | Self::Failed | Self::Cancelled | Self::TimedOut | Self::Rejected
        )
    }
}

/// Typed action goal under a local command envelope.
#[derive(Clone, Debug, PartialEq)]
pub struct ActionGoal {
    /// Authority and UUID correlation metadata.
    pub envelope: CommandEnvelope,
    /// Versioned action identifier.
    pub action_name: ActionName,
    /// Registered goal payload.
    pub goal: RegisteredPayload,
}

/// Bounded action feedback.
#[derive(Clone, Debug, PartialEq)]
pub struct ActionFeedback {
    /// Response metadata.
    pub header: Header,
    /// Canonical command ID/ROS UUID.
    pub command_id: Id128,
    /// Executing or canceling status.
    pub status: ActionStatus,
    /// Registered feedback payload.
    pub feedback: RegisteredPayload,
}

impl ActionFeedback {
    /// Rejects feedback with a terminal or pre-execution status.
    pub fn validate(self) -> Result<Self, SemanticError> {
        if !matches!(
            self.status,
            ActionStatus::Executing | ActionStatus::Canceling
        ) {
            return Err(SemanticError::InvalidActionStatus {
                context: "feedback",
            });
        }
        Ok(self)
    }
}

/// Cancellation request correlated by command ID.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActionCancelRequest {
    /// Request metadata.
    pub header: Header,
    /// Canonical command ID/ROS UUID.
    pub command_id: Id128,
}

/// Cancellation decision.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionCancelDecision {
    /// Cancellation will proceed; terminal cancellation is still pending.
    Accepted,
    /// Cancellation was rejected and execution may continue.
    Rejected,
}

/// Cancellation response correlated by command ID.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionCancelResponse {
    /// Response metadata.
    pub header: Header,
    /// Canonical command ID/ROS UUID.
    pub command_id: Id128,
    /// Cancellation decision.
    pub decision: ActionCancelDecision,
    /// Stable machine-readable decision code.
    pub result_code: u32,
    /// Human-readable detail not consumed by safety logic.
    pub detail: String,
}

/// Terminal action result.
#[derive(Clone, Debug, PartialEq)]
pub struct ActionResult {
    /// Response metadata.
    pub header: Header,
    /// Canonical command ID/ROS UUID.
    pub command_id: Id128,
    /// Terminal status.
    pub status: ActionStatus,
    /// Stable machine-readable result code.
    pub result_code: u32,
    /// Human-readable detail not consumed by safety logic.
    pub detail: String,
    /// Registered result payload.
    pub result: RegisteredPayload,
}

impl ActionResult {
    /// Rejects nonterminal statuses in a terminal result message.
    pub fn validate(self) -> Result<Self, SemanticError> {
        if !self.status.is_terminal() {
            return Err(SemanticError::InvalidActionStatus { context: "result" });
        }
        Ok(self)
    }
}

/// Pure lifecycle state for action protocol tests and adapter implementations.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionLifecycle {
    /// Goal was submitted and awaits a decision.
    Submitted,
    /// Goal was accepted and awaits execution.
    Accepted,
    /// Goal is executing.
    Executing,
    /// A cancellation request awaits a decision.
    CancelRequested,
    /// Cancellation was accepted and is in progress.
    Canceling,
    /// Terminal result was produced.
    Terminal(ActionStatus),
}

/// Event consumed by the exhaustive action lifecycle reducer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionEvent {
    /// Goal acceptance.
    Accept,
    /// Goal rejection.
    Reject,
    /// Execution start.
    Start,
    /// Nonterminal feedback.
    Feedback,
    /// Cancellation request.
    RequestCancel,
    /// Cancellation acceptance.
    AcceptCancel,
    /// Cancellation rejection.
    RejectCancel,
    /// Terminal completion.
    Complete(ActionStatus),
}

impl ActionLifecycle {
    /// Applies one lifecycle event and rejects invalid ordering.
    pub fn apply(self, event: ActionEvent) -> Result<Self, SemanticError> {
        let next = match (self, event) {
            (Self::Submitted, ActionEvent::Accept) => Self::Accepted,
            (Self::Submitted, ActionEvent::Reject) => Self::Terminal(ActionStatus::Rejected),
            (Self::Accepted, ActionEvent::Start) => Self::Executing,
            (Self::Accepted | Self::Executing, ActionEvent::RequestCancel) => Self::CancelRequested,
            (Self::Executing, ActionEvent::Feedback) => Self::Executing,
            (Self::CancelRequested, ActionEvent::AcceptCancel) => Self::Canceling,
            (Self::CancelRequested, ActionEvent::RejectCancel) => Self::Executing,
            (Self::Canceling, ActionEvent::Feedback) => Self::Canceling,
            (Self::Accepted | Self::Executing | Self::Canceling, ActionEvent::Complete(status))
                if status.is_terminal() =>
            {
                Self::Terminal(status)
            }
            _ => return Err(SemanticError::InvalidActionTransition),
        };
        Ok(next)
    }
}

/// Bounded terminal-result retention policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActionResultCachePolicy {
    /// Maximum retained results.
    pub depth: usize,
    /// Retention duration in nanoseconds.
    pub retention_ns: u64,
}

impl ActionResultCachePolicy {
    /// Requires finite nonzero storage and time bounds.
    pub fn validate(self, maximum_depth: usize) -> Result<Self, SemanticError> {
        if self.depth == 0 || self.depth > maximum_depth {
            return Err(SemanticError::InvalidBound {
                field: "action.result_cache.depth",
                maximum: maximum_depth,
            });
        }
        if self.retention_ns == 0 {
            return Err(SemanticError::ZeroDeadline {
                field: "action.result_cache.retention_ns",
            });
        }
        Ok(self)
    }
}

/// One point in the concrete chassis trajectory action.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChassisTrajectoryPoint {
    /// Strictly increasing offset from acceptance.
    pub time_from_start_ns: u64,
    /// Finite body-frame velocity target.
    pub target: Twist2,
}

/// Concrete ExecuteChassisTrajectory goal payload.
#[derive(Clone, Debug, PartialEq)]
pub struct ExecuteChassisTrajectoryGoal {
    /// Nonempty bounded and strictly time-ordered points.
    pub points: ArrayVec<ChassisTrajectoryPoint, MAX_TRAJECTORY_POINTS>,
}

impl ExecuteChassisTrajectoryGoal {
    /// Validates the point sequence.
    pub fn validate(self) -> Result<Self, SemanticError> {
        if self.points.is_empty() {
            return Err(SemanticError::EmptyCollection {
                field: "trajectory.points",
            });
        }
        let mut previous = 0;
        for point in &self.points {
            if point.time_from_start_ns == 0 || point.time_from_start_ns <= previous {
                return Err(SemanticError::InvalidTimeOrder {
                    field: "trajectory.points",
                });
            }
            previous = point.time_from_start_ns;
        }
        Ok(self)
    }
}

/// Concrete ExecuteChassisTrajectory feedback payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ExecuteChassisTrajectoryFeedback {
    /// Zero-based active point index.
    pub active_point: u32,
    /// Progress in the closed interval [0, 1].
    pub progress: f64,
}

impl ExecuteChassisTrajectoryFeedback {
    /// Validates finite bounded progress.
    pub fn validate(self) -> Result<Self, SemanticError> {
        require_finite(self.progress, "trajectory.feedback.progress")?;
        if !(0.0..=1.0).contains(&self.progress) {
            return Err(SemanticError::OutsideClosedUnitInterval {
                field: "trajectory.feedback.progress",
            });
        }
        Ok(self)
    }
}

/// Concrete ExecuteChassisTrajectory terminal result payload.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExecuteChassisTrajectoryResult {
    /// Number of completed points.
    pub completed_points: u32,
}

#[cfg(test)]
mod tests {
    use super::{ActionEvent, ActionLifecycle, ActionResultCachePolicy, ActionStatus};

    #[test]
    fn action_lifecycle_distinguishes_cancel_acceptance_from_completion() {
        let state = ActionLifecycle::Submitted
            .apply(ActionEvent::Accept)
            .unwrap()
            .apply(ActionEvent::Start)
            .unwrap()
            .apply(ActionEvent::RequestCancel)
            .unwrap()
            .apply(ActionEvent::AcceptCancel)
            .unwrap();
        assert_eq!(state, ActionLifecycle::Canceling);
        assert_eq!(
            state
                .apply(ActionEvent::Complete(ActionStatus::Cancelled))
                .unwrap(),
            ActionLifecycle::Terminal(ActionStatus::Cancelled)
        );
    }

    #[test]
    fn action_lifecycle_rejects_terminal_feedback() {
        let terminal = ActionLifecycle::Terminal(ActionStatus::Succeeded);
        assert!(terminal.apply(ActionEvent::Feedback).is_err());
    }

    #[test]
    fn action_lifecycle_covers_goal_and_cancel_rejection() {
        assert_eq!(
            ActionLifecycle::Submitted
                .apply(ActionEvent::Reject)
                .unwrap(),
            ActionLifecycle::Terminal(ActionStatus::Rejected)
        );
        let executing = ActionLifecycle::Submitted
            .apply(ActionEvent::Accept)
            .unwrap()
            .apply(ActionEvent::Start)
            .unwrap();
        assert_eq!(
            executing
                .apply(ActionEvent::RequestCancel)
                .unwrap()
                .apply(ActionEvent::RejectCancel)
                .unwrap(),
            ActionLifecycle::Executing
        );
    }

    #[test]
    fn action_result_retention_is_bounded_and_nonzero() {
        assert!(
            ActionResultCachePolicy {
                depth: 32,
                retention_ns: 60_000_000_000,
            }
            .validate(32)
            .is_ok()
        );
        assert!(
            ActionResultCachePolicy {
                depth: 0,
                retention_ns: 1,
            }
            .validate(32)
            .is_err()
        );
        assert!(
            ActionResultCachePolicy {
                depth: 1,
                retention_ns: 0,
            }
            .validate(32)
            .is_err()
        );
    }
}
