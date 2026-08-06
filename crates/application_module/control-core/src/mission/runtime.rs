//! Deterministic fixed-capacity mission event reducer.

use arrayvec::ArrayVec;
use message_model::{ActionCancelDecision, ActionStatus, Id128};

use crate::behavior_tree::{
    BehaviorTreeActivationSnapshot, BehaviorTreeActivationState, BehaviorTreeNodeParticipation,
    BehaviorTreeNodeSnapshot, BehaviorTreeNodeType, BehaviorTreeSnapshot,
};

use super::{
    CancelScopeId, GuardId, MAX_MISSION_ACTION_INVOCATIONS, MAX_MISSION_ACTIVE_ACTIVATIONS,
    MAX_MISSION_CHILDREN, MAX_MISSION_NODES, MissionAction, MissionIr, MissionNodeId,
    MissionNodeKind,
};

/// Maximum simultaneous activation frames.
pub const MAX_ACTIVE_ACTIVATIONS: usize = MAX_MISSION_ACTIVE_ACTIVATIONS;
/// Maximum command identities retained for one run.
pub const MAX_ACTION_INVOCATIONS: usize = MAX_MISSION_ACTION_INVOCATIONS;
/// Maximum effects emitted by one serialized event.
pub const MAX_MISSION_EFFECTS: usize = MAX_ACTIVE_ACTIVATIONS * 2 + 1;

/// Generation-tagged activation identity.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct MissionActivationId {
    slot: u8,
    generation: u32,
}

impl MissionActivationId {
    /// Returns the stable slot used within one runtime.
    #[must_use]
    pub const fn slot(self) -> u8 {
        self.slot
    }

    /// Returns the reuse generation used to reject stale events.
    #[must_use]
    pub const fn generation(self) -> u32 {
        self.generation
    }
}

/// Terminal node/mission outcome.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionOutcome {
    /// Required work completed.
    Succeeded,
    /// Work failed.
    Failed,
    /// A bounded deadline elapsed.
    TimedOut,
    /// Cancellation completed.
    Cancelled,
    /// Action dispatch was rejected before execution.
    Rejected,
}

/// Why action cancellation was requested.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionCancelReason {
    /// Named cancellation scope was selected.
    Scope,
    /// Whole mission cancellation was selected.
    Mission,
    /// Action or wrapper deadline elapsed.
    Timeout,
    /// A parallel sibling determined the composite outcome.
    ParallelSibling,
}

/// Serialized mission runtime event.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionEvent {
    /// Start the immutable graph exactly once.
    Start,
    /// Bind an externally generated command ID to a dispatch request.
    ActionDispatched {
        /// Exact pending action activation.
        activation: MissionActivationId,
        /// Canonical action command identity.
        command_id: Id128,
    },
    /// Goal acceptance/rejection correlated by command identity.
    ActionAcknowledged {
        /// Exact action command identity.
        command_id: Id128,
        /// Whether the action server accepted the goal.
        accepted: bool,
    },
    /// Cancellation acceptance is nonterminal and correlated by command ID.
    CancelDecision {
        /// Exact action command identity.
        command_id: Id128,
        /// Remote cancellation decision.
        decision: ActionCancelDecision,
    },
    /// Terminal action evidence correlated by command ID.
    ActionTerminal {
        /// Exact action command identity.
        command_id: Id128,
        /// Terminal wire-facing status.
        status: ActionStatus,
    },
    /// Resolve one pending guard activation.
    GuardResolved {
        /// Exact guard activation.
        activation: MissionActivationId,
        /// Selected condition value.
        value: bool,
    },
    /// Deliver a deadline for an exact action or Timeout activation.
    DeadlineElapsed {
        /// Exact activation whose declared deadline elapsed.
        activation: MissionActivationId,
    },
    /// Cancel every active instance below one named scope.
    CancelScope {
        /// Validated unique scope identity.
        scope_id: CancelScopeId,
    },
    /// Cancel the complete active mission.
    CancelAll,
}

/// Typed rejection that leaves runtime state unchanged.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionEventRejection {
    /// Start was delivered after the runtime left Idle.
    AlreadyStarted,
    /// A non-Start event was delivered while Idle or after completion.
    NotRunning,
    /// Activation slot/generation is no longer active.
    StaleActivation,
    /// Event is invalid for the activation's current phase.
    InvalidActivationPhase,
    /// No active action owns this command identity.
    UnknownCommand,
    /// A command identity was already used in this run.
    DuplicateCommand,
    /// Action status was not terminal.
    NonterminalActionStatus,
    /// No active instance of the named cancellation scope exists.
    InactiveCancelScope,
    /// Deadline was delivered for a node without a runtime deadline.
    ActivationHasNoDeadline,
}

/// Internal fixed-capacity failure that terminates the mission fail closed.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionRuntimeFault {
    /// More simultaneous graph activations were required than allowed.
    ActivationCapacity,
    /// More action commands were dispatched than the bounded history permits.
    CommandHistoryCapacity,
    /// A slot generation was exhausted.
    ActivationGenerationExhausted,
    /// Validated graph/runtime bookkeeping disagreed.
    InvariantViolation,
}

/// Ordered reducer effect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionEffect<'ir> {
    /// Adapter must allocate/send a command and return `ActionDispatched`.
    DispatchAction {
        /// Correlation identity for the pending dispatch.
        activation: MissionActivationId,
        /// Stable node identity.
        node_id: MissionNodeId,
        /// Immutable action definition.
        action: &'ir MissionAction,
    },
    /// Scheduler must return a correlated `DeadlineElapsed`.
    ArmDeadline {
        /// Deadline-bearing action or Timeout-wrapper activation.
        activation: MissionActivationId,
        /// Relative finite deadline.
        timeout_ns: u64,
    },
    /// Guard adapter must return a correlated boolean result.
    EvaluateGuard {
        /// Guard activation.
        activation: MissionActivationId,
        /// Registered guard identity.
        guard_id: GuardId,
    },
    /// Request cancellation; terminal completion is still pending.
    RequestActionCancel {
        /// Exact command identity.
        command_id: Id128,
        /// Why cancellation was requested.
        reason: MissionCancelReason,
    },
    /// Cancellation was rejected; the action remains active.
    CancellationRejected {
        /// Exact command identity.
        command_id: Id128,
    },
    /// Correlated terminal action evidence permits resource release.
    ReleaseActionResources {
        /// Action activation completing.
        activation: MissionActivationId,
        /// Stable action node.
        node_id: MissionNodeId,
        /// Exact command identity.
        command_id: Id128,
        /// Effective terminal outcome.
        outcome: MissionOutcome,
    },
    /// Root activation completed.
    MissionCompleted(MissionOutcome),
    /// Event was rejected without advancing runtime state.
    EventRejected(MissionEventRejection),
    /// Fixed-capacity runtime failed closed.
    RuntimeFault(MissionRuntimeFault),
}

/// Fixed-capacity effect batch from one event.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionStep<'ir> {
    effects: ArrayVec<MissionEffect<'ir>, MAX_MISSION_EFFECTS>,
}

impl<'ir> MissionStep<'ir> {
    fn new() -> Self {
        Self {
            effects: ArrayVec::new(),
        }
    }

    fn push(&mut self, effect: MissionEffect<'ir>) {
        self.effects
            .try_push(effect)
            .expect("effect bound follows active activation bound");
    }

    /// Returns effects in deterministic execution order.
    #[must_use]
    pub fn effects(&self) -> &[MissionEffect<'ir>] {
        self.effects.as_slice()
    }
}

/// Public runtime state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MissionRuntimeState {
    /// Graph has not started.
    Idle,
    /// At least one activation is live.
    Running,
    /// Root produced a terminal outcome.
    Terminal(MissionOutcome),
    /// Fixed-capacity runtime failed closed.
    Faulted(MissionRuntimeFault),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ActivationPhase {
    Entering,
    WaitingDispatch,
    WaitingAcceptance(Id128),
    RunningAction(Id128),
    CancelRequested(Id128),
    Canceling(Id128),
    WaitingGuard,
    WaitingChildren,
}

impl ActivationPhase {
    const fn command_id(self) -> Option<Id128> {
        match self {
            Self::WaitingAcceptance(command_id)
            | Self::RunningAction(command_id)
            | Self::CancelRequested(command_id)
            | Self::Canceling(command_id) => Some(command_id),
            Self::Entering | Self::WaitingDispatch | Self::WaitingGuard | Self::WaitingChildren => {
                None
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct Activation {
    node_id: MissionNodeId,
    parent: Option<MissionActivationId>,
    phase: ActivationPhase,
    children: [Option<MissionActivationId>; MAX_MISSION_CHILDREN],
    live_children: u8,
    next_child: u8,
    repeat_count: u16,
    forced_outcome: Option<MissionOutcome>,
    cancel_reason: Option<MissionCancelReason>,
}

impl Activation {
    const fn new(node_id: MissionNodeId, parent: Option<MissionActivationId>) -> Self {
        Self {
            node_id,
            parent,
            phase: ActivationPhase::Entering,
            children: [None; MAX_MISSION_CHILDREN],
            live_children: 0,
            next_child: 0,
            repeat_count: 0,
            forced_outcome: None,
            cancel_reason: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ActivationSlot {
    generation: u32,
    activation: Option<Activation>,
}

const EMPTY_SLOT: ActivationSlot = ActivationSlot {
    generation: 0,
    activation: None,
};

/// Single-thread deterministic mission reducer.
pub struct MissionRuntime<'ir> {
    ir: &'ir MissionIr,
    state: MissionRuntimeState,
    slots: [ActivationSlot; MAX_ACTIVE_ACTIVATIONS],
    used_commands: ArrayVec<Id128, MAX_ACTION_INVOCATIONS>,
    last_outcomes: [Option<MissionOutcome>; MAX_MISSION_NODES],
    root_activation: Option<MissionActivationId>,
}

impl<'ir> MissionRuntime<'ir> {
    /// Creates an idle runtime over immutable validated IR.
    #[must_use]
    pub const fn new(ir: &'ir MissionIr) -> Self {
        Self {
            ir,
            state: MissionRuntimeState::Idle,
            slots: [EMPTY_SLOT; MAX_ACTIVE_ACTIVATIONS],
            used_commands: ArrayVec::new_const(),
            last_outcomes: [None; MAX_MISSION_NODES],
            root_activation: None,
        }
    }

    /// Returns the current runtime state.
    #[must_use]
    pub const fn state(&self) -> MissionRuntimeState {
        self.state
    }

    /// Applies one serialized event and returns ordered effects.
    pub fn dispatch(&mut self, event: MissionEvent) -> MissionStep<'ir> {
        self.advance(event)
    }

    /// Advances the behavior tree with one externally correlated event.
    ///
    /// This is an additive spelling for the public behavior-tree facade.
    /// It is equivalent to [`Self::dispatch`].
    pub fn advance(&mut self, event: MissionEvent) -> MissionStep<'ir> {
        let mut step = MissionStep::new();
        match (self.state, event) {
            (MissionRuntimeState::Idle, MissionEvent::Start) => {
                self.state = MissionRuntimeState::Running;
                match self.allocate_activation(self.ir.root(), None) {
                    Ok(root) => self.root_activation = Some(root),
                    Err(fault) => self.fail_runtime(fault, &mut step),
                }
            }
            (
                MissionRuntimeState::Running,
                MissionEvent::ActionDispatched {
                    activation,
                    command_id,
                },
            ) => self.action_dispatched(activation, command_id, &mut step),
            (
                MissionRuntimeState::Running,
                MissionEvent::ActionAcknowledged {
                    command_id,
                    accepted,
                },
            ) => self.action_acknowledged(command_id, accepted, &mut step),
            (
                MissionRuntimeState::Running,
                MissionEvent::CancelDecision {
                    command_id,
                    decision,
                },
            ) => self.cancel_decision(command_id, decision, &mut step),
            (MissionRuntimeState::Running, MissionEvent::ActionTerminal { command_id, status }) => {
                self.action_terminal(command_id, status, &mut step)
            }
            (MissionRuntimeState::Running, MissionEvent::GuardResolved { activation, value }) => {
                self.guard_resolved(activation, value, &mut step)
            }
            (MissionRuntimeState::Running, MissionEvent::DeadlineElapsed { activation }) => {
                self.deadline_elapsed(activation, &mut step)
            }
            (MissionRuntimeState::Running, MissionEvent::CancelScope { scope_id }) => {
                self.cancel_scope(scope_id, &mut step);
            }
            (MissionRuntimeState::Running, MissionEvent::CancelAll) => {
                if let Some(root) = self.root_activation {
                    self.request_cancel(
                        root,
                        MissionCancelReason::Mission,
                        MissionOutcome::Cancelled,
                        &mut step,
                    );
                }
            }
            (MissionRuntimeState::Running, MissionEvent::Start) => {
                step.push(MissionEffect::EventRejected(
                    MissionEventRejection::AlreadyStarted,
                ));
            }
            (MissionRuntimeState::Idle, _) => {
                step.push(MissionEffect::EventRejected(
                    MissionEventRejection::NotRunning,
                ));
            }
            (MissionRuntimeState::Terminal(_) | MissionRuntimeState::Faulted(_), _) => {
                step.push(MissionEffect::EventRejected(
                    MissionEventRejection::NotRunning,
                ));
            }
        }
        self.drive(&mut step);
        step
    }

    /// Captures the bounded live participation state without allocating.
    #[must_use]
    pub fn snapshot(&self) -> BehaviorTreeSnapshot {
        let mut activations = ArrayVec::new();
        for (slot_index, slot) in self.slots.iter().enumerate() {
            let Some(activation) = slot.activation else {
                continue;
            };
            let node = self
                .ir
                .node(activation.node_id)
                .expect("validated IR node remains available");
            activations
                .try_push(BehaviorTreeActivationSnapshot::new(
                    MissionActivationId {
                        slot: u8::try_from(slot_index).expect("activation capacity fits u8"),
                        generation: slot.generation,
                    },
                    activation.parent,
                    activation.node_id,
                    node.name(),
                    BehaviorTreeNodeType::from_mission(node.kind()),
                    snapshot_activation_state(activation.phase),
                    activation.phase.command_id(),
                ))
                .expect("live activation count is bounded by slot capacity");
        }

        let mut nodes = ArrayVec::new();
        for (node_index, node) in self.ir.nodes().iter().enumerate() {
            let active_count = activations
                .iter()
                .filter(|activation| activation.node_id() == node.id())
                .count();
            let participation = if active_count == 0 {
                match self.last_outcomes[node_index] {
                    Some(outcome) => BehaviorTreeNodeParticipation::Terminal(outcome),
                    None => BehaviorTreeNodeParticipation::Inactive,
                }
            } else {
                BehaviorTreeNodeParticipation::Active {
                    activation_count: u8::try_from(active_count)
                        .expect("active count fits fixed activation capacity"),
                }
            };
            nodes
                .try_push(BehaviorTreeNodeSnapshot::new(
                    node.id(),
                    node.name(),
                    BehaviorTreeNodeType::from_mission(node.kind()),
                    participation,
                    self.last_outcomes[node_index],
                ))
                .expect("validated node count is bounded by snapshot capacity");
        }

        BehaviorTreeSnapshot::new(self.state, nodes, activations)
    }

    fn allocate_activation(
        &mut self,
        node_id: MissionNodeId,
        parent: Option<MissionActivationId>,
    ) -> Result<MissionActivationId, MissionRuntimeFault> {
        let Some(slot_index) = self.slots.iter().position(|slot| slot.activation.is_none()) else {
            return Err(MissionRuntimeFault::ActivationCapacity);
        };
        let generation = self.slots[slot_index]
            .generation
            .checked_add(1)
            .ok_or(MissionRuntimeFault::ActivationGenerationExhausted)?;
        let id = MissionActivationId {
            slot: u8::try_from(slot_index).expect("activation capacity fits u8"),
            generation,
        };
        let node_index = self
            .ir
            .node_index(node_id)
            .expect("validated runtime only activates known nodes");
        self.last_outcomes[node_index] = None;
        self.slots[slot_index] = ActivationSlot {
            generation,
            activation: Some(Activation::new(node_id, parent)),
        };
        Ok(id)
    }

    fn spawn_child(
        &mut self,
        parent: MissionActivationId,
        node_id: MissionNodeId,
        step: &mut MissionStep<'ir>,
    ) -> Option<MissionActivationId> {
        let child = match self.allocate_activation(node_id, Some(parent)) {
            Ok(child) => child,
            Err(fault) => {
                self.fail_runtime(fault, step);
                return None;
            }
        };
        let mut activation = self
            .activation(parent)
            .expect("parent remains active while spawning");
        let slot = activation
            .children
            .iter()
            .position(Option::is_none)
            .expect("validated direct child bound");
        activation.children[slot] = Some(child);
        activation.live_children = activation.live_children.saturating_add(1);
        self.set_activation(parent, activation);
        Some(child)
    }

    fn drive(&mut self, step: &mut MissionStep<'ir>) {
        while self.state == MissionRuntimeState::Running {
            let next = self.slots.iter().enumerate().find_map(|(index, slot)| {
                let activation = slot.activation?;
                (activation.phase == ActivationPhase::Entering).then_some(MissionActivationId {
                    slot: u8::try_from(index).expect("activation capacity fits u8"),
                    generation: slot.generation,
                })
            });
            let Some(id) = next else {
                break;
            };
            self.enter_activation(id, step);
        }
    }

    fn enter_activation(&mut self, id: MissionActivationId, step: &mut MissionStep<'ir>) {
        let Some(mut activation) = self.activation(id) else {
            return;
        };
        let node_index = self
            .ir
            .node_index(activation.node_id)
            .expect("validated IR node");
        match self.ir.nodes()[node_index].kind() {
            MissionNodeKind::Action(action) => {
                activation.phase = ActivationPhase::WaitingDispatch;
                self.set_activation(id, activation);
                step.push(MissionEffect::DispatchAction {
                    activation: id,
                    node_id: activation.node_id,
                    action,
                });
                step.push(MissionEffect::ArmDeadline {
                    activation: id,
                    timeout_ns: action.timeout_ns(),
                });
            }
            MissionNodeKind::Sequence(children) | MissionNodeKind::Selector(children) => {
                let first = children.as_slice()[0];
                activation.phase = ActivationPhase::WaitingChildren;
                activation.next_child = 1;
                self.set_activation(id, activation);
                let _ = self.spawn_child(id, first, step);
            }
            MissionNodeKind::ParallelAll(children) | MissionNodeKind::ParallelAny(children) => {
                let children = children.clone();
                activation.phase = ActivationPhase::WaitingChildren;
                self.set_activation(id, activation);
                for child in children.as_slice() {
                    if self.spawn_child(id, *child, step).is_none() {
                        return;
                    }
                }
            }
            MissionNodeKind::Timeout { child, timeout_ns } => {
                let child = *child;
                let timeout_ns = *timeout_ns;
                activation.phase = ActivationPhase::WaitingChildren;
                self.set_activation(id, activation);
                step.push(MissionEffect::ArmDeadline {
                    activation: id,
                    timeout_ns,
                });
                let _ = self.spawn_child(id, child, step);
            }
            MissionNodeKind::Retry {
                child,
                max_attempts: _,
            }
            | MissionNodeKind::Loop {
                child,
                max_iterations: _,
            } => {
                let child = *child;
                activation.phase = ActivationPhase::WaitingChildren;
                activation.repeat_count = 1;
                self.set_activation(id, activation);
                let _ = self.spawn_child(id, child, step);
            }
            MissionNodeKind::Guard { guard_id, .. } => {
                let guard_id = *guard_id;
                activation.phase = ActivationPhase::WaitingGuard;
                self.set_activation(id, activation);
                step.push(MissionEffect::EvaluateGuard {
                    activation: id,
                    guard_id,
                });
            }
            MissionNodeKind::CancelScope { child, .. } | MissionNodeKind::Subflow { child } => {
                let child = *child;
                activation.phase = ActivationPhase::WaitingChildren;
                self.set_activation(id, activation);
                let _ = self.spawn_child(id, child, step);
            }
        }
    }

    fn action_dispatched(
        &mut self,
        id: MissionActivationId,
        command_id: Id128,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(mut activation) = self.activation(id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::StaleActivation,
            ));
            return;
        };
        if activation.phase != ActivationPhase::WaitingDispatch {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        }
        if self.used_commands.contains(&command_id) {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::DuplicateCommand,
            ));
            return;
        }
        if self.used_commands.try_push(command_id).is_err() {
            self.fail_runtime(MissionRuntimeFault::CommandHistoryCapacity, step);
            return;
        }
        activation.phase = ActivationPhase::WaitingAcceptance(command_id);
        self.set_activation(id, activation);
    }

    fn action_acknowledged(
        &mut self,
        command_id: Id128,
        accepted: bool,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(id) = self.find_command(command_id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::UnknownCommand,
            ));
            return;
        };
        let mut activation = self.activation(id).expect("command activation exists");
        if activation.phase != ActivationPhase::WaitingAcceptance(command_id) {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        }
        if accepted {
            if let Some(reason) = activation.cancel_reason {
                activation.phase = ActivationPhase::CancelRequested(command_id);
                self.set_activation(id, activation);
                step.push(MissionEffect::RequestActionCancel { command_id, reason });
            } else {
                activation.phase = ActivationPhase::RunningAction(command_id);
                self.set_activation(id, activation);
            }
        } else {
            let outcome = activation
                .forced_outcome
                .unwrap_or(MissionOutcome::Rejected);
            self.release_and_complete_action(id, command_id, outcome, step);
        }
    }

    fn cancel_decision(
        &mut self,
        command_id: Id128,
        decision: ActionCancelDecision,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(id) = self.find_command(command_id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::UnknownCommand,
            ));
            return;
        };
        let mut activation = self.activation(id).expect("command activation exists");
        if activation.phase != ActivationPhase::CancelRequested(command_id) {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        }
        match decision {
            ActionCancelDecision::Accepted => {
                activation.phase = ActivationPhase::Canceling(command_id);
                self.set_activation(id, activation);
            }
            ActionCancelDecision::Rejected => {
                activation.phase = ActivationPhase::RunningAction(command_id);
                if activation.forced_outcome == Some(MissionOutcome::Cancelled) {
                    activation.forced_outcome = Some(MissionOutcome::Failed);
                }
                activation.cancel_reason = None;
                self.set_activation(id, activation);
                self.mark_cancel_rejected_ancestors(activation.parent);
                step.push(MissionEffect::CancellationRejected { command_id });
            }
        }
    }

    fn action_terminal(
        &mut self,
        command_id: Id128,
        status: ActionStatus,
        step: &mut MissionStep<'ir>,
    ) {
        if !status.is_terminal() {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::NonterminalActionStatus,
            ));
            return;
        }
        let Some(id) = self.find_command(command_id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::UnknownCommand,
            ));
            return;
        };
        let activation = self.activation(id).expect("command activation exists");
        if !matches!(
            activation.phase,
            ActivationPhase::RunningAction(_)
                | ActivationPhase::CancelRequested(_)
                | ActivationPhase::Canceling(_)
        ) {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        }
        let observed = match status {
            ActionStatus::Succeeded => MissionOutcome::Succeeded,
            ActionStatus::Failed => MissionOutcome::Failed,
            ActionStatus::Cancelled => MissionOutcome::Cancelled,
            ActionStatus::TimedOut => MissionOutcome::TimedOut,
            ActionStatus::Rejected => MissionOutcome::Rejected,
            ActionStatus::Accepted | ActionStatus::Executing | ActionStatus::Canceling => {
                step.push(MissionEffect::EventRejected(
                    MissionEventRejection::NonterminalActionStatus,
                ));
                return;
            }
        };
        let outcome = activation.forced_outcome.unwrap_or(observed);
        self.release_and_complete_action(id, command_id, outcome, step);
    }

    fn release_and_complete_action(
        &mut self,
        id: MissionActivationId,
        command_id: Id128,
        outcome: MissionOutcome,
        step: &mut MissionStep<'ir>,
    ) {
        let node_id = self
            .activation(id)
            .expect("action activation exists")
            .node_id;
        step.push(MissionEffect::ReleaseActionResources {
            activation: id,
            node_id,
            command_id,
            outcome,
        });
        self.complete_activation(id, outcome, step);
    }

    fn guard_resolved(
        &mut self,
        id: MissionActivationId,
        value: bool,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(mut activation) = self.activation(id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::StaleActivation,
            ));
            return;
        };
        if activation.phase != ActivationPhase::WaitingGuard {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        }
        let node = self.ir.node(activation.node_id).expect("validated IR node");
        let MissionNodeKind::Guard {
            if_true, if_false, ..
        } = node.kind()
        else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InvalidActivationPhase,
            ));
            return;
        };
        let child = if value { *if_true } else { *if_false };
        activation.phase = ActivationPhase::WaitingChildren;
        self.set_activation(id, activation);
        let _ = self.spawn_child(id, child, step);
    }

    fn deadline_elapsed(&mut self, id: MissionActivationId, step: &mut MissionStep<'ir>) {
        let Some(activation) = self.activation(id) else {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::StaleActivation,
            ));
            return;
        };
        let node = self.ir.node(activation.node_id).expect("validated IR node");
        if !matches!(
            node.kind(),
            MissionNodeKind::Action(_) | MissionNodeKind::Timeout { .. }
        ) {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::ActivationHasNoDeadline,
            ));
            return;
        }
        self.request_cancel(
            id,
            MissionCancelReason::Timeout,
            MissionOutcome::TimedOut,
            step,
        );
    }

    fn cancel_scope(&mut self, scope_id: CancelScopeId, step: &mut MissionStep<'ir>) {
        let mut matches = ArrayVec::<MissionActivationId, MAX_ACTIVE_ACTIVATIONS>::new();
        for (index, slot) in self.slots.iter().enumerate() {
            let Some(activation) = slot.activation else {
                continue;
            };
            let node = self.ir.node(activation.node_id).expect("validated IR node");
            if matches!(
                node.kind(),
                MissionNodeKind::CancelScope {
                    scope_id: active_scope,
                    ..
                } if *active_scope == scope_id
            ) {
                matches
                    .try_push(MissionActivationId {
                        slot: u8::try_from(index).expect("activation capacity fits u8"),
                        generation: slot.generation,
                    })
                    .expect("match count cannot exceed active slots");
            }
        }
        if matches.is_empty() {
            step.push(MissionEffect::EventRejected(
                MissionEventRejection::InactiveCancelScope,
            ));
            return;
        }
        for id in matches {
            if self.activation(id).is_some() {
                self.request_cancel(
                    id,
                    MissionCancelReason::Scope,
                    MissionOutcome::Cancelled,
                    step,
                );
            }
        }
    }

    fn request_cancel(
        &mut self,
        id: MissionActivationId,
        reason: MissionCancelReason,
        forced: MissionOutcome,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(mut activation) = self.activation(id) else {
            return;
        };
        if activation.forced_outcome != Some(MissionOutcome::TimedOut)
            || forced == MissionOutcome::TimedOut
        {
            activation.forced_outcome = Some(forced);
            activation.cancel_reason = Some(reason);
        }
        match activation.phase {
            ActivationPhase::Entering | ActivationPhase::WaitingGuard => {
                self.set_activation(id, activation);
                self.complete_activation(id, forced, step);
            }
            ActivationPhase::WaitingDispatch | ActivationPhase::WaitingAcceptance(_) => {
                self.set_activation(id, activation);
            }
            ActivationPhase::RunningAction(command_id) => {
                activation.phase = ActivationPhase::CancelRequested(command_id);
                self.set_activation(id, activation);
                step.push(MissionEffect::RequestActionCancel { command_id, reason });
            }
            ActivationPhase::CancelRequested(_) | ActivationPhase::Canceling(_) => {
                self.set_activation(id, activation);
            }
            ActivationPhase::WaitingChildren => {
                self.set_activation(id, activation);
                let children = activation.children;
                if activation.live_children == 0 {
                    self.complete_activation(id, forced, step);
                    return;
                }
                for child in children.into_iter().flatten() {
                    if self.activation(child).is_some() {
                        self.request_cancel(child, reason, forced, step);
                    }
                }
            }
        }
    }

    fn complete_activation(
        &mut self,
        id: MissionActivationId,
        outcome: MissionOutcome,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(activation) = self.activation(id) else {
            return;
        };
        let node_index = self
            .ir
            .node_index(activation.node_id)
            .expect("validated runtime only completes known nodes");
        self.last_outcomes[node_index] = Some(outcome);
        self.clear_activation(id);
        let Some(parent) = activation.parent else {
            self.root_activation = None;
            self.state = MissionRuntimeState::Terminal(outcome);
            step.push(MissionEffect::MissionCompleted(outcome));
            return;
        };
        self.child_completed(parent, id, outcome, step);
    }

    fn child_completed(
        &mut self,
        parent_id: MissionActivationId,
        child_id: MissionActivationId,
        outcome: MissionOutcome,
        step: &mut MissionStep<'ir>,
    ) {
        let Some(mut parent) = self.activation(parent_id) else {
            return;
        };
        if let Some(index) = parent
            .children
            .iter()
            .position(|child| *child == Some(child_id))
        {
            parent.children[index] = None;
            parent.live_children = parent.live_children.saturating_sub(1);
        }
        let node = self.ir.node(parent.node_id).expect("validated IR node");

        match node.kind() {
            MissionNodeKind::Sequence(children) => {
                if outcome == MissionOutcome::Succeeded
                    && usize::from(parent.next_child) < children.as_slice().len()
                {
                    let child = children.as_slice()[usize::from(parent.next_child)];
                    parent.next_child = parent.next_child.saturating_add(1);
                    self.set_activation(parent_id, parent);
                    let _ = self.spawn_child(parent_id, child, step);
                } else {
                    self.set_activation(parent_id, parent);
                    self.complete_activation(parent_id, outcome, step);
                }
            }
            MissionNodeKind::Selector(children) => {
                if outcome == MissionOutcome::Succeeded || outcome == MissionOutcome::Cancelled {
                    self.set_activation(parent_id, parent);
                    self.complete_activation(parent_id, outcome, step);
                } else if usize::from(parent.next_child) < children.as_slice().len() {
                    let child = children.as_slice()[usize::from(parent.next_child)];
                    parent.next_child = parent.next_child.saturating_add(1);
                    self.set_activation(parent_id, parent);
                    let _ = self.spawn_child(parent_id, child, step);
                } else {
                    self.set_activation(parent_id, parent);
                    self.complete_activation(parent_id, MissionOutcome::Failed, step);
                }
            }
            MissionNodeKind::ParallelAll(_) => {
                if outcome != MissionOutcome::Succeeded && parent.forced_outcome.is_none() {
                    parent.forced_outcome = Some(outcome);
                }
                let forced = parent.forced_outcome;
                let children = parent.children;
                let live = parent.live_children;
                self.set_activation(parent_id, parent);
                if forced.is_some() && live > 0 {
                    for sibling in children.into_iter().flatten() {
                        if self.activation(sibling).is_some() {
                            self.request_cancel(
                                sibling,
                                MissionCancelReason::ParallelSibling,
                                MissionOutcome::Cancelled,
                                step,
                            );
                        }
                    }
                }
                if self
                    .activation(parent_id)
                    .is_some_and(|active| active.live_children == 0)
                {
                    self.complete_activation(
                        parent_id,
                        forced.unwrap_or(MissionOutcome::Succeeded),
                        step,
                    );
                }
            }
            MissionNodeKind::ParallelAny(_) => {
                if outcome == MissionOutcome::Succeeded && parent.forced_outcome.is_none() {
                    parent.forced_outcome = Some(MissionOutcome::Succeeded);
                }
                let forced = parent.forced_outcome;
                let children = parent.children;
                let live = parent.live_children;
                self.set_activation(parent_id, parent);
                if forced == Some(MissionOutcome::Succeeded) && live > 0 {
                    for sibling in children.into_iter().flatten() {
                        if self.activation(sibling).is_some() {
                            self.request_cancel(
                                sibling,
                                MissionCancelReason::ParallelSibling,
                                MissionOutcome::Cancelled,
                                step,
                            );
                        }
                    }
                }
                if self
                    .activation(parent_id)
                    .is_some_and(|active| active.live_children == 0)
                {
                    self.complete_activation(
                        parent_id,
                        forced.unwrap_or(MissionOutcome::Failed),
                        step,
                    );
                }
            }
            MissionNodeKind::Retry {
                child,
                max_attempts,
            } => {
                let forced = parent.forced_outcome;
                let maximum = max_attempts.expect("validated bounded retry");
                if forced.is_none()
                    && outcome != MissionOutcome::Succeeded
                    && outcome != MissionOutcome::Cancelled
                    && parent.repeat_count < maximum
                {
                    parent.repeat_count = parent.repeat_count.saturating_add(1);
                    self.set_activation(parent_id, parent);
                    let _ = self.spawn_child(parent_id, *child, step);
                } else {
                    self.set_activation(parent_id, parent);
                    self.complete_activation(parent_id, forced.unwrap_or(outcome), step);
                }
            }
            MissionNodeKind::Loop {
                child,
                max_iterations,
            } => {
                let forced = parent.forced_outcome;
                let maximum = max_iterations.expect("validated bounded loop");
                if forced.is_none()
                    && outcome == MissionOutcome::Succeeded
                    && parent.repeat_count < maximum
                {
                    parent.repeat_count = parent.repeat_count.saturating_add(1);
                    self.set_activation(parent_id, parent);
                    let _ = self.spawn_child(parent_id, *child, step);
                } else {
                    self.set_activation(parent_id, parent);
                    self.complete_activation(parent_id, forced.unwrap_or(outcome), step);
                }
            }
            MissionNodeKind::Timeout { .. }
            | MissionNodeKind::Guard { .. }
            | MissionNodeKind::CancelScope { .. }
            | MissionNodeKind::Subflow { .. } => {
                let effective = parent.forced_outcome.unwrap_or(outcome);
                self.set_activation(parent_id, parent);
                self.complete_activation(parent_id, effective, step);
            }
            MissionNodeKind::Action(_) => {
                self.fail_runtime(MissionRuntimeFault::InvariantViolation, step);
            }
        }
    }

    fn activation(&self, id: MissionActivationId) -> Option<Activation> {
        let slot = self.slots.get(usize::from(id.slot))?;
        (slot.generation == id.generation)
            .then_some(slot.activation)
            .flatten()
    }

    fn set_activation(&mut self, id: MissionActivationId, activation: Activation) {
        let slot = &mut self.slots[usize::from(id.slot)];
        debug_assert_eq!(slot.generation, id.generation);
        debug_assert!(slot.activation.is_some());
        slot.activation = Some(activation);
    }

    fn clear_activation(&mut self, id: MissionActivationId) {
        let slot = &mut self.slots[usize::from(id.slot)];
        if slot.generation == id.generation {
            slot.activation = None;
        }
    }

    fn find_command(&self, command_id: Id128) -> Option<MissionActivationId> {
        self.slots.iter().enumerate().find_map(|(index, slot)| {
            let activation = slot.activation?;
            (activation.phase.command_id() == Some(command_id)).then_some(MissionActivationId {
                slot: u8::try_from(index).expect("activation capacity fits u8"),
                generation: slot.generation,
            })
        })
    }

    fn mark_cancel_rejected_ancestors(&mut self, mut parent: Option<MissionActivationId>) {
        while let Some(id) = parent {
            let Some(mut activation) = self.activation(id) else {
                break;
            };
            if activation.forced_outcome == Some(MissionOutcome::Cancelled) {
                activation.forced_outcome = Some(MissionOutcome::Failed);
            }
            parent = activation.parent;
            self.set_activation(id, activation);
        }
    }

    fn fail_runtime(&mut self, fault: MissionRuntimeFault, step: &mut MissionStep<'ir>) {
        for slot in &mut self.slots {
            slot.activation = None;
        }
        self.root_activation = None;
        self.state = MissionRuntimeState::Faulted(fault);
        step.push(MissionEffect::RuntimeFault(fault));
    }
}

fn snapshot_activation_state(phase: ActivationPhase) -> BehaviorTreeActivationState {
    match phase {
        ActivationPhase::Entering => BehaviorTreeActivationState::Entering,
        ActivationPhase::WaitingDispatch => BehaviorTreeActivationState::WaitingDispatch,
        ActivationPhase::WaitingAcceptance(_) => {
            BehaviorTreeActivationState::WaitingAcknowledgement
        }
        ActivationPhase::RunningAction(_) => BehaviorTreeActivationState::Running,
        ActivationPhase::CancelRequested(_) => BehaviorTreeActivationState::CancelRequested,
        ActivationPhase::Canceling(_) => BehaviorTreeActivationState::Canceling,
        ActivationPhase::WaitingGuard => BehaviorTreeActivationState::WaitingCondition,
        ActivationPhase::WaitingChildren => BehaviorTreeActivationState::WaitingChildren,
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use message_model::{ActionName, ResourceId};

    use super::*;
    use crate::{
        MissionBuilder, MissionChildren, MissionNode, MissionNodeKind, MissionNodeName,
        MissionRegistry, MissionResources, TypedAction,
    };

    struct Goal;
    struct Output;

    fn token(value: u8) -> Id128 {
        Id128::new([value; 16]).unwrap()
    }

    fn name(value: &str) -> MissionNodeName {
        MissionNodeName::parse(value).unwrap()
    }

    fn action_name(value: &str) -> ActionName {
        ActionName::parse(value).unwrap()
    }

    fn resource(value: &str) -> ResourceId {
        ResourceId::parse(value).unwrap()
    }

    fn action(id: u16, action_name_value: &str, resource_value: &str) -> MissionNode {
        TypedAction::<Goal, Output>::new(
            action_name(action_name_value),
            MissionResources::try_from_slice(&[resource(resource_value)]).unwrap(),
            1_000,
        )
        .into_node(MissionNodeId::new(id), name(&format!("node_{id}")))
    }

    fn registry(actions: &[&str], guards: &[&str]) -> MissionRegistry {
        let mut registry = MissionRegistry::new();
        for action in actions {
            registry.register_action(action_name(action)).unwrap();
        }
        for guard in guards {
            registry
                .register_guard(GuardId::parse(guard).unwrap())
                .unwrap();
        }
        registry
    }

    fn validate(root: u16, nodes: impl IntoIterator<Item = MissionNode>) -> MissionIr {
        let mut builder = MissionBuilder::new(MissionNodeId::new(root));
        for node in nodes {
            builder.add_node(node).unwrap();
        }
        builder
            .validate(&registry(&["one", "two", "fallback"], &["condition"]))
            .unwrap()
    }

    fn dispatch_activation(step: &MissionStep<'_>, node_id: u16) -> MissionActivationId {
        step.effects()
            .iter()
            .find_map(|effect| match effect {
                MissionEffect::DispatchAction {
                    activation,
                    node_id: actual,
                    ..
                } if *actual == MissionNodeId::new(node_id) => Some(*activation),
                _ => None,
            })
            .expect("dispatch effect")
    }

    fn run_action<'ir>(
        runtime: &mut MissionRuntime<'ir>,
        activation: MissionActivationId,
        command_id: Id128,
        status: ActionStatus,
    ) -> MissionStep<'ir> {
        runtime.dispatch(MissionEvent::ActionDispatched {
            activation,
            command_id,
        });
        runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id,
            accepted: true,
        });
        runtime.dispatch(MissionEvent::ActionTerminal { command_id, status })
    }

    #[test]
    fn sequence_correlates_actions_and_replay_is_deterministic() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("root"),
                    MissionNodeKind::Sequence(
                        MissionChildren::try_from_slice(&[
                            MissionNodeId::new(1),
                            MissionNodeId::new(2),
                        ])
                        .unwrap(),
                    ),
                ),
                action(1, "one", "chassis"),
                action(2, "two", "arm"),
            ],
        );

        let replay = || {
            let mut runtime = MissionRuntime::new(&ir);
            let start = runtime.dispatch(MissionEvent::Start);
            let first = dispatch_activation(&start, 1);
            let wrong = runtime.dispatch(MissionEvent::ActionAcknowledged {
                command_id: token(9),
                accepted: true,
            });
            assert_eq!(
                wrong.effects(),
                &[MissionEffect::EventRejected(
                    MissionEventRejection::UnknownCommand
                )]
            );
            let next = run_action(&mut runtime, first, token(1), ActionStatus::Succeeded);
            assert!(matches!(
                next.effects().first(),
                Some(MissionEffect::ReleaseActionResources { .. })
            ));
            let second = dispatch_activation(&next, 2);
            let done = run_action(&mut runtime, second, token(2), ActionStatus::Succeeded);
            assert_eq!(
                done.effects().last(),
                Some(&MissionEffect::MissionCompleted(MissionOutcome::Succeeded))
            );
            assert_eq!(
                runtime.state(),
                MissionRuntimeState::Terminal(MissionOutcome::Succeeded)
            );
            (start, next, done)
        };
        assert_eq!(replay(), replay());
    }

    #[test]
    fn retry_and_loop_obey_finite_bounds() {
        let retry_ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("retry"),
                    MissionNodeKind::Retry {
                        child: MissionNodeId::new(1),
                        max_attempts: Some(2),
                    },
                ),
                action(1, "one", "chassis"),
            ],
        );
        let mut runtime = MissionRuntime::new(&retry_ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let first = dispatch_activation(&start, 1);
        let retried = run_action(&mut runtime, first, token(1), ActionStatus::Failed);
        let second = dispatch_activation(&retried, 1);
        let done = run_action(&mut runtime, second, token(2), ActionStatus::Succeeded);
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Succeeded))
        );

        let loop_ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("loop"),
                    MissionNodeKind::Loop {
                        child: MissionNodeId::new(1),
                        max_iterations: Some(2),
                    },
                ),
                action(1, "one", "chassis"),
            ],
        );
        let mut runtime = MissionRuntime::new(&loop_ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let first = dispatch_activation(&start, 1);
        let repeated = run_action(&mut runtime, first, token(3), ActionStatus::Succeeded);
        let second = dispatch_activation(&repeated, 1);
        let done = run_action(&mut runtime, second, token(4), ActionStatus::Succeeded);
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Succeeded))
        );
    }

    #[test]
    fn guard_and_selector_route_deterministically() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("guard"),
                    MissionNodeKind::Guard {
                        guard_id: GuardId::parse("condition").unwrap(),
                        if_true: MissionNodeId::new(1),
                        if_false: MissionNodeId::new(2),
                    },
                ),
                action(1, "one", "chassis"),
                MissionNode::new(
                    MissionNodeId::new(2),
                    name("selector"),
                    MissionNodeKind::Selector(
                        MissionChildren::try_from_slice(&[
                            MissionNodeId::new(3),
                            MissionNodeId::new(4),
                        ])
                        .unwrap(),
                    ),
                ),
                action(3, "two", "arm"),
                action(4, "fallback", "camera"),
            ],
        );
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let guard = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                MissionEffect::EvaluateGuard { activation, .. } => Some(*activation),
                _ => None,
            })
            .unwrap();
        let selected = runtime.dispatch(MissionEvent::GuardResolved {
            activation: guard,
            value: false,
        });
        let first = dispatch_activation(&selected, 3);
        let fallback = run_action(&mut runtime, first, token(1), ActionStatus::Failed);
        let second = dispatch_activation(&fallback, 4);
        let done = run_action(&mut runtime, second, token(2), ActionStatus::Succeeded);
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Succeeded))
        );
    }

    #[test]
    fn parallel_failure_cancels_siblings_and_waits_for_terminal_evidence() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("parallel"),
                    MissionNodeKind::ParallelAll(
                        MissionChildren::try_from_slice(&[
                            MissionNodeId::new(1),
                            MissionNodeId::new(2),
                        ])
                        .unwrap(),
                    ),
                ),
                action(1, "one", "chassis"),
                action(2, "two", "arm"),
            ],
        );
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let first = dispatch_activation(&start, 1);
        let second = dispatch_activation(&start, 2);
        for (activation, command_id) in [(first, token(1)), (second, token(2))] {
            runtime.dispatch(MissionEvent::ActionDispatched {
                activation,
                command_id,
            });
            runtime.dispatch(MissionEvent::ActionAcknowledged {
                command_id,
                accepted: true,
            });
        }
        let failed = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Failed,
        });
        assert!(
            failed
                .effects()
                .contains(&MissionEffect::RequestActionCancel {
                    command_id: token(2),
                    reason: MissionCancelReason::ParallelSibling
                })
        );
        assert_eq!(runtime.state(), MissionRuntimeState::Running);
        let accepted = runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(2),
            decision: ActionCancelDecision::Accepted,
        });
        assert!(accepted.effects().is_empty());
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(2),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Failed))
        );
    }

    #[test]
    fn timeout_and_scope_cancellation_are_correlated_and_nonterminal() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("timeout"),
                    MissionNodeKind::Timeout {
                        child: MissionNodeId::new(1),
                        timeout_ns: 100,
                    },
                ),
                MissionNode::new(
                    MissionNodeId::new(1),
                    name("scope"),
                    MissionNodeKind::CancelScope {
                        scope_id: CancelScopeId::parse("work").unwrap(),
                        child: MissionNodeId::new(2),
                    },
                ),
                action(2, "one", "chassis"),
            ],
        );
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let timeout = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                MissionEffect::ArmDeadline { activation, .. } => Some(*activation),
                _ => None,
            })
            .unwrap();
        let action = dispatch_activation(&start, 2);
        runtime.dispatch(MissionEvent::ActionDispatched {
            activation: action,
            command_id: token(1),
        });
        runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id: token(1),
            accepted: true,
        });
        let timed_out = runtime.dispatch(MissionEvent::DeadlineElapsed {
            activation: timeout,
        });
        assert_eq!(
            timed_out.effects(),
            &[MissionEffect::RequestActionCancel {
                command_id: token(1),
                reason: MissionCancelReason::Timeout
            }]
        );
        assert_eq!(runtime.state(), MissionRuntimeState::Running);
        assert!(
            runtime
                .dispatch(MissionEvent::CancelAll)
                .effects()
                .is_empty()
        );
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(1),
            decision: ActionCancelDecision::Accepted,
        });
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::TimedOut))
        );
    }

    #[test]
    fn parallel_any_waits_for_losing_sibling_cancellation() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("parallel_any"),
                    MissionNodeKind::ParallelAny(
                        MissionChildren::try_from_slice(&[
                            MissionNodeId::new(1),
                            MissionNodeId::new(2),
                        ])
                        .unwrap(),
                    ),
                ),
                action(1, "one", "chassis"),
                action(2, "two", "arm"),
            ],
        );
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let winner = dispatch_activation(&start, 1);
        let loser = dispatch_activation(&start, 2);
        for (activation, command_id) in [(winner, token(1)), (loser, token(2))] {
            runtime.dispatch(MissionEvent::ActionDispatched {
                activation,
                command_id,
            });
            runtime.dispatch(MissionEvent::ActionAcknowledged {
                command_id,
                accepted: true,
            });
        }
        let succeeded = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Succeeded,
        });
        assert!(
            succeeded
                .effects()
                .contains(&MissionEffect::RequestActionCancel {
                    command_id: token(2),
                    reason: MissionCancelReason::ParallelSibling
                })
        );
        assert_eq!(runtime.state(), MissionRuntimeState::Running);
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(2),
            decision: ActionCancelDecision::Accepted,
        });
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(2),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Succeeded))
        );
    }

    #[test]
    fn named_scope_initially_cancels_only_its_descendants() {
        let ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("parallel"),
                    MissionNodeKind::ParallelAll(
                        MissionChildren::try_from_slice(&[
                            MissionNodeId::new(1),
                            MissionNodeId::new(2),
                        ])
                        .unwrap(),
                    ),
                ),
                MissionNode::new(
                    MissionNodeId::new(1),
                    name("scope"),
                    MissionNodeKind::CancelScope {
                        scope_id: CancelScopeId::parse("target").unwrap(),
                        child: MissionNodeId::new(3),
                    },
                ),
                action(2, "two", "arm"),
                action(3, "one", "chassis"),
            ],
        );
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let scoped = dispatch_activation(&start, 3);
        let unscoped = dispatch_activation(&start, 2);
        for (activation, command_id) in [(scoped, token(1)), (unscoped, token(2))] {
            runtime.dispatch(MissionEvent::ActionDispatched {
                activation,
                command_id,
            });
            runtime.dispatch(MissionEvent::ActionAcknowledged {
                command_id,
                accepted: true,
            });
        }
        let cancel = runtime.dispatch(MissionEvent::CancelScope {
            scope_id: CancelScopeId::parse("target").unwrap(),
        });
        assert_eq!(
            cancel.effects(),
            &[MissionEffect::RequestActionCancel {
                command_id: token(1),
                reason: MissionCancelReason::Scope
            }]
        );
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(1),
            decision: ActionCancelDecision::Accepted,
        });
        let scoped_done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Cancelled,
        });
        assert!(
            scoped_done
                .effects()
                .contains(&MissionEffect::RequestActionCancel {
                    command_id: token(2),
                    reason: MissionCancelReason::ParallelSibling
                })
        );
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(2),
            decision: ActionCancelDecision::Accepted,
        });
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(2),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Cancelled))
        );
    }

    #[test]
    fn cancellation_before_dispatch_binding_waits_for_goal_acceptance() {
        let ir = validate(0, [action(0, "one", "chassis")]);
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let action = dispatch_activation(&start, 0);

        let pending = runtime.dispatch(MissionEvent::CancelAll);
        assert!(pending.effects().is_empty());
        assert_eq!(runtime.state(), MissionRuntimeState::Running);
        let bound = runtime.dispatch(MissionEvent::ActionDispatched {
            activation: action,
            command_id: token(1),
        });
        assert!(bound.effects().is_empty());
        let accepted = runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id: token(1),
            accepted: true,
        });
        assert_eq!(
            accepted.effects(),
            &[MissionEffect::RequestActionCancel {
                command_id: token(1),
                reason: MissionCancelReason::Mission
            }]
        );
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(1),
            decision: ActionCancelDecision::Accepted,
        });
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Cancelled))
        );
    }

    #[test]
    fn representative_project_runs_retry_branch_parallel_timeout_and_cancellation() {
        let ir = crate::representative_automatic_project().unwrap();
        let mut runtime = MissionRuntime::new(&ir);

        let start = runtime.dispatch(MissionEvent::Start);
        let guard = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                MissionEffect::EvaluateGuard { activation, .. } => Some(*activation),
                _ => None,
            })
            .unwrap();
        let navigate = runtime.dispatch(MissionEvent::GuardResolved {
            activation: guard,
            value: true,
        });
        let first_navigation = dispatch_activation(&navigate, 6);
        let retry = run_action(
            &mut runtime,
            first_navigation,
            token(1),
            ActionStatus::Failed,
        );
        let second_navigation = dispatch_activation(&retry, 6);
        let parallel = run_action(
            &mut runtime,
            second_navigation,
            token(2),
            ActionStatus::Succeeded,
        );
        let inspect = dispatch_activation(&parallel, 7);
        let align = dispatch_activation(&parallel, 9);
        assert!(
            parallel
                .effects()
                .iter()
                .any(|effect| matches!(effect, MissionEffect::ArmDeadline { .. }))
        );

        run_action(&mut runtime, inspect, token(3), ActionStatus::Succeeded);
        let docking = run_action(&mut runtime, align, token(4), ActionStatus::Succeeded);
        let dock = dispatch_activation(&docking, 12);
        runtime.dispatch(MissionEvent::ActionDispatched {
            activation: dock,
            command_id: token(5),
        });
        runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id: token(5),
            accepted: true,
        });

        let cancel = runtime.dispatch(MissionEvent::CancelScope {
            scope_id: CancelScopeId::parse("docking").unwrap(),
        });
        assert_eq!(
            cancel.effects(),
            &[MissionEffect::RequestActionCancel {
                command_id: token(5),
                reason: MissionCancelReason::Scope
            }]
        );
        runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(5),
            decision: ActionCancelDecision::Accepted,
        });
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(5),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Cancelled))
        );
    }

    #[test]
    fn stale_activation_duplicate_command_and_cancel_rejection_do_not_advance() {
        let ir = validate(0, [action(0, "one", "chassis")]);
        let mut runtime = MissionRuntime::new(&ir);
        let start = runtime.dispatch(MissionEvent::Start);
        let action_activation = dispatch_activation(&start, 0);
        runtime.dispatch(MissionEvent::ActionDispatched {
            activation: action_activation,
            command_id: token(1),
        });
        runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id: token(1),
            accepted: true,
        });
        let cancel = runtime.dispatch(MissionEvent::CancelAll);
        assert_eq!(
            cancel.effects(),
            &[MissionEffect::RequestActionCancel {
                command_id: token(1),
                reason: MissionCancelReason::Mission
            }]
        );
        let rejected = runtime.dispatch(MissionEvent::CancelDecision {
            command_id: token(1),
            decision: ActionCancelDecision::Rejected,
        });
        assert_eq!(
            rejected.effects(),
            &[MissionEffect::CancellationRejected {
                command_id: token(1)
            }]
        );
        assert_eq!(runtime.state(), MissionRuntimeState::Running);
        let done = runtime.dispatch(MissionEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Cancelled,
        });
        assert_eq!(
            done.effects().last(),
            Some(&MissionEffect::MissionCompleted(MissionOutcome::Failed))
        );
        let stale = runtime.dispatch(MissionEvent::ActionDispatched {
            activation: action_activation,
            command_id: token(2),
        });
        assert_eq!(
            stale.effects(),
            &[MissionEffect::EventRejected(
                MissionEventRejection::NotRunning
            )]
        );

        let retry_ir = validate(
            0,
            [
                MissionNode::new(
                    MissionNodeId::new(0),
                    name("retry"),
                    MissionNodeKind::Retry {
                        child: MissionNodeId::new(1),
                        max_attempts: Some(2),
                    },
                ),
                action(1, "one", "chassis"),
            ],
        );
        let mut second_runtime = MissionRuntime::new(&retry_ir);
        let second_start = second_runtime.dispatch(MissionEvent::Start);
        let old_activation = dispatch_activation(&second_start, 1);
        second_runtime.dispatch(MissionEvent::ActionDispatched {
            activation: old_activation,
            command_id: token(1),
        });
        let retried = second_runtime.dispatch(MissionEvent::ActionAcknowledged {
            command_id: token(1),
            accepted: false,
        });
        let new_activation = dispatch_activation(&retried, 1);
        let stale_during_run = second_runtime.dispatch(MissionEvent::ActionDispatched {
            activation: old_activation,
            command_id: token(2),
        });
        assert_eq!(
            stale_during_run.effects(),
            &[MissionEffect::EventRejected(
                MissionEventRejection::StaleActivation
            )]
        );
        let duplicate = second_runtime.dispatch(MissionEvent::ActionDispatched {
            activation: new_activation,
            command_id: token(1),
        });
        assert_eq!(
            duplicate.effects(),
            &[MissionEffect::EventRejected(
                MissionEventRejection::DuplicateCommand
            )]
        );
    }

    #[test]
    fn runtime_hot_path_allocates_nothing_after_ir_validation() {
        let ir = validate(0, [action(0, "one", "chassis")]);
        let allocations = measure(|| {
            let mut runtime = MissionRuntime::new(&ir);
            let start = runtime.dispatch(MissionEvent::Start);
            let action = dispatch_activation(&start, 0);
            runtime.dispatch(MissionEvent::ActionDispatched {
                activation: action,
                command_id: token(1),
            });
            runtime.dispatch(MissionEvent::ActionAcknowledged {
                command_id: token(1),
                accepted: true,
            });
            runtime.dispatch(MissionEvent::ActionTerminal {
                command_id: token(1),
                status: ActionStatus::Succeeded,
            });
        });
        assert_eq!(allocations.count_total, 0);
    }
}
