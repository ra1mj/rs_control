//! Public behavior-tree facade over the bounded mission reducer.
//!
//! This module intentionally reuses the validated mission IR and its single
//! event-driven runtime. It does not introduce a second scheduler, dynamic
//! callbacks, a configuration parser, or an I/O boundary. Composition roots
//! execute returned effects and feed their correlated evidence back through
//! [`BehaviorTreeRuntime`].

use std::fmt::Write as _;

use arrayvec::ArrayVec;

use crate::mission::{
    MAX_ACTIVE_ACTIVATIONS, MAX_MISSION_NODES, MissionActivationId, MissionIr, MissionNodeId,
    MissionNodeKind, MissionOutcome, MissionRuntimeState,
};

pub use crate::mission::{
    CancelScopeId as BehaviorTreeCancelScopeId, GuardId as BehaviorTreeGuardId,
    MissionAction as BehaviorTreeAction, MissionActivationId as BehaviorTreeActivationId,
    MissionBuildError as BehaviorTreeBuildError, MissionBuilder as BehaviorTreeBuilder,
    MissionCancelReason as BehaviorTreeCancelReason, MissionChildren as BehaviorTreeChildren,
    MissionEffect as BehaviorTreeEffect, MissionEvent as BehaviorTreeEvent,
    MissionEventRejection as BehaviorTreeEventRejection, MissionIr as BehaviorTree,
    MissionNode as BehaviorTreeNode, MissionNodeId as BehaviorTreeNodeId,
    MissionNodeKind as BehaviorTreeNodeKind, MissionNodeName as BehaviorTreeNodeName,
    MissionOutcome as BehaviorTreeOutcome, MissionRegistry as BehaviorTreeRegistry,
    MissionResources as BehaviorTreeResources, MissionRuntime as BehaviorTreeRuntime,
    MissionRuntimeFault as BehaviorTreeRuntimeFault,
    MissionRuntimeState as BehaviorTreeRuntimeState, MissionStep as BehaviorTreeStep,
    TypedAction as TypedBehaviorTreeAction,
};
/// Canonical externally allocated action command identity.
pub use message_model::Id128 as BehaviorTreeCommandId;

/// Maximum immutable nodes in one behavior tree.
pub const MAX_BEHAVIOR_TREE_NODES: usize = MAX_MISSION_NODES;
/// Maximum simultaneous behavior-tree activations.
pub const MAX_BEHAVIOR_TREE_ACTIVATIONS: usize = MAX_ACTIVE_ACTIVATIONS;

/// Static behavior-tree node category exposed by runtime snapshots.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BehaviorTreeNodeType {
    /// Registered action leaf.
    Action,
    /// Ordered all-success composite.
    Sequence,
    /// Ordered first-success composite.
    Selector,
    /// Concurrent all-success composite.
    ParallelAll,
    /// Concurrent first-success composite.
    ParallelAny,
    /// Finite deadline wrapper.
    Timeout,
    /// Finite retry wrapper.
    Retry,
    /// Finite loop wrapper.
    Loop,
    /// External condition branch.
    Guard,
    /// Named cancellation boundary.
    CancelScope,
    /// Named reusable subflow call.
    Subflow,
}

impl BehaviorTreeNodeType {
    /// Returns a stable lowercase identifier for text renderers.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Action => "action",
            Self::Sequence => "sequence",
            Self::Selector => "selector",
            Self::ParallelAll => "parallel_all",
            Self::ParallelAny => "parallel_any",
            Self::Timeout => "timeout",
            Self::Retry => "retry",
            Self::Loop => "loop",
            Self::Guard => "guard",
            Self::CancelScope => "cancel_scope",
            Self::Subflow => "subflow",
        }
    }

    pub(crate) fn from_mission(kind: &MissionNodeKind) -> Self {
        match kind {
            MissionNodeKind::Action(_) => Self::Action,
            MissionNodeKind::Sequence(_) => Self::Sequence,
            MissionNodeKind::Selector(_) => Self::Selector,
            MissionNodeKind::ParallelAll(_) => Self::ParallelAll,
            MissionNodeKind::ParallelAny(_) => Self::ParallelAny,
            MissionNodeKind::Timeout { .. } => Self::Timeout,
            MissionNodeKind::Retry { .. } => Self::Retry,
            MissionNodeKind::Loop { .. } => Self::Loop,
            MissionNodeKind::Guard { .. } => Self::Guard,
            MissionNodeKind::CancelScope { .. } => Self::CancelScope,
            MissionNodeKind::Subflow { .. } => Self::Subflow,
        }
    }
}

/// Aggregate participation of one static node in the current run.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BehaviorTreeNodeParticipation {
    /// The node has not run, or its previous terminal state was cleared by a
    /// new activation.
    Inactive,
    /// One or more exact activation records are live for this node.
    Active {
        /// Number of live activations. The exact records are available from
        /// [`BehaviorTreeSnapshot::activations`].
        activation_count: u8,
    },
    /// The node has no live activation and has a retained terminal outcome.
    Terminal(BehaviorTreeOutcome),
}

impl BehaviorTreeNodeParticipation {
    /// Returns the number of currently live activations.
    #[must_use]
    pub const fn active_count(self) -> u8 {
        match self {
            Self::Active { activation_count } => activation_count,
            Self::Inactive | Self::Terminal(_) => 0,
        }
    }
}

/// Exact execution phase of one live activation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BehaviorTreeActivationState {
    /// Activation was allocated and has not yet produced entry effects.
    Entering,
    /// Action needs an externally allocated command identity.
    WaitingDispatch,
    /// Action command is bound and waits for goal acknowledgement.
    WaitingAcknowledgement,
    /// Action goal is executing.
    Running,
    /// A cancel request was emitted and a decision is pending.
    CancelRequested,
    /// Remote cancel was accepted and terminal evidence is pending.
    Canceling,
    /// Guard evaluation is pending.
    WaitingCondition,
    /// A composite waits for one or more child activations.
    WaitingChildren,
}

impl BehaviorTreeActivationState {
    /// Returns a stable lowercase identifier for text renderers.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Entering => "entering",
            Self::WaitingDispatch => "waiting_dispatch",
            Self::WaitingAcknowledgement => "waiting_acknowledgement",
            Self::Running => "running",
            Self::CancelRequested => "cancel_requested",
            Self::Canceling => "canceling",
            Self::WaitingCondition => "waiting_condition",
            Self::WaitingChildren => "waiting_children",
        }
    }
}

/// Snapshot of one static node, always emitted in validated builder order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BehaviorTreeNodeSnapshot {
    node_id: BehaviorTreeNodeId,
    name: BehaviorTreeNodeName,
    node_type: BehaviorTreeNodeType,
    participation: BehaviorTreeNodeParticipation,
    last_outcome: Option<BehaviorTreeOutcome>,
}

impl BehaviorTreeNodeSnapshot {
    pub(crate) const fn new(
        node_id: BehaviorTreeNodeId,
        name: BehaviorTreeNodeName,
        node_type: BehaviorTreeNodeType,
        participation: BehaviorTreeNodeParticipation,
        last_outcome: Option<BehaviorTreeOutcome>,
    ) -> Self {
        Self {
            node_id,
            name,
            node_type,
            participation,
            last_outcome,
        }
    }

    /// Returns the immutable node identity.
    #[must_use]
    pub const fn node_id(&self) -> BehaviorTreeNodeId {
        self.node_id
    }

    /// Returns the immutable node name.
    #[must_use]
    pub const fn name(&self) -> BehaviorTreeNodeName {
        self.name
    }

    /// Returns the immutable node category.
    #[must_use]
    pub const fn node_type(&self) -> BehaviorTreeNodeType {
        self.node_type
    }

    /// Returns current aggregate participation.
    #[must_use]
    pub const fn participation(&self) -> BehaviorTreeNodeParticipation {
        self.participation
    }

    /// Returns the latest terminal outcome, if the node is not reactivated.
    #[must_use]
    pub const fn last_outcome(&self) -> Option<BehaviorTreeOutcome> {
        self.last_outcome
    }
}

/// Snapshot of one exact live activation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BehaviorTreeActivationSnapshot {
    activation: BehaviorTreeActivationId,
    parent_activation: Option<BehaviorTreeActivationId>,
    node_id: BehaviorTreeNodeId,
    name: BehaviorTreeNodeName,
    node_type: BehaviorTreeNodeType,
    state: BehaviorTreeActivationState,
    command_id: Option<BehaviorTreeCommandId>,
}

impl BehaviorTreeActivationSnapshot {
    #[allow(clippy::too_many_arguments)]
    pub(crate) const fn new(
        activation: BehaviorTreeActivationId,
        parent_activation: Option<BehaviorTreeActivationId>,
        node_id: BehaviorTreeNodeId,
        name: BehaviorTreeNodeName,
        node_type: BehaviorTreeNodeType,
        state: BehaviorTreeActivationState,
        command_id: Option<BehaviorTreeCommandId>,
    ) -> Self {
        Self {
            activation,
            parent_activation,
            node_id,
            name,
            node_type,
            state,
            command_id,
        }
    }

    /// Returns the generation-tagged activation identity.
    #[must_use]
    pub const fn activation(&self) -> BehaviorTreeActivationId {
        self.activation
    }

    /// Returns the parent activation, when this is not the root.
    #[must_use]
    pub const fn parent_activation(&self) -> Option<BehaviorTreeActivationId> {
        self.parent_activation
    }

    /// Returns the static node identity.
    #[must_use]
    pub const fn node_id(&self) -> BehaviorTreeNodeId {
        self.node_id
    }

    /// Returns the static node name.
    #[must_use]
    pub const fn name(&self) -> BehaviorTreeNodeName {
        self.name
    }

    /// Returns the static node category.
    #[must_use]
    pub const fn node_type(&self) -> BehaviorTreeNodeType {
        self.node_type
    }

    /// Returns the exact live execution phase.
    #[must_use]
    pub const fn state(&self) -> BehaviorTreeActivationState {
        self.state
    }

    /// Returns the bound action command identity, where applicable.
    #[must_use]
    pub const fn command_id(&self) -> Option<BehaviorTreeCommandId> {
        self.command_id
    }
}

/// Bounded, allocation-free runtime inspection result.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BehaviorTreeSnapshot {
    state: BehaviorTreeRuntimeState,
    nodes: ArrayVec<BehaviorTreeNodeSnapshot, MAX_BEHAVIOR_TREE_NODES>,
    activations: ArrayVec<BehaviorTreeActivationSnapshot, MAX_BEHAVIOR_TREE_ACTIVATIONS>,
}

impl BehaviorTreeSnapshot {
    pub(crate) const fn new(
        state: BehaviorTreeRuntimeState,
        nodes: ArrayVec<BehaviorTreeNodeSnapshot, MAX_BEHAVIOR_TREE_NODES>,
        activations: ArrayVec<BehaviorTreeActivationSnapshot, MAX_BEHAVIOR_TREE_ACTIVATIONS>,
    ) -> Self {
        Self {
            state,
            nodes,
            activations,
        }
    }

    /// Returns the aggregate tree runtime state.
    #[must_use]
    pub const fn state(&self) -> BehaviorTreeRuntimeState {
        self.state
    }

    /// Returns every static node in deterministic builder order.
    #[must_use]
    pub fn nodes(&self) -> &[BehaviorTreeNodeSnapshot] {
        self.nodes.as_slice()
    }

    /// Returns every live activation in deterministic slot order.
    #[must_use]
    pub fn activations(&self) -> &[BehaviorTreeActivationSnapshot] {
        self.activations.as_slice()
    }

    /// Renders stable human-readable live state outside the realtime path.
    #[must_use]
    pub fn render(&self) -> String {
        let mut rendered = String::new();
        rendered.push_str("BehaviorTreeSnapshot state=");
        write_runtime_state(&mut rendered, self.state);
        rendered.push('\n');

        for node in &self.nodes {
            write!(
                rendered,
                "node id={} name={} type={} state=",
                node.node_id.get(),
                node.name,
                node.node_type.as_str(),
            )
            .expect("writing to String cannot fail");
            write_participation(&mut rendered, node.participation);
            rendered.push_str(" last_outcome=");
            write_optional_outcome(&mut rendered, node.last_outcome);
            rendered.push('\n');
        }

        for activation in &self.activations {
            write!(
                rendered,
                "activation slot={} generation={} node={} name={} type={} state={} command=",
                activation.activation.slot(),
                activation.activation.generation(),
                activation.node_id.get(),
                activation.name,
                activation.node_type.as_str(),
                activation.state.as_str(),
            )
            .expect("writing to String cannot fail");
            match activation.command_id {
                Some(command_id) => write_id128(&mut rendered, command_id),
                None => rendered.push('-'),
            }
            rendered.push_str(" parent=");
            write_optional_activation(&mut rendered, activation.parent_activation);
            rendered.push('\n');
        }
        rendered
    }
}

impl MissionIr {
    /// Renders the immutable validated tree in deterministic hierarchy order.
    ///
    /// Rendering may allocate and is intended for operator tools, tests and
    /// offline diagnostics rather than the reducer hot path.
    #[must_use]
    pub fn render_tree(&self) -> String {
        let mut rendered = String::new();
        writeln!(rendered, "BehaviorTree root={}", self.root().get())
            .expect("writing to String cannot fail");
        let mut visited = [false; MAX_MISSION_NODES];
        render_node(self, self.root(), 0, &mut visited, &mut rendered);
        rendered
    }
}

fn render_node(
    tree: &MissionIr,
    node_id: MissionNodeId,
    depth: usize,
    visited: &mut [bool; MAX_MISSION_NODES],
    rendered: &mut String,
) {
    let node_index = tree
        .node_index(node_id)
        .expect("validated behavior tree always resolves child nodes");
    let node = &tree.nodes()[node_index];
    for _ in 0..depth {
        rendered.push_str("  ");
    }
    write!(
        rendered,
        "- [{}] {} type={}",
        node.id().get(),
        node.name(),
        BehaviorTreeNodeType::from_mission(node.kind()).as_str(),
    )
    .expect("writing to String cannot fail");
    render_node_details(rendered, node.kind());
    if visited[node_index] {
        rendered.push_str(" shared=true\n");
        return;
    }
    visited[node_index] = true;
    rendered.push('\n');

    for child_index in 0..node.kind().child_count() {
        let child = node
            .kind()
            .child_at(child_index)
            .expect("node child count and lookup agree");
        render_node(tree, child, depth.saturating_add(1), visited, rendered);
    }
}

fn render_node_details(rendered: &mut String, kind: &MissionNodeKind) {
    match kind {
        MissionNodeKind::Action(action) => {
            write!(rendered, " action={} resources=[", action.action_name(),)
                .expect("writing to String cannot fail");
            for (index, resource) in action.resources().as_slice().iter().enumerate() {
                if index != 0 {
                    rendered.push(',');
                }
                write!(rendered, "{resource}").expect("writing to String cannot fail");
            }
            write!(rendered, "] timeout_ns={}", action.timeout_ns())
                .expect("writing to String cannot fail");
        }
        MissionNodeKind::Sequence(children)
        | MissionNodeKind::Selector(children)
        | MissionNodeKind::ParallelAll(children)
        | MissionNodeKind::ParallelAny(children) => {
            rendered.push_str(" children=[");
            render_children(rendered, children.as_slice());
            rendered.push(']');
        }
        MissionNodeKind::Timeout { timeout_ns, .. } => {
            write!(rendered, " timeout_ns={timeout_ns}").expect("writing to String cannot fail");
        }
        MissionNodeKind::Retry { max_attempts, .. } => {
            write!(rendered, " max_attempts={max_attempts:?}")
                .expect("writing to String cannot fail");
        }
        MissionNodeKind::Loop { max_iterations, .. } => {
            write!(rendered, " max_iterations={max_iterations:?}")
                .expect("writing to String cannot fail");
        }
        MissionNodeKind::Guard {
            guard_id,
            if_true,
            if_false,
        } => {
            write!(
                rendered,
                " guard={} if_true={} if_false={}",
                guard_id,
                if_true.get(),
                if_false.get(),
            )
            .expect("writing to String cannot fail");
        }
        MissionNodeKind::CancelScope { scope_id, .. } => {
            write!(rendered, " scope={scope_id}").expect("writing to String cannot fail");
        }
        MissionNodeKind::Subflow { child } => {
            write!(rendered, " child={}", child.get()).expect("writing to String cannot fail");
        }
    }
}

fn render_children(rendered: &mut String, children: &[MissionNodeId]) {
    for (index, child) in children.iter().enumerate() {
        if index != 0 {
            rendered.push(',');
        }
        write!(rendered, "{}", child.get()).expect("writing to String cannot fail");
    }
}

fn write_runtime_state(rendered: &mut String, state: MissionRuntimeState) {
    match state {
        MissionRuntimeState::Idle => rendered.push_str("idle"),
        MissionRuntimeState::Running => rendered.push_str("running"),
        MissionRuntimeState::Terminal(outcome) => {
            rendered.push_str("terminal:");
            rendered.push_str(outcome_name(outcome));
        }
        MissionRuntimeState::Faulted(fault) => {
            write!(rendered, "faulted:{fault:?}").expect("writing to String cannot fail");
        }
    }
}

fn write_participation(rendered: &mut String, participation: BehaviorTreeNodeParticipation) {
    match participation {
        BehaviorTreeNodeParticipation::Inactive => rendered.push_str("inactive"),
        BehaviorTreeNodeParticipation::Active { activation_count } => {
            write!(rendered, "active:{activation_count}").expect("writing to String cannot fail");
        }
        BehaviorTreeNodeParticipation::Terminal(outcome) => {
            rendered.push_str("terminal:");
            rendered.push_str(outcome_name(outcome));
        }
    }
}

fn write_optional_outcome(rendered: &mut String, outcome: Option<MissionOutcome>) {
    match outcome {
        Some(outcome) => rendered.push_str(outcome_name(outcome)),
        None => rendered.push('-'),
    }
}

fn write_optional_activation(rendered: &mut String, activation: Option<MissionActivationId>) {
    match activation {
        Some(activation) => write!(
            rendered,
            "{}:{}",
            activation.slot(),
            activation.generation(),
        )
        .expect("writing to String cannot fail"),
        None => rendered.push('-'),
    }
}

fn write_id128(rendered: &mut String, value: BehaviorTreeCommandId) {
    for byte in value.as_bytes() {
        write!(rendered, "{byte:02x}").expect("writing to String cannot fail");
    }
}

const fn outcome_name(outcome: MissionOutcome) -> &'static str {
    match outcome {
        MissionOutcome::Succeeded => "succeeded",
        MissionOutcome::Failed => "failed",
        MissionOutcome::TimedOut => "timed_out",
        MissionOutcome::Cancelled => "cancelled",
        MissionOutcome::Rejected => "rejected",
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use message_model::{ActionCancelDecision, ActionName, ActionStatus, Id128, ResourceId};

    use super::*;

    struct Goal;
    struct ResultPayload;

    fn token(value: u8) -> Id128 {
        Id128::new([value; 16]).expect("nonzero static identity")
    }

    fn name(value: &str) -> BehaviorTreeNodeName {
        BehaviorTreeNodeName::parse(value).expect("static node name")
    }

    fn action_name(value: &str) -> ActionName {
        ActionName::parse(value).expect("static action name")
    }

    fn resource(value: &str) -> ResourceId {
        ResourceId::parse(value).expect("static resource name")
    }

    fn action_node(
        id: u16,
        node_name: &str,
        action: &str,
        resource_name: &str,
    ) -> BehaviorTreeNode {
        TypedBehaviorTreeAction::<Goal, ResultPayload>::new(
            action_name(action),
            BehaviorTreeResources::try_from_slice(&[resource(resource_name)])
                .expect("single resource fits"),
            1_000,
        )
        .into_node(BehaviorTreeNodeId::new(id), name(node_name))
    }

    fn registry(actions: &[&str], guards: &[&str]) -> BehaviorTreeRegistry {
        let mut registry = BehaviorTreeRegistry::new();
        for action in actions {
            registry
                .register_action(action_name(action))
                .expect("unique static action");
        }
        for guard in guards {
            registry
                .register_guard(BehaviorTreeGuardId::parse(guard).expect("static guard"))
                .expect("unique static guard");
        }
        registry
    }

    fn action_tree() -> BehaviorTree {
        let mut builder = BehaviorTreeBuilder::new(BehaviorTreeNodeId::new(0));
        builder
            .add_node(action_node(0, "move", "move", "chassis"))
            .expect("one node fits");
        builder
            .validate(&registry(&["move"], &[]))
            .expect("valid static action tree")
    }

    #[test]
    fn facade_snapshot_tracks_dispatch_cancellation_terminal_and_stale_events() {
        let tree = action_tree();
        assert_eq!(
            tree.render_tree(),
            "BehaviorTree root=0\n- [0] move type=action action=move resources=[chassis] timeout_ns=1000\n"
        );

        let mut runtime = BehaviorTreeRuntime::new(&tree);
        let initial = runtime.snapshot();
        assert_eq!(initial.state(), BehaviorTreeRuntimeState::Idle);
        assert_eq!(
            initial.nodes()[0].participation(),
            BehaviorTreeNodeParticipation::Inactive
        );

        let start = runtime.advance(BehaviorTreeEvent::Start);
        let activation = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                BehaviorTreeEffect::DispatchAction { activation, .. } => Some(*activation),
                _ => None,
            })
            .expect("start dispatches the action");
        assert_eq!(
            start
                .effects()
                .iter()
                .filter(|effect| matches!(effect, BehaviorTreeEffect::ArmDeadline { .. }))
                .count(),
            1
        );

        let waiting_dispatch = runtime.snapshot();
        assert_eq!(
            waiting_dispatch.nodes()[0].participation(),
            BehaviorTreeNodeParticipation::Active {
                activation_count: 1
            }
        );
        assert_eq!(
            waiting_dispatch.activations()[0].state(),
            BehaviorTreeActivationState::WaitingDispatch
        );
        assert_eq!(waiting_dispatch.activations()[0].command_id(), None);
        assert_eq!(
            waiting_dispatch.render(),
            "BehaviorTreeSnapshot state=running\nnode id=0 name=move type=action state=active:1 last_outcome=-\nactivation slot=0 generation=1 node=0 name=move type=action state=waiting_dispatch command=- parent=-\n"
        );

        runtime.advance(BehaviorTreeEvent::ActionDispatched {
            activation,
            command_id: token(1),
        });
        let waiting_ack = runtime.snapshot();
        assert_eq!(
            waiting_ack.activations()[0].state(),
            BehaviorTreeActivationState::WaitingAcknowledgement
        );
        assert_eq!(waiting_ack.activations()[0].command_id(), Some(token(1)));

        runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
            command_id: token(1),
            accepted: true,
        });
        assert_eq!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::Running
        );

        let cancel = runtime.advance(BehaviorTreeEvent::CancelAll);
        assert_eq!(
            cancel.effects(),
            &[BehaviorTreeEffect::RequestActionCancel {
                command_id: token(1),
                reason: BehaviorTreeCancelReason::Mission,
            }]
        );
        assert_eq!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::CancelRequested
        );

        runtime.advance(BehaviorTreeEvent::CancelDecision {
            command_id: token(1),
            decision: ActionCancelDecision::Accepted,
        });
        assert_eq!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::Canceling
        );

        runtime.advance(BehaviorTreeEvent::ActionTerminal {
            command_id: token(1),
            status: ActionStatus::Cancelled,
        });
        let terminal = runtime.snapshot();
        assert_eq!(
            terminal.state(),
            BehaviorTreeRuntimeState::Terminal(BehaviorTreeOutcome::Cancelled)
        );
        assert!(terminal.activations().is_empty());
        assert_eq!(
            terminal.nodes()[0].participation(),
            BehaviorTreeNodeParticipation::Terminal(BehaviorTreeOutcome::Cancelled)
        );
        assert_eq!(
            terminal.nodes()[0].last_outcome(),
            Some(BehaviorTreeOutcome::Cancelled)
        );

        let stale = runtime.advance(BehaviorTreeEvent::DeadlineElapsed { activation });
        assert_eq!(
            stale.effects(),
            &[BehaviorTreeEffect::EventRejected(
                BehaviorTreeEventRejection::NotRunning
            )]
        );
    }

    #[test]
    fn facade_validates_and_renders_every_supported_composition() {
        let mut builder = BehaviorTreeBuilder::new(BehaviorTreeNodeId::new(0));
        for node in [
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(0),
                name("root"),
                BehaviorTreeNodeKind::Sequence(
                    BehaviorTreeChildren::try_from_slice(&[
                        BehaviorTreeNodeId::new(1),
                        BehaviorTreeNodeId::new(2),
                        BehaviorTreeNodeId::new(3),
                    ])
                    .expect("bounded children"),
                ),
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(1),
                name("route_guard"),
                BehaviorTreeNodeKind::Guard {
                    guard_id: BehaviorTreeGuardId::parse("route_clear").expect("static guard"),
                    if_true: BehaviorTreeNodeId::new(4),
                    if_false: BehaviorTreeNodeId::new(7),
                },
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(2),
                name("observe_and_align"),
                BehaviorTreeNodeKind::ParallelAll(
                    BehaviorTreeChildren::try_from_slice(&[
                        BehaviorTreeNodeId::new(8),
                        BehaviorTreeNodeId::new(9),
                    ])
                    .expect("bounded children"),
                ),
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(3),
                name("dock_scope"),
                BehaviorTreeNodeKind::CancelScope {
                    scope_id: BehaviorTreeCancelScopeId::parse("docking").expect("static scope"),
                    child: BehaviorTreeNodeId::new(13),
                },
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(4),
                name("route_selector"),
                BehaviorTreeNodeKind::Selector(
                    BehaviorTreeChildren::try_from_slice(&[
                        BehaviorTreeNodeId::new(5),
                        BehaviorTreeNodeId::new(6),
                    ])
                    .expect("bounded children"),
                ),
            ),
            action_node(5, "navigate", "navigate", "chassis"),
            action_node(6, "fallback", "fallback", "chassis"),
            action_node(7, "safe_wait", "safe_wait", "chassis"),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(8),
                name("retry_inspect"),
                BehaviorTreeNodeKind::Retry {
                    child: BehaviorTreeNodeId::new(10),
                    max_attempts: Some(2),
                },
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(9),
                name("align_or_scan"),
                BehaviorTreeNodeKind::ParallelAny(
                    BehaviorTreeChildren::try_from_slice(&[
                        BehaviorTreeNodeId::new(11),
                        BehaviorTreeNodeId::new(12),
                    ])
                    .expect("bounded children"),
                ),
            ),
            action_node(10, "inspect", "inspect", "camera"),
            action_node(11, "align", "align", "arm"),
            action_node(12, "scan", "scan", "sensor"),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(13),
                name("dock_timeout"),
                BehaviorTreeNodeKind::Timeout {
                    child: BehaviorTreeNodeId::new(14),
                    timeout_ns: 2_000,
                },
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(14),
                name("dock_loop"),
                BehaviorTreeNodeKind::Loop {
                    child: BehaviorTreeNodeId::new(15),
                    max_iterations: Some(2),
                },
            ),
            BehaviorTreeNode::new(
                BehaviorTreeNodeId::new(15),
                name("dock_subflow"),
                BehaviorTreeNodeKind::Subflow {
                    child: BehaviorTreeNodeId::new(16),
                },
            ),
            action_node(16, "dock", "dock", "chassis"),
        ] {
            builder.add_node(node).expect("bounded tree node");
        }
        let tree = builder
            .validate(&registry(
                &[
                    "navigate",
                    "fallback",
                    "safe_wait",
                    "inspect",
                    "align",
                    "scan",
                    "dock",
                ],
                &["route_clear"],
            ))
            .expect("complete bounded tree validates");
        let first = tree.render_tree();
        assert_eq!(first, tree.render_tree());
        for node_type in [
            "sequence",
            "selector",
            "parallel_all",
            "parallel_any",
            "timeout",
            "retry",
            "loop",
            "guard",
            "cancel_scope",
            "subflow",
        ] {
            assert!(first.contains(&format!("type={node_type}")), "{node_type}");
        }
    }

    #[test]
    fn action_deadline_is_armed_once_and_preserves_timeout_through_cancellation() {
        let tree = action_tree();
        let mut runtime = BehaviorTreeRuntime::new(&tree);
        let start = runtime.advance(BehaviorTreeEvent::Start);
        let activation = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                BehaviorTreeEffect::DispatchAction { activation, .. } => Some(*activation),
                _ => None,
            })
            .expect("start dispatches action");
        let deadlines = start
            .effects()
            .iter()
            .filter_map(|effect| match effect {
                BehaviorTreeEffect::ArmDeadline {
                    activation: deadline_activation,
                    timeout_ns,
                } if *deadline_activation == activation => Some(*timeout_ns),
                _ => None,
            })
            .collect::<Vec<_>>();
        assert_eq!(deadlines, vec![1_000]);

        let before_binding = runtime.advance(BehaviorTreeEvent::DeadlineElapsed { activation });
        assert!(before_binding.effects().is_empty());
        let bound = runtime.advance(BehaviorTreeEvent::ActionDispatched {
            activation,
            command_id: token(9),
        });
        assert!(bound.effects().is_empty());
        let acknowledged = runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
            command_id: token(9),
            accepted: true,
        });
        assert_eq!(
            acknowledged.effects(),
            &[BehaviorTreeEffect::RequestActionCancel {
                command_id: token(9),
                reason: BehaviorTreeCancelReason::Timeout,
            }]
        );
        assert_eq!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::CancelRequested
        );

        runtime.advance(BehaviorTreeEvent::CancelDecision {
            command_id: token(9),
            decision: ActionCancelDecision::Accepted,
        });
        runtime.advance(BehaviorTreeEvent::ActionTerminal {
            command_id: token(9),
            status: ActionStatus::Cancelled,
        });
        let snapshot = runtime.snapshot();
        assert_eq!(
            snapshot.state(),
            BehaviorTreeRuntimeState::Terminal(BehaviorTreeOutcome::TimedOut)
        );
        assert_eq!(
            snapshot.nodes()[0].last_outcome(),
            Some(BehaviorTreeOutcome::TimedOut)
        );
    }

    #[test]
    fn runtime_snapshot_creation_is_allocation_free() {
        let tree = action_tree();
        let mut runtime = BehaviorTreeRuntime::new(&tree);
        runtime.advance(BehaviorTreeEvent::Start);
        let allocations = measure(|| {
            let snapshot = runtime.snapshot();
            assert_eq!(snapshot.nodes().len(), 1);
        });
        assert_eq!(allocations.count_total, 0);
    }
}
