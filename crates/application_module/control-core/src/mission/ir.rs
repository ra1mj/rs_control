//! Bounded typed mission builder and immutable validated IR.

use std::marker::PhantomData;

use arrayvec::ArrayVec;
use message_model::{ActionName, AsciiId, ResourceId};
use thiserror::Error;

/// Maximum nodes in one validated mission.
pub const MAX_MISSION_NODES: usize = 64;
/// Maximum direct children of one composite.
pub const MAX_MISSION_CHILDREN: usize = 8;
/// Maximum exclusive resources declared by one action.
pub const MAX_ACTION_RESOURCES: usize = 8;
/// Maximum registered action types.
pub const MAX_REGISTERED_ACTIONS: usize = 32;
/// Maximum registered guard conditions.
pub const MAX_REGISTERED_GUARDS: usize = 32;
/// Maximum action invocations in one complete validated run.
pub const MAX_MISSION_ACTION_INVOCATIONS: usize = 256;
/// Maximum simultaneous activation frames in one validated run.
pub const MAX_MISSION_ACTIVE_ACTIVATIONS: usize = MAX_MISSION_NODES;
/// Maximum transitive resource references in one mission.
const MAX_RESOURCE_REFERENCES: usize = MAX_MISSION_NODES * MAX_ACTION_RESOURCES;

/// Stable mission node identity.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct MissionNodeId(u16);

impl MissionNodeId {
    /// Creates a stable numeric identity.
    #[must_use]
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    /// Returns the stable numeric value.
    #[must_use]
    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Stable human-readable node name.
pub type MissionNodeName = AsciiId<64>;
/// Stable guard registry identity.
pub type GuardId = AsciiId<64>;
/// Stable cancellation-scope identity.
pub type CancelScopeId = AsciiId<64>;

/// Bounded exclusive-resource declaration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionResources {
    items: ArrayVec<ResourceId, MAX_ACTION_RESOURCES>,
}

impl MissionResources {
    /// Copies a bounded resource slice in declaration order.
    pub fn try_from_slice(resources: &[ResourceId]) -> Result<Self, MissionBuildError> {
        let mut items = ArrayVec::new();
        for resource in resources {
            items
                .try_push(*resource)
                .map_err(|_| MissionBuildError::ResourceCapacityExceeded {
                    maximum: MAX_ACTION_RESOURCES,
                })?;
        }
        Ok(Self { items })
    }

    /// Returns resources in deterministic declaration order.
    #[must_use]
    pub fn as_slice(&self) -> &[ResourceId] {
        self.items.as_slice()
    }
}

/// Bounded direct-child list.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionChildren {
    items: ArrayVec<MissionNodeId, MAX_MISSION_CHILDREN>,
}

impl MissionChildren {
    /// Copies a bounded child slice in evaluation order.
    pub fn try_from_slice(children: &[MissionNodeId]) -> Result<Self, MissionBuildError> {
        let mut items = ArrayVec::new();
        for child in children {
            items
                .try_push(*child)
                .map_err(|_| MissionBuildError::ChildCapacityExceeded {
                    maximum: MAX_MISSION_CHILDREN,
                })?;
        }
        Ok(Self { items })
    }

    /// Returns child IDs in deterministic evaluation order.
    #[must_use]
    pub fn as_slice(&self) -> &[MissionNodeId] {
        self.items.as_slice()
    }
}

/// Erased action definition retained in validated IR.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionAction {
    action_name: ActionName,
    resources: MissionResources,
    timeout_ns: u64,
}

impl MissionAction {
    /// Returns the registered action identity.
    #[must_use]
    pub const fn action_name(&self) -> ActionName {
        self.action_name
    }

    /// Returns declared exclusive resources.
    #[must_use]
    pub const fn resources(&self) -> &MissionResources {
        &self.resources
    }

    /// Returns the finite action timeout.
    #[must_use]
    pub const fn timeout_ns(&self) -> u64 {
        self.timeout_ns
    }
}

/// Compile-time input/output marker for a registered action node.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TypedAction<Input, Output> {
    action: MissionAction,
    marker: PhantomData<fn(Input) -> Output>,
}

impl<Input, Output> TypedAction<Input, Output> {
    /// Declares a typed action before it is erased into mission IR.
    #[must_use]
    pub const fn new(
        action_name: ActionName,
        resources: MissionResources,
        timeout_ns: u64,
    ) -> Self {
        Self {
            action: MissionAction {
                action_name,
                resources,
                timeout_ns,
            },
            marker: PhantomData,
        }
    }

    /// Creates an action node while preserving the registered semantic
    /// contract in the erased IR.
    #[must_use]
    pub fn into_node(self, id: MissionNodeId, name: MissionNodeName) -> MissionNode {
        MissionNode::new(id, name, MissionNodeKind::Action(Box::new(self.action)))
    }
}

/// Immutable node semantics after construction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum MissionNodeKind {
    /// Registered leaf action.
    Action(Box<MissionAction>),
    /// Ordered all-success composition.
    Sequence(MissionChildren),
    /// Ordered first-success composition.
    Selector(MissionChildren),
    /// Concurrent all-success composition.
    ParallelAll(MissionChildren),
    /// Concurrent first-success composition.
    ParallelAny(MissionChildren),
    /// Finite deadline wrapper.
    Timeout {
        /// Wrapped child.
        child: MissionNodeId,
        /// Nonzero deadline relative to activation.
        timeout_ns: u64,
    },
    /// Finite retry wrapper.
    Retry {
        /// Wrapped child.
        child: MissionNodeId,
        /// `None` represents an invalid unbounded policy rejected by validation.
        max_attempts: Option<u16>,
    },
    /// Finite loop wrapper.
    Loop {
        /// Wrapped child.
        child: MissionNodeId,
        /// `None` represents an invalid unbounded policy rejected by validation.
        max_iterations: Option<u16>,
    },
    /// Correlated conditional branch.
    Guard {
        /// Registered condition.
        guard_id: GuardId,
        /// Child activated for true.
        if_true: MissionNodeId,
        /// Child activated for false.
        if_false: MissionNodeId,
    },
    /// Named cancellation boundary.
    CancelScope {
        /// Scope selected by runtime cancellation events.
        scope_id: CancelScopeId,
        /// Scoped child.
        child: MissionNodeId,
    },
    /// Named reusable subflow call.
    Subflow {
        /// Referenced subflow root.
        child: MissionNodeId,
    },
}

impl MissionNodeKind {
    pub(crate) fn child_count(&self) -> usize {
        match self {
            Self::Action(_) => 0,
            Self::Sequence(children)
            | Self::Selector(children)
            | Self::ParallelAll(children)
            | Self::ParallelAny(children) => children.as_slice().len(),
            Self::Timeout { .. }
            | Self::Retry { .. }
            | Self::Loop { .. }
            | Self::CancelScope { .. }
            | Self::Subflow { .. } => 1,
            Self::Guard { .. } => 2,
        }
    }

    pub(crate) fn child_at(&self, index: usize) -> Option<MissionNodeId> {
        match self {
            Self::Action(_) => None,
            Self::Sequence(children)
            | Self::Selector(children)
            | Self::ParallelAll(children)
            | Self::ParallelAny(children) => children.as_slice().get(index).copied(),
            Self::Timeout { child, .. }
            | Self::Retry { child, .. }
            | Self::Loop { child, .. }
            | Self::CancelScope { child, .. }
            | Self::Subflow { child } => (index == 0).then_some(*child),
            Self::Guard {
                if_true, if_false, ..
            } => match index {
                0 => Some(*if_true),
                1 => Some(*if_false),
                _ => None,
            },
        }
    }
}

/// Named mission node.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionNode {
    id: MissionNodeId,
    name: MissionNodeName,
    kind: MissionNodeKind,
}

impl MissionNode {
    /// Constructs one node for a bounded builder.
    #[must_use]
    pub const fn new(id: MissionNodeId, name: MissionNodeName, kind: MissionNodeKind) -> Self {
        Self { id, name, kind }
    }

    /// Returns the stable node identity.
    #[must_use]
    pub const fn id(&self) -> MissionNodeId {
        self.id
    }

    /// Returns the stable node name.
    #[must_use]
    pub const fn name(&self) -> MissionNodeName {
        self.name
    }

    /// Returns immutable node semantics.
    #[must_use]
    pub const fn kind(&self) -> &MissionNodeKind {
        &self.kind
    }
}

/// Fixed registry used to reject unknown action and guard identities.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct MissionRegistry {
    actions: ArrayVec<ActionName, MAX_REGISTERED_ACTIONS>,
    guards: ArrayVec<GuardId, MAX_REGISTERED_GUARDS>,
}

impl MissionRegistry {
    /// Creates an empty registry.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            actions: ArrayVec::new_const(),
            guards: ArrayVec::new_const(),
        }
    }

    /// Registers one unique action identity.
    pub fn register_action(&mut self, action: ActionName) -> Result<(), MissionBuildError> {
        if self.actions.contains(&action) {
            return Err(MissionBuildError::DuplicateRegisteredAction);
        }
        self.actions.try_push(action).map_err(|_| {
            MissionBuildError::ActionRegistryCapacityExceeded {
                maximum: MAX_REGISTERED_ACTIONS,
            }
        })
    }

    /// Registers one unique guard identity.
    pub fn register_guard(&mut self, guard: GuardId) -> Result<(), MissionBuildError> {
        if self.guards.contains(&guard) {
            return Err(MissionBuildError::DuplicateRegisteredGuard);
        }
        self.guards
            .try_push(guard)
            .map_err(|_| MissionBuildError::GuardRegistryCapacityExceeded {
                maximum: MAX_REGISTERED_GUARDS,
            })
    }

    fn knows_action(&self, action: ActionName) -> bool {
        self.actions.contains(&action)
    }

    fn knows_guard(&self, guard: GuardId) -> bool {
        self.guards.contains(&guard)
    }
}

/// Mutable startup-only builder consumed by validation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionBuilder {
    root: MissionNodeId,
    nodes: ArrayVec<MissionNode, MAX_MISSION_NODES>,
}

impl MissionBuilder {
    /// Creates an empty builder with an explicit intended root.
    #[must_use]
    pub const fn new(root: MissionNodeId) -> Self {
        Self {
            root,
            nodes: ArrayVec::new_const(),
        }
    }

    /// Adds one node within the fixed mission capacity.
    pub fn add_node(&mut self, node: MissionNode) -> Result<(), MissionBuildError> {
        self.nodes
            .try_push(node)
            .map_err(|_| MissionBuildError::NodeCapacityExceeded {
                maximum: MAX_MISSION_NODES,
            })
    }

    /// Consumes the builder and returns an immutable validated graph.
    pub fn validate(self, registry: &MissionRegistry) -> Result<MissionIr, MissionBuildError> {
        validate_nodes(self.root, &self.nodes, registry)?;
        Ok(MissionIr {
            root: self.root,
            nodes: self.nodes,
        })
    }
}

/// Immutable validated mission graph.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MissionIr {
    root: MissionNodeId,
    nodes: ArrayVec<MissionNode, MAX_MISSION_NODES>,
}

impl MissionIr {
    /// Returns the root node.
    #[must_use]
    pub const fn root(&self) -> MissionNodeId {
        self.root
    }

    /// Returns nodes in deterministic builder order.
    #[must_use]
    pub fn nodes(&self) -> &[MissionNode] {
        self.nodes.as_slice()
    }

    /// Resolves one stable node identity.
    #[must_use]
    pub fn node(&self, id: MissionNodeId) -> Option<&MissionNode> {
        self.nodes.iter().find(|node| node.id == id)
    }

    pub(crate) fn node_index(&self, id: MissionNodeId) -> Option<usize> {
        self.nodes.iter().position(|node| node.id == id)
    }
}

/// Typed mission construction/validation failure.
#[derive(Clone, Debug, Eq, Error, PartialEq)]
pub enum MissionBuildError {
    /// Node capacity was exceeded.
    #[error("mission node capacity exceeds {maximum}")]
    NodeCapacityExceeded {
        /// Configured maximum.
        maximum: usize,
    },
    /// Direct-child capacity was exceeded.
    #[error("mission child capacity exceeds {maximum}")]
    ChildCapacityExceeded {
        /// Configured maximum.
        maximum: usize,
    },
    /// Per-action resource capacity was exceeded.
    #[error("action resource capacity exceeds {maximum}")]
    ResourceCapacityExceeded {
        /// Configured maximum.
        maximum: usize,
    },
    /// Action registry capacity was exceeded.
    #[error("action registry capacity exceeds {maximum}")]
    ActionRegistryCapacityExceeded {
        /// Configured maximum.
        maximum: usize,
    },
    /// Guard registry capacity was exceeded.
    #[error("guard registry capacity exceeds {maximum}")]
    GuardRegistryCapacityExceeded {
        /// Configured maximum.
        maximum: usize,
    },
    /// The mission contained no nodes.
    #[error("mission contains no nodes")]
    EmptyMission,
    /// Root identity did not resolve.
    #[error("mission root {0:?} does not resolve")]
    MissingRoot(MissionNodeId),
    /// Node identity was duplicated.
    #[error("mission node ID {0:?} is duplicated")]
    DuplicateNodeId(MissionNodeId),
    /// Node name was duplicated.
    #[error("mission node name {0} is duplicated")]
    DuplicateNodeName(MissionNodeName),
    /// Registered action identity was duplicated.
    #[error("registered action identity is duplicated")]
    DuplicateRegisteredAction,
    /// Registered guard identity was duplicated.
    #[error("registered guard identity is duplicated")]
    DuplicateRegisteredGuard,
    /// Node referenced an unknown child.
    #[error("mission node {node:?} references unknown child {child:?}")]
    UnknownChild {
        /// Referring node.
        node: MissionNodeId,
        /// Missing child.
        child: MissionNodeId,
    },
    /// Action was not registered.
    #[error("mission action node {0:?} is not registered")]
    UnknownAction(MissionNodeId),
    /// Guard was not registered.
    #[error("mission guard node {0:?} is not registered")]
    UnknownGuard(MissionNodeId),
    /// Composite node had no children.
    #[error("mission composite {0:?} has no children")]
    EmptyComposite(MissionNodeId),
    /// One child was repeated inside the same direct composite.
    #[error("mission composite {node:?} repeats child {child:?}")]
    DuplicateChild {
        /// Composite node.
        node: MissionNodeId,
        /// Repeated child.
        child: MissionNodeId,
    },
    /// Action repeated an exclusive resource.
    #[error("mission action {node:?} repeats resource {resource}")]
    DuplicateActionResource {
        /// Action node.
        node: MissionNodeId,
        /// Repeated resource.
        resource: ResourceId,
    },
    /// Action had no finite timeout.
    #[error("mission action {0:?} requires a nonzero timeout")]
    MissingActionTimeout(MissionNodeId),
    /// Timeout wrapper had no finite timeout.
    #[error("mission timeout {0:?} requires a nonzero duration")]
    MissingWrapperTimeout(MissionNodeId),
    /// Retry policy was absent or zero.
    #[error("mission retry {0:?} is unbounded or zero")]
    UnboundedRetry(MissionNodeId),
    /// Loop policy was absent or zero.
    #[error("mission loop {0:?} is unbounded or zero")]
    UnboundedLoop(MissionNodeId),
    /// Cancellation scope identity was duplicated.
    #[error("mission cancellation scope {0} is duplicated")]
    DuplicateCancelScope(CancelScopeId),
    /// Graph contains a structural cycle.
    #[error("mission graph contains a cycle at {0:?}")]
    Cycle(MissionNodeId),
    /// Node is not reachable from the root.
    #[error("mission node {0:?} is unreachable")]
    UnreachableNode(MissionNodeId),
    /// Parallel branches request the same exclusive resource.
    #[error("parallel node {node:?} conflicts on resource {resource}")]
    ParallelResourceConflict {
        /// Parallel composite.
        node: MissionNodeId,
        /// Conflicting resource.
        resource: ResourceId,
    },
    /// Worst-case graph execution exceeds retained command correlation history.
    #[error("mission requires {required} action invocations, maximum is {maximum}")]
    InvocationBudgetExceeded {
        /// Worst-case action invocations.
        required: u64,
        /// Runtime maximum.
        maximum: usize,
    },
    /// Worst-case graph execution exceeds simultaneous activation slots.
    #[error("mission requires {required} active frames, maximum is {maximum}")]
    ActivationBudgetExceeded {
        /// Worst-case simultaneous frames.
        required: u64,
        /// Runtime maximum.
        maximum: usize,
    },
}

fn validate_nodes(
    root: MissionNodeId,
    nodes: &[MissionNode],
    registry: &MissionRegistry,
) -> Result<(), MissionBuildError> {
    if nodes.is_empty() {
        return Err(MissionBuildError::EmptyMission);
    }
    let root_index = find_node_index(nodes, root).ok_or(MissionBuildError::MissingRoot(root))?;

    for (index, node) in nodes.iter().enumerate() {
        for previous in &nodes[..index] {
            if previous.id == node.id {
                return Err(MissionBuildError::DuplicateNodeId(node.id));
            }
            if previous.name == node.name {
                return Err(MissionBuildError::DuplicateNodeName(node.name));
            }
        }
        validate_node(node, nodes, registry)?;
    }
    validate_unique_scopes(nodes)?;

    let mut marks = [0_u8; MAX_MISSION_NODES];
    visit_node(root_index, nodes, &mut marks)?;
    for (index, mark) in marks[..nodes.len()].iter().enumerate() {
        if *mark == 0 {
            return Err(MissionBuildError::UnreachableNode(nodes[index].id));
        }
    }

    validate_runtime_budgets(root_index, nodes)?;

    for node in nodes {
        if matches!(
            node.kind,
            MissionNodeKind::ParallelAll(_) | MissionNodeKind::ParallelAny(_)
        ) {
            validate_parallel_resources(node, nodes)?;
        }
    }
    Ok(())
}

fn validate_runtime_budgets(
    root_index: usize,
    nodes: &[MissionNode],
) -> Result<(), MissionBuildError> {
    let mut invocation_memo = [None; MAX_MISSION_NODES];
    let invocations = maximum_action_invocations(root_index, nodes, &mut invocation_memo);
    if invocations > MAX_MISSION_ACTION_INVOCATIONS as u64 {
        return Err(MissionBuildError::InvocationBudgetExceeded {
            required: invocations,
            maximum: MAX_MISSION_ACTION_INVOCATIONS,
        });
    }

    let mut activation_memo = [None; MAX_MISSION_NODES];
    let activations = maximum_active_frames(root_index, nodes, &mut activation_memo);
    if activations > MAX_MISSION_ACTIVE_ACTIVATIONS as u64 {
        return Err(MissionBuildError::ActivationBudgetExceeded {
            required: activations,
            maximum: MAX_MISSION_ACTIVE_ACTIVATIONS,
        });
    }
    Ok(())
}

fn maximum_action_invocations(
    index: usize,
    nodes: &[MissionNode],
    memo: &mut [Option<u64>; MAX_MISSION_NODES],
) -> u64 {
    if let Some(value) = memo[index] {
        return value;
    }
    let kind = &nodes[index].kind;
    let child_invocations = |child: MissionNodeId, memo: &mut [Option<u64>; MAX_MISSION_NODES]| {
        maximum_action_invocations(
            find_node_index(nodes, child).expect("validated child"),
            nodes,
            memo,
        )
    };
    let value = match kind {
        MissionNodeKind::Action(_) => 1,
        MissionNodeKind::Sequence(children)
        | MissionNodeKind::Selector(children)
        | MissionNodeKind::ParallelAll(children)
        | MissionNodeKind::ParallelAny(children) => {
            children.as_slice().iter().fold(0_u64, |total, child| {
                total.saturating_add(child_invocations(*child, memo))
            })
        }
        MissionNodeKind::Timeout { child, .. }
        | MissionNodeKind::CancelScope { child, .. }
        | MissionNodeKind::Subflow { child } => child_invocations(*child, memo),
        MissionNodeKind::Retry {
            child,
            max_attempts,
        } => child_invocations(*child, memo)
            .saturating_mul(u64::from(max_attempts.expect("validated retry"))),
        MissionNodeKind::Loop {
            child,
            max_iterations,
        } => child_invocations(*child, memo)
            .saturating_mul(u64::from(max_iterations.expect("validated loop"))),
        MissionNodeKind::Guard {
            if_true, if_false, ..
        } => child_invocations(*if_true, memo).max(child_invocations(*if_false, memo)),
    };
    memo[index] = Some(value);
    value
}

fn maximum_active_frames(
    index: usize,
    nodes: &[MissionNode],
    memo: &mut [Option<u64>; MAX_MISSION_NODES],
) -> u64 {
    if let Some(value) = memo[index] {
        return value;
    }
    let kind = &nodes[index].kind;
    let child_frames = |child: MissionNodeId, memo: &mut [Option<u64>; MAX_MISSION_NODES]| {
        maximum_active_frames(
            find_node_index(nodes, child).expect("validated child"),
            nodes,
            memo,
        )
    };
    let descendants = match kind {
        MissionNodeKind::Action(_) => 0,
        MissionNodeKind::Sequence(children) | MissionNodeKind::Selector(children) => children
            .as_slice()
            .iter()
            .map(|child| child_frames(*child, memo))
            .max()
            .unwrap_or(0),
        MissionNodeKind::ParallelAll(children) | MissionNodeKind::ParallelAny(children) => {
            children.as_slice().iter().fold(0_u64, |total, child| {
                total.saturating_add(child_frames(*child, memo))
            })
        }
        MissionNodeKind::Timeout { child, .. }
        | MissionNodeKind::Retry { child, .. }
        | MissionNodeKind::Loop { child, .. }
        | MissionNodeKind::CancelScope { child, .. }
        | MissionNodeKind::Subflow { child } => child_frames(*child, memo),
        MissionNodeKind::Guard {
            if_true, if_false, ..
        } => child_frames(*if_true, memo).max(child_frames(*if_false, memo)),
    };
    let value = 1_u64.saturating_add(descendants);
    memo[index] = Some(value);
    value
}

fn validate_node(
    node: &MissionNode,
    nodes: &[MissionNode],
    registry: &MissionRegistry,
) -> Result<(), MissionBuildError> {
    match &node.kind {
        MissionNodeKind::Action(action) => {
            if !registry.knows_action(action.action_name) {
                return Err(MissionBuildError::UnknownAction(node.id));
            }
            if action.timeout_ns == 0 {
                return Err(MissionBuildError::MissingActionTimeout(node.id));
            }
            for (index, resource) in action.resources.as_slice().iter().enumerate() {
                if action.resources.as_slice()[..index].contains(resource) {
                    return Err(MissionBuildError::DuplicateActionResource {
                        node: node.id,
                        resource: *resource,
                    });
                }
            }
        }
        MissionNodeKind::Sequence(children)
        | MissionNodeKind::Selector(children)
        | MissionNodeKind::ParallelAll(children)
        | MissionNodeKind::ParallelAny(children) => {
            if children.as_slice().is_empty() {
                return Err(MissionBuildError::EmptyComposite(node.id));
            }
            for (index, child) in children.as_slice().iter().enumerate() {
                if children.as_slice()[..index].contains(child) {
                    return Err(MissionBuildError::DuplicateChild {
                        node: node.id,
                        child: *child,
                    });
                }
            }
        }
        MissionNodeKind::Timeout { timeout_ns, .. } if *timeout_ns == 0 => {
            return Err(MissionBuildError::MissingWrapperTimeout(node.id));
        }
        MissionNodeKind::Retry { max_attempts, .. }
            if max_attempts.is_none_or(|attempts| attempts == 0) =>
        {
            return Err(MissionBuildError::UnboundedRetry(node.id));
        }
        MissionNodeKind::Loop { max_iterations, .. }
            if max_iterations.is_none_or(|iterations| iterations == 0) =>
        {
            return Err(MissionBuildError::UnboundedLoop(node.id));
        }
        MissionNodeKind::Guard { guard_id, .. } if !registry.knows_guard(*guard_id) => {
            return Err(MissionBuildError::UnknownGuard(node.id));
        }
        MissionNodeKind::Timeout { .. }
        | MissionNodeKind::Retry { .. }
        | MissionNodeKind::Loop { .. }
        | MissionNodeKind::Guard { .. }
        | MissionNodeKind::CancelScope { .. }
        | MissionNodeKind::Subflow { .. } => {}
    }

    for child_index in 0..node.kind.child_count() {
        let child = node
            .kind
            .child_at(child_index)
            .expect("child_count and child_at are consistent");
        if find_node_index(nodes, child).is_none() {
            return Err(MissionBuildError::UnknownChild {
                node: node.id,
                child,
            });
        }
    }
    Ok(())
}

fn validate_unique_scopes(nodes: &[MissionNode]) -> Result<(), MissionBuildError> {
    for (index, node) in nodes.iter().enumerate() {
        let MissionNodeKind::CancelScope { scope_id, .. } = node.kind else {
            continue;
        };
        if nodes[..index].iter().any(|previous| {
            matches!(
                previous.kind,
                MissionNodeKind::CancelScope {
                    scope_id: previous_scope,
                    ..
                } if previous_scope == scope_id
            )
        }) {
            return Err(MissionBuildError::DuplicateCancelScope(scope_id));
        }
    }
    Ok(())
}

fn visit_node(
    index: usize,
    nodes: &[MissionNode],
    marks: &mut [u8; MAX_MISSION_NODES],
) -> Result<(), MissionBuildError> {
    if marks[index] == 1 {
        return Err(MissionBuildError::Cycle(nodes[index].id));
    }
    if marks[index] == 2 {
        return Ok(());
    }
    marks[index] = 1;
    for child_index in 0..nodes[index].kind.child_count() {
        let child = nodes[index]
            .kind
            .child_at(child_index)
            .expect("child_count and child_at are consistent");
        let resolved = find_node_index(nodes, child).expect("references validated before DFS");
        visit_node(resolved, nodes, marks)?;
    }
    marks[index] = 2;
    Ok(())
}

fn validate_parallel_resources(
    parallel: &MissionNode,
    nodes: &[MissionNode],
) -> Result<(), MissionBuildError> {
    let mut prior = ArrayVec::<ResourceId, MAX_RESOURCE_REFERENCES>::new();
    for child_index in 0..parallel.kind.child_count() {
        let child = parallel
            .kind
            .child_at(child_index)
            .expect("child_count and child_at are consistent");
        let mut branch = ArrayVec::<ResourceId, MAX_RESOURCE_REFERENCES>::new();
        collect_resources(child, nodes, &mut branch);
        for resource in branch {
            if prior.contains(&resource) {
                return Err(MissionBuildError::ParallelResourceConflict {
                    node: parallel.id,
                    resource,
                });
            }
            if !prior.contains(&resource) {
                prior
                    .try_push(resource)
                    .expect("resource reference bound follows node/action bounds");
            }
        }
    }
    Ok(())
}

fn collect_resources(
    node_id: MissionNodeId,
    nodes: &[MissionNode],
    resources: &mut ArrayVec<ResourceId, MAX_RESOURCE_REFERENCES>,
) {
    let node = &nodes[find_node_index(nodes, node_id).expect("validated node reference")];
    if let MissionNodeKind::Action(action) = &node.kind {
        for resource in action.resources.as_slice() {
            if !resources.contains(resource) {
                resources
                    .try_push(*resource)
                    .expect("resource reference bound follows node/action bounds");
            }
        }
    }
    for index in 0..node.kind.child_count() {
        collect_resources(
            node.kind
                .child_at(index)
                .expect("child_count and child_at are consistent"),
            nodes,
            resources,
        );
    }
}

fn find_node_index(nodes: &[MissionNode], id: MissionNodeId) -> Option<usize> {
    nodes.iter().position(|node| node.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Goal;
    struct ResultPayload;

    fn name(value: &str) -> MissionNodeName {
        MissionNodeName::parse(value).unwrap()
    }

    fn action_name(value: &str) -> ActionName {
        ActionName::parse(value).unwrap()
    }

    fn resource(value: &str) -> ResourceId {
        ResourceId::parse(value).unwrap()
    }

    fn registry() -> MissionRegistry {
        let mut registry = MissionRegistry::new();
        for action in ["move", "inspect", "dock"] {
            registry.register_action(action_name(action)).unwrap();
        }
        registry
            .register_guard(GuardId::parse("path_clear").unwrap())
            .unwrap();
        registry
    }

    fn action(id: u16, action: &str, resources: &[ResourceId], timeout_ns: u64) -> MissionNode {
        TypedAction::<Goal, ResultPayload>::new(
            action_name(action),
            MissionResources::try_from_slice(resources).unwrap(),
            timeout_ns,
        )
        .into_node(MissionNodeId::new(id), name(&format!("node_{id}")))
    }

    fn representative_builder() -> MissionBuilder {
        let move_resource = resource("chassis");
        let camera_resource = resource("camera");
        let mut builder = MissionBuilder::new(MissionNodeId::new(0));
        let nodes = [
            MissionNode::new(
                MissionNodeId::new(0),
                name("root"),
                MissionNodeKind::Sequence(
                    MissionChildren::try_from_slice(&[
                        MissionNodeId::new(1),
                        MissionNodeId::new(2),
                        MissionNodeId::new(3),
                    ])
                    .unwrap(),
                ),
            ),
            MissionNode::new(
                MissionNodeId::new(1),
                name("guard"),
                MissionNodeKind::Guard {
                    guard_id: GuardId::parse("path_clear").unwrap(),
                    if_true: MissionNodeId::new(4),
                    if_false: MissionNodeId::new(5),
                },
            ),
            MissionNode::new(
                MissionNodeId::new(2),
                name("parallel"),
                MissionNodeKind::ParallelAll(
                    MissionChildren::try_from_slice(&[
                        MissionNodeId::new(6),
                        MissionNodeId::new(7),
                    ])
                    .unwrap(),
                ),
            ),
            MissionNode::new(
                MissionNodeId::new(3),
                name("scope"),
                MissionNodeKind::CancelScope {
                    scope_id: CancelScopeId::parse("dock_scope").unwrap(),
                    child: MissionNodeId::new(8),
                },
            ),
            action(4, "move", &[move_resource], 1_000),
            action(5, "dock", &[move_resource], 1_000),
            MissionNode::new(
                MissionNodeId::new(6),
                name("retry"),
                MissionNodeKind::Retry {
                    child: MissionNodeId::new(9),
                    max_attempts: Some(3),
                },
            ),
            action(7, "inspect", &[camera_resource], 1_000),
            MissionNode::new(
                MissionNodeId::new(8),
                name("timeout"),
                MissionNodeKind::Timeout {
                    child: MissionNodeId::new(10),
                    timeout_ns: 2_000,
                },
            ),
            action(9, "move", &[move_resource], 1_000),
            MissionNode::new(
                MissionNodeId::new(10),
                name("loop"),
                MissionNodeKind::Loop {
                    child: MissionNodeId::new(11),
                    max_iterations: Some(2),
                },
            ),
            action(11, "dock", &[move_resource], 1_000),
        ];
        for node in nodes {
            builder.add_node(node).unwrap();
        }
        builder
    }

    #[test]
    fn representative_typed_graph_validates_into_immutable_ir() {
        let ir = representative_builder().validate(&registry()).unwrap();
        assert_eq!(ir.root(), MissionNodeId::new(0));
        assert_eq!(ir.nodes().len(), 12);
        assert!(matches!(
            ir.node(MissionNodeId::new(6)).unwrap().kind(),
            MissionNodeKind::Retry {
                max_attempts: Some(3),
                ..
            }
        ));
    }

    #[test]
    fn identities_references_reachability_and_cycles_are_rejected() {
        let registry = registry();

        let mut duplicate = MissionBuilder::new(MissionNodeId::new(1));
        duplicate.add_node(action(1, "move", &[], 1)).unwrap();
        duplicate
            .add_node(MissionNode::new(
                MissionNodeId::new(1),
                name("other"),
                MissionNodeKind::Subflow {
                    child: MissionNodeId::new(1),
                },
            ))
            .unwrap();
        assert!(matches!(
            duplicate.validate(&registry),
            Err(MissionBuildError::DuplicateNodeId(MissionNodeId(1)))
        ));

        let mut missing = MissionBuilder::new(MissionNodeId::new(1));
        missing
            .add_node(MissionNode::new(
                MissionNodeId::new(1),
                name("missing"),
                MissionNodeKind::Subflow {
                    child: MissionNodeId::new(2),
                },
            ))
            .unwrap();
        assert!(matches!(
            missing.validate(&registry),
            Err(MissionBuildError::UnknownChild { .. })
        ));

        let mut unreachable = MissionBuilder::new(MissionNodeId::new(1));
        unreachable.add_node(action(1, "move", &[], 1)).unwrap();
        unreachable.add_node(action(2, "move", &[], 1)).unwrap();
        assert_eq!(
            unreachable.validate(&registry),
            Err(MissionBuildError::UnreachableNode(MissionNodeId::new(2)))
        );

        let mut cycle = MissionBuilder::new(MissionNodeId::new(1));
        cycle
            .add_node(MissionNode::new(
                MissionNodeId::new(1),
                name("cycle_a"),
                MissionNodeKind::Subflow {
                    child: MissionNodeId::new(2),
                },
            ))
            .unwrap();
        cycle
            .add_node(MissionNode::new(
                MissionNodeId::new(2),
                name("cycle_b"),
                MissionNodeKind::Subflow {
                    child: MissionNodeId::new(1),
                },
            ))
            .unwrap();
        assert!(matches!(
            cycle.validate(&registry),
            Err(MissionBuildError::Cycle(_))
        ));
    }

    #[test]
    fn unknown_empty_timeout_and_unbounded_policy_are_rejected() {
        let registry = registry();
        assert_eq!(
            MissionBuilder::new(MissionNodeId::new(0)).validate(&registry),
            Err(MissionBuildError::EmptyMission)
        );

        let cases = [
            action(1, "unknown", &[], 1),
            action(1, "move", &[], 0),
            MissionNode::new(
                MissionNodeId::new(1),
                name("empty"),
                MissionNodeKind::Sequence(MissionChildren::try_from_slice(&[]).unwrap()),
            ),
            MissionNode::new(
                MissionNodeId::new(1),
                name("timeout"),
                MissionNodeKind::Timeout {
                    child: MissionNodeId::new(2),
                    timeout_ns: 0,
                },
            ),
            MissionNode::new(
                MissionNodeId::new(1),
                name("retry"),
                MissionNodeKind::Retry {
                    child: MissionNodeId::new(2),
                    max_attempts: None,
                },
            ),
            MissionNode::new(
                MissionNodeId::new(1),
                name("loop"),
                MissionNodeKind::Loop {
                    child: MissionNodeId::new(2),
                    max_iterations: Some(0),
                },
            ),
        ];
        for (index, node) in cases.into_iter().enumerate() {
            let mut builder = MissionBuilder::new(MissionNodeId::new(1));
            builder.add_node(node).unwrap();
            if index >= 3 {
                builder.add_node(action(2, "move", &[], 1)).unwrap();
            }
            assert!(builder.validate(&registry).is_err(), "case {index}");
        }
    }

    #[test]
    fn transitive_parallel_resource_conflict_is_rejected() {
        let chassis = resource("chassis");
        let mut builder = MissionBuilder::new(MissionNodeId::new(0));
        for node in [
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
                name("nested"),
                MissionNodeKind::Sequence(
                    MissionChildren::try_from_slice(&[MissionNodeId::new(3)]).unwrap(),
                ),
            ),
            action(2, "move", &[chassis], 1),
            action(3, "dock", &[chassis], 1),
        ] {
            builder.add_node(node).unwrap();
        }
        assert_eq!(
            builder.validate(&registry()),
            Err(MissionBuildError::ParallelResourceConflict {
                node: MissionNodeId::new(0),
                resource: chassis
            })
        );
    }

    #[test]
    fn duplicate_resources_children_scopes_and_registry_entries_are_rejected() {
        let chassis = resource("chassis");
        let registry = registry();

        let mut duplicate_resource = MissionBuilder::new(MissionNodeId::new(1));
        duplicate_resource
            .add_node(action(1, "move", &[chassis, chassis], 1))
            .unwrap();
        assert!(matches!(
            duplicate_resource.validate(&registry),
            Err(MissionBuildError::DuplicateActionResource { .. })
        ));

        let mut duplicate_child = MissionBuilder::new(MissionNodeId::new(1));
        duplicate_child
            .add_node(MissionNode::new(
                MissionNodeId::new(1),
                name("sequence"),
                MissionNodeKind::Sequence(
                    MissionChildren::try_from_slice(&[
                        MissionNodeId::new(2),
                        MissionNodeId::new(2),
                    ])
                    .unwrap(),
                ),
            ))
            .unwrap();
        duplicate_child.add_node(action(2, "move", &[], 1)).unwrap();
        assert!(matches!(
            duplicate_child.validate(&registry),
            Err(MissionBuildError::DuplicateChild { .. })
        ));

        let mut duplicate_scope = MissionBuilder::new(MissionNodeId::new(0));
        for node in [
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
            MissionNode::new(
                MissionNodeId::new(1),
                name("scope_a"),
                MissionNodeKind::CancelScope {
                    scope_id: CancelScopeId::parse("same").unwrap(),
                    child: MissionNodeId::new(3),
                },
            ),
            MissionNode::new(
                MissionNodeId::new(2),
                name("scope_b"),
                MissionNodeKind::CancelScope {
                    scope_id: CancelScopeId::parse("same").unwrap(),
                    child: MissionNodeId::new(4),
                },
            ),
            action(3, "move", &[], 1),
            action(4, "inspect", &[], 1),
        ] {
            duplicate_scope.add_node(node).unwrap();
        }
        assert!(matches!(
            duplicate_scope.validate(&registry),
            Err(MissionBuildError::DuplicateCancelScope(_))
        ));

        let mut duplicate_registry = MissionRegistry::new();
        duplicate_registry
            .register_action(action_name("move"))
            .unwrap();
        assert!(matches!(
            duplicate_registry.register_action(action_name("move")),
            Err(MissionBuildError::DuplicateRegisteredAction)
        ));
    }

    #[test]
    fn worst_case_invocation_and_activation_budgets_are_validated() {
        let mut invocation = MissionBuilder::new(MissionNodeId::new(0));
        invocation
            .add_node(MissionNode::new(
                MissionNodeId::new(0),
                name("too_many_retries"),
                MissionNodeKind::Retry {
                    child: MissionNodeId::new(1),
                    max_attempts: Some(257),
                },
            ))
            .unwrap();
        invocation.add_node(action(1, "move", &[], 1)).unwrap();
        assert_eq!(
            invocation.validate(&registry()),
            Err(MissionBuildError::InvocationBudgetExceeded {
                required: 257,
                maximum: MAX_MISSION_ACTION_INVOCATIONS
            })
        );

        let root_children = (1_u16..=8)
            .map(MissionNodeId::new)
            .collect::<ArrayVec<_, 8>>();
        let shared_children = (10_u16..=17)
            .map(MissionNodeId::new)
            .collect::<ArrayVec<_, 8>>();
        let mut activation = MissionBuilder::new(MissionNodeId::new(0));
        activation
            .add_node(MissionNode::new(
                MissionNodeId::new(0),
                name("wide_root"),
                MissionNodeKind::ParallelAll(
                    MissionChildren::try_from_slice(root_children.as_slice()).unwrap(),
                ),
            ))
            .unwrap();
        for id in 1_u16..=8 {
            activation
                .add_node(MissionNode::new(
                    MissionNodeId::new(id),
                    name(&format!("call_{id}")),
                    MissionNodeKind::Subflow {
                        child: MissionNodeId::new(9),
                    },
                ))
                .unwrap();
        }
        activation
            .add_node(MissionNode::new(
                MissionNodeId::new(9),
                name("shared_parallel"),
                MissionNodeKind::ParallelAll(
                    MissionChildren::try_from_slice(shared_children.as_slice()).unwrap(),
                ),
            ))
            .unwrap();
        for id in 10_u16..=17 {
            activation.add_node(action(id, "move", &[], 1)).unwrap();
        }
        assert_eq!(
            activation.validate(&registry()),
            Err(MissionBuildError::ActivationBudgetExceeded {
                required: 81,
                maximum: MAX_MISSION_ACTIVE_ACTIVATIONS
            })
        );
    }
}
