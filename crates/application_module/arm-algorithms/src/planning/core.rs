//! Allocation-bounded primitives shared by the named planning kernels.

use std::{cmp::Ordering, collections::BinaryHeap, mem::size_of};

use message_model::{
    ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId, MAX_JOINTS,
    MAX_TRAJECTORY_POINTS,
};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

use crate::{ArmAlgorithmError, kinematics::PreparedKinematics};

use super::{ArmStateValidator, PlannerConfig};

const NO_PARENT: usize = usize::MAX;

#[derive(Clone, Copy, Debug)]
pub(crate) struct State {
    values: [f64; MAX_JOINTS],
}

impl State {
    pub(crate) const fn zero() -> Self {
        Self {
            values: [0.0; MAX_JOINTS],
        }
    }

    pub(crate) fn as_slice(&self, dof: usize) -> &[f64] {
        &self.values[..dof]
    }

    pub(crate) fn as_mut_slice(&mut self, dof: usize) -> &mut [f64] {
        &mut self.values[..dof]
    }
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct Node {
    pub(crate) state: State,
    pub(crate) parent: usize,
    pub(crate) cost: f64,
}

impl Node {
    pub(crate) const fn root(state: State) -> Self {
        Self {
            state,
            parent: NO_PARENT,
            cost: 0.0,
        }
    }

    pub(crate) const fn child(state: State, parent: usize, cost: f64) -> Self {
        Self {
            state,
            parent,
            cost,
        }
    }

    pub(crate) const fn parent(self) -> Option<usize> {
        if self.parent == NO_PARENT {
            None
        } else {
            Some(self.parent)
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ProjectionCell {
    pub(crate) key: [i32; 2],
    pub(crate) selections: u32,
    pub(crate) successes: u32,
    pub(crate) failures: u32,
    pub(crate) occupied_neighbors: u8,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct OpenEntry {
    pub(crate) node: usize,
    pub(crate) estimate: f64,
}

impl PartialEq for OpenEntry {
    fn eq(&self, other: &Self) -> bool {
        self.node == other.node && self.estimate.to_bits() == other.estimate.to_bits()
    }
}

impl Eq for OpenEntry {}

impl PartialOrd for OpenEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OpenEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .estimate
            .total_cmp(&self.estimate)
            .then_with(|| other.node.cmp(&self.node))
    }
}

/// A planning context that borrows the project's single prepared-kinematics identity.
pub struct PreparedPlanning<'prepared, 'model> {
    pub(crate) kinematics: &'prepared PreparedKinematics<'model>,
    constraint_generation: u64,
}

impl<'prepared, 'model> PreparedPlanning<'prepared, 'model> {
    /// Binds semantic constraints to an existing prepared model generation.
    pub fn new(
        kinematics: &'prepared PreparedKinematics<'model>,
        constraint_generation: u64,
    ) -> Result<Self, ArmAlgorithmError> {
        if constraint_generation == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.constraint_generation",
            });
        }
        Ok(Self {
            kinematics,
            constraint_generation,
        })
    }

    /// Returns the immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.kinematics.profile_id()
    }

    /// Returns the selected kinematic group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.kinematics.group_id()
    }

    /// Returns stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        self.kinematics.order()
    }

    /// Returns the existing model generation.
    #[must_use]
    pub const fn model_generation(&self) -> u64 {
        self.kinematics.generation()
    }

    /// Returns the caller-managed constraint generation.
    #[must_use]
    pub const fn constraint_generation(&self) -> u64 {
        self.constraint_generation
    }

    /// Returns active degrees of freedom.
    #[must_use]
    pub fn dof(&self) -> usize {
        self.kinematics.dof()
    }

    /// Allocates all reusable planner scratch once.
    pub fn workspace(&self, config: PlannerConfig) -> Result<PlannerWorkspace, ArmAlgorithmError> {
        PlannerWorkspace::new(self, config)
    }

    /// Allocates planner scratch only when it fits a caller-provided byte ceiling.
    pub fn workspace_with_byte_limit(
        &self,
        config: PlannerConfig,
        maximum_bytes: usize,
    ) -> Result<PlannerWorkspace, ArmAlgorithmError> {
        PlannerWorkspace::with_byte_limit(self, config, maximum_bytes)
    }
}

/// Reusable, preparation-time allocated scratch for every sampling planner.
pub struct PlannerWorkspace {
    pub(crate) profile_id: ArmProfileId,
    pub(crate) group_id: KinematicGroupId,
    pub(crate) order: ArmJointOrder,
    pub(crate) model_generation: u64,
    pub(crate) constraint_generation: u64,
    pub(crate) dof: usize,
    pub(crate) maximum_nodes: usize,
    pub(crate) first: Vec<Node>,
    pub(crate) second: Vec<Node>,
    pub(crate) neighbors: Vec<usize>,
    pub(crate) density: Vec<u32>,
    pub(crate) cells: Vec<ProjectionCell>,
    pub(crate) path: Vec<State>,
    pub(crate) scores: Vec<f64>,
    pub(crate) parents: Vec<usize>,
    pub(crate) open: BinaryHeap<OpenEntry>,
    pub(crate) rng: ChaCha8Rng,
}

impl PlannerWorkspace {
    /// Allocates reusable storage once from the validated node ceiling.
    pub fn new(
        prepared: &PreparedPlanning<'_, '_>,
        config: PlannerConfig,
    ) -> Result<Self, ArmAlgorithmError> {
        let maximum_nodes = config.maximum_nodes as usize;
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            dof: prepared.dof(),
            maximum_nodes,
            first: Vec::with_capacity(maximum_nodes),
            second: Vec::with_capacity(maximum_nodes),
            neighbors: Vec::with_capacity(maximum_nodes),
            density: Vec::with_capacity(maximum_nodes),
            cells: Vec::with_capacity(maximum_nodes),
            path: Vec::with_capacity(MAX_TRAJECTORY_POINTS),
            scores: Vec::with_capacity(maximum_nodes),
            parents: Vec::with_capacity(maximum_nodes),
            open: BinaryHeap::with_capacity(maximum_nodes),
            rng: ChaCha8Rng::from_seed(config.seed),
        })
    }

    /// Returns a conservative preparation-time storage estimate.
    pub fn required_bytes(config: PlannerConfig) -> Result<usize, ArmAlgorithmError> {
        let nodes = config.maximum_nodes as usize;
        let per_node = size_of::<Node>()
            .checked_mul(2)
            .and_then(|value| value.checked_add(size_of::<usize>()))
            .and_then(|value| value.checked_add(size_of::<u32>()))
            .and_then(|value| value.checked_add(size_of::<ProjectionCell>()))
            .and_then(|value| value.checked_add(size_of::<f64>()))
            .and_then(|value| value.checked_add(size_of::<usize>()))
            .and_then(|value| value.checked_add(size_of::<OpenEntry>()))
            .ok_or(ArmAlgorithmError::InvalidBudget {
                field: "planner.workspace_bytes",
            })?;
        nodes
            .checked_mul(per_node)
            .and_then(|value| {
                value.checked_add(MAX_TRAJECTORY_POINTS.checked_mul(size_of::<State>())?)
            })
            .ok_or(ArmAlgorithmError::InvalidBudget {
                field: "planner.workspace_bytes",
            })
    }

    /// Returns the storage reserved during workspace preparation.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.first.capacity() * size_of::<Node>()
            + self.second.capacity() * size_of::<Node>()
            + self.neighbors.capacity() * size_of::<usize>()
            + self.density.capacity() * size_of::<u32>()
            + self.cells.capacity() * size_of::<ProjectionCell>()
            + self.path.capacity() * size_of::<State>()
            + self.scores.capacity() * size_of::<f64>()
            + self.parents.capacity() * size_of::<usize>()
            + self.open.capacity() * size_of::<OpenEntry>()
    }

    /// Allocates only when the storage estimate fits the caller's byte ceiling.
    pub fn with_byte_limit(
        prepared: &PreparedPlanning<'_, '_>,
        config: PlannerConfig,
        maximum_bytes: usize,
    ) -> Result<Self, ArmAlgorithmError> {
        let required = Self::required_bytes(config)?;
        if required > maximum_bytes {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.workspace_bytes",
                actual: required,
                maximum: maximum_bytes,
            });
        }
        Self::new(prepared, config)
    }

    pub(crate) fn reset(
        &mut self,
        prepared: &PreparedPlanning<'_, '_>,
        config: PlannerConfig,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || &self.order != prepared.order()
            || self.dof != prepared.dof()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "planner profile, group or joint order changed",
            });
        }
        if self.model_generation != prepared.model_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.model_generation,
                actual: prepared.model_generation(),
            });
        }
        if self.constraint_generation != prepared.constraint_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.constraint_generation,
                actual: prepared.constraint_generation(),
            });
        }
        if self.maximum_nodes < config.maximum_nodes as usize {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.workspace_nodes",
                actual: config.maximum_nodes as usize,
                maximum: self.maximum_nodes,
            });
        }
        self.first.clear();
        self.second.clear();
        self.neighbors.clear();
        self.density.clear();
        self.cells.clear();
        self.path.clear();
        self.scores.clear();
        self.parents.clear();
        self.open.clear();
        self.rng = ChaCha8Rng::from_seed(config.seed);
        Ok(())
    }
}

pub(crate) fn copy_position(
    prepared: &PreparedPlanning<'_, '_>,
    position: &ArmJointVector<ArmPosition>,
) -> Result<State, ArmAlgorithmError> {
    if position.order() != prepared.order() {
        return Err(ArmAlgorithmError::JointOrderMismatch {
            group: prepared.group_id(),
        });
    }
    let mut state = State::zero();
    for (index, (descriptor, value)) in prepared
        .order()
        .joints()
        .iter()
        .zip(position.values())
        .enumerate()
    {
        if descriptor.kind().coordinate_kind() != value.kind() {
            return Err(ArmAlgorithmError::UnitMismatch {
                joint: descriptor.joint_id(),
            });
        }
        state.values[index] = value.value_si();
    }
    prepared
        .kinematics
        .validate_values(state.as_slice(prepared.dof()))?;
    Ok(state)
}

pub(crate) fn validate_state(
    prepared: &PreparedPlanning<'_, '_>,
    state: &State,
    validator: &impl ArmStateValidator,
    sample_index: usize,
) -> Result<(), ArmAlgorithmError> {
    prepared
        .kinematics
        .validate_values(state.as_slice(prepared.dof()))?;
    validator
        .validate(state.as_slice(prepared.dof()))
        .map_err(|reason| ArmAlgorithmError::ConstraintViolation {
            sample_index,
            reason,
        })
}

pub(crate) fn edge_is_valid(
    prepared: &PreparedPlanning<'_, '_>,
    start: &State,
    goal: &State,
    resolution: f64,
    validator: &impl ArmStateValidator,
) -> bool {
    let distance = squared_distance(start, goal, prepared.dof()).sqrt();
    let steps = (distance / resolution).ceil().max(1.0) as usize;
    let mut state = State::zero();
    for index in 0..=steps {
        interpolate(
            start,
            goal,
            index as f64 / steps as f64,
            prepared.dof(),
            &mut state,
        );
        if validate_state(prepared, &state, validator, index).is_err() {
            return false;
        }
    }
    true
}

pub(crate) fn interpolate_edge(
    start: &State,
    goal: &State,
    step_size: f64,
    dof: usize,
    output: &mut Vec<State>,
) -> Result<(), ArmAlgorithmError> {
    output.clear();
    let distance = squared_distance(start, goal, dof).sqrt();
    let steps = (distance / step_size).ceil().max(1.0) as usize;
    let required = steps + 1;
    if required > output.capacity() || required > MAX_TRAJECTORY_POINTS {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.waypoints",
            actual: required,
            maximum: output.capacity().min(MAX_TRAJECTORY_POINTS),
        });
    }
    for index in 0..=steps {
        let mut state = State::zero();
        interpolate(start, goal, index as f64 / steps as f64, dof, &mut state);
        output.push(state);
    }
    Ok(())
}

pub(crate) fn sample_uniform(
    prepared: &PreparedPlanning<'_, '_>,
    rng: &mut ChaCha8Rng,
    output: &mut State,
) {
    for index in 0..prepared.dof() {
        let [lower, upper] = prepared.kinematics.sampling_bound(index);
        output.values[index] = rng.random_range(lower..=upper);
    }
}

pub(crate) fn interpolate(start: &State, goal: &State, alpha: f64, dof: usize, output: &mut State) {
    for index in 0..dof {
        output.values[index] =
            (goal.values[index] - start.values[index]).mul_add(alpha, start.values[index]);
    }
}

pub(crate) fn steer(
    start: &State,
    target: &State,
    maximum_step: f64,
    dof: usize,
    output: &mut State,
) {
    let distance = squared_distance(start, target, dof).sqrt();
    if distance <= maximum_step {
        *output = *target;
    } else {
        interpolate(start, target, maximum_step / distance, dof, output);
    }
}

pub(crate) fn squared_distance(left: &State, right: &State, dof: usize) -> f64 {
    left.values[..dof]
        .iter()
        .zip(&right.values[..dof])
        .map(|(left, right)| (right - left).powi(2))
        .sum()
}

pub(crate) fn nearest(nodes: &[Node], target: &State, dof: usize) -> usize {
    nodes
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| {
            squared_distance(&left.state, target, dof).total_cmp(&squared_distance(
                &right.state,
                target,
                dof,
            ))
        })
        .map_or(0, |(index, _)| index)
}

pub(crate) fn total_nodes(workspace: &PlannerWorkspace) -> usize {
    workspace.first.len() + workspace.second.len()
}

pub(crate) fn push_node(
    nodes: &mut Vec<Node>,
    node: Node,
    total_nodes: usize,
    config: PlannerConfig,
) -> Option<usize> {
    if total_nodes >= config.maximum_nodes as usize || nodes.len() == nodes.capacity() {
        return None;
    }
    let index = nodes.len();
    nodes.push(node);
    Some(index)
}

pub(crate) fn join_bidirectional(
    first: &[Node],
    first_index: usize,
    second: &[Node],
    second_index: usize,
    first_from_start: bool,
    output: &mut Vec<State>,
) -> Result<(), ArmAlgorithmError> {
    output.clear();
    let (start_tree, start_index, goal_tree, goal_index) = if first_from_start {
        (first, first_index, second, second_index)
    } else {
        (second, second_index, first, first_index)
    };
    trace_root_to(start_tree, start_index, output)?;
    let mut index = goal_index;
    loop {
        if output.len() == output.capacity() || output.len() == MAX_TRAJECTORY_POINTS {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.waypoints",
                actual: output.len() + 1,
                maximum: output.capacity().min(MAX_TRAJECTORY_POINTS),
            });
        }
        output.push(goal_tree[index].state);
        let Some(parent) = goal_tree[index].parent() else {
            break;
        };
        index = parent;
    }
    Ok(())
}

pub(crate) fn trace_root_to(
    nodes: &[Node],
    mut index: usize,
    output: &mut Vec<State>,
) -> Result<(), ArmAlgorithmError> {
    let start = output.len();
    loop {
        if output.len() == output.capacity() || output.len() == MAX_TRAJECTORY_POINTS {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.waypoints",
                actual: output.len() + 1,
                maximum: output.capacity().min(MAX_TRAJECTORY_POINTS),
            });
        }
        output.push(nodes[index].state);
        let Some(parent) = nodes[index].parent() else {
            break;
        };
        index = parent;
    }
    output[start..].reverse();
    Ok(())
}

pub(crate) fn joint_center_cost(prepared: &PreparedPlanning<'_, '_>, state: &State) -> f64 {
    let mut total = 0.0;
    for index in 0..prepared.dof() {
        let [lower, upper] = prepared.kinematics.sampling_bound(index);
        let center = (lower + upper) * 0.5;
        let range = (upper - lower).max(1.0e-9);
        total += ((state.values[index] - center) / range).powi(2);
    }
    total.sqrt()
}
