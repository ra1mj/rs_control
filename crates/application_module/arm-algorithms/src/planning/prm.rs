//! KINETIC-informed bounded probabilistic-roadmap kernel.
//!
//! Source: `kinetic-planning/src/prm.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `772650a0a84ea2184d8eef48bdba3590df6aa0a0d47bb2ef6513358b7edc9577`).
//! The project adaptation separates construction from allocation-free online
//! A* queries and binds reuse to model plus semantic-constraint generations.

use message_model::{ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};

use crate::{ArmAlgorithmError, CollisionUnchecked};

use super::{
    ArmPlan, ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning, build_plan,
    core::{
        PlannerWorkspace, State, copy_position, edge_is_valid, interpolate_edge, sample_uniform,
        squared_distance, validate_state,
    },
};

const MAX_NEIGHBORS: usize = 16;
const DEFAULT_ROADMAP_SAMPLES: usize = 256;
const NO_NODE: usize = usize::MAX;

#[derive(Clone, Copy, Debug)]
struct RoadmapEdge {
    node: usize,
    cost: f64,
}

impl RoadmapEdge {
    const EMPTY: Self = Self {
        node: NO_NODE,
        cost: 0.0,
    };
}

#[derive(Clone, Debug)]
struct RoadmapNode {
    state: State,
    edges: [RoadmapEdge; MAX_NEIGHBORS],
    edge_count: u8,
}

impl RoadmapNode {
    fn new(state: State) -> Self {
        Self {
            state,
            edges: [RoadmapEdge::EMPTY; MAX_NEIGHBORS],
            edge_count: 0,
        }
    }

    fn edges(&self) -> &[RoadmapEdge] {
        &self.edges[..usize::from(self.edge_count)]
    }

    fn has_edge(&self, node: usize) -> bool {
        self.edges().iter().any(|edge| edge.node == node)
    }

    fn push_edge(&mut self, edge: RoadmapEdge, maximum_neighbors: usize) -> bool {
        if usize::from(self.edge_count) >= maximum_neighbors {
            return false;
        }
        self.edges[usize::from(self.edge_count)] = edge;
        self.edge_count += 1;
        true
    }
}

/// Construction/query policy for a reusable roadmap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PrmQueryOptions {
    maximum_neighbors: u8,
    lazy_validation: bool,
    roadmap_samples: u32,
}

impl PrmQueryOptions {
    /// Validates a fixed connection degree and optional query-time edge checks.
    pub fn new(maximum_neighbors: u8, lazy_validation: bool) -> Result<Self, ArmAlgorithmError> {
        if maximum_neighbors == 0 || usize::from(maximum_neighbors) > MAX_NEIGHBORS {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.prm_neighbors",
            });
        }
        Ok(Self {
            maximum_neighbors,
            lazy_validation,
            roadmap_samples: DEFAULT_ROADMAP_SAMPLES as u32,
        })
    }

    /// Selects the desired construction sample count within request budgets.
    pub fn with_roadmap_samples(mut self, roadmap_samples: u32) -> Result<Self, ArmAlgorithmError> {
        if roadmap_samples < 2 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.prm_samples",
            });
        }
        self.roadmap_samples = roadmap_samples;
        Ok(self)
    }

    /// Returns the desired roadmap node count before validity rejection.
    #[must_use]
    pub const fn roadmap_samples(self) -> u32 {
        self.roadmap_samples
    }

    /// Returns whether edges are checked during each query.
    #[must_use]
    pub const fn lazy_validation(self) -> bool {
        self.lazy_validation
    }
}

impl Default for PrmQueryOptions {
    fn default() -> Self {
        Self {
            maximum_neighbors: 8,
            lazy_validation: false,
            roadmap_samples: DEFAULT_ROADMAP_SAMPLES as u32,
        }
    }
}

/// Deterministic roadmap-construction accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PrmBuildStatistics {
    samples: u32,
    accepted_nodes: u32,
    edges: u32,
}

impl PrmBuildStatistics {
    /// Returns consumed random samples.
    #[must_use]
    pub const fn samples(self) -> u32 {
        self.samples
    }

    /// Returns accepted roadmap nodes.
    #[must_use]
    pub const fn accepted_nodes(self) -> u32 {
        self.accepted_nodes
    }

    /// Returns undirected roadmap edges.
    #[must_use]
    pub const fn edges(self) -> u32 {
        self.edges
    }
}

/// Reusable roadmap bound to the exact prepared profile and constraint generation.
pub struct PreparedRoadmap {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    model_generation: u64,
    constraint_generation: u64,
    seed: [u8; 32],
    options: PrmQueryOptions,
    nodes: Vec<RoadmapNode>,
    statistics: PrmBuildStatistics,
}

impl PreparedRoadmap {
    /// Builds a bounded graph once; subsequent queries reuse all graph storage.
    pub fn build(
        prepared: &PreparedPlanning<'_, '_>,
        config: PlannerConfig,
        options: PrmQueryOptions,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<Self, ArmAlgorithmError> {
        workspace.reset(prepared, config)?;
        let target = (options.roadmap_samples as usize)
            .min(config.maximum_samples as usize)
            .min(config.maximum_nodes as usize)
            .max(2);
        let mut nodes = Vec::with_capacity(target);
        let mut samples = 0_u32;
        while nodes.len() < target && samples < config.maximum_samples {
            let mut state = State::zero();
            sample_uniform(prepared, &mut workspace.rng, &mut state);
            samples += 1;
            if validate_state(prepared, &state, validator, samples as usize).is_ok() {
                nodes.push(RoadmapNode::new(state));
            }
        }
        if nodes.len() < 2 {
            return Err(ArmAlgorithmError::WorkBudgetExhausted {
                planner: PlannerKind::Prm.name(),
                iterations: 0,
                samples,
                nodes: nodes.len() as u32,
            });
        }

        let maximum_neighbors = usize::from(options.maximum_neighbors);
        let mut edge_count = 0_u32;
        for index in 0..nodes.len() {
            workspace.neighbors.clear();
            for candidate in 0..nodes.len() {
                if candidate == index {
                    continue;
                }
                insert_nearest(
                    &nodes,
                    index,
                    candidate,
                    prepared.dof(),
                    maximum_neighbors,
                    &mut workspace.neighbors,
                );
            }
            for offset in 0..workspace.neighbors.len() {
                let neighbor = workspace.neighbors[offset];
                if nodes[index].has_edge(neighbor)
                    || usize::from(nodes[index].edge_count) >= maximum_neighbors
                    || usize::from(nodes[neighbor].edge_count) >= maximum_neighbors
                {
                    continue;
                }
                if !options.lazy_validation
                    && !edge_is_valid(
                        prepared,
                        &nodes[index].state,
                        &nodes[neighbor].state,
                        config.edge_resolution,
                        validator,
                    )
                {
                    continue;
                }
                let cost =
                    squared_distance(&nodes[index].state, &nodes[neighbor].state, prepared.dof())
                        .sqrt();
                let forward = nodes[index].push_edge(
                    RoadmapEdge {
                        node: neighbor,
                        cost,
                    },
                    maximum_neighbors,
                );
                let reverse =
                    nodes[neighbor].push_edge(RoadmapEdge { node: index, cost }, maximum_neighbors);
                if forward && reverse {
                    edge_count += 1;
                }
            }
        }
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            seed: config.seed,
            options,
            statistics: PrmBuildStatistics {
                samples,
                accepted_nodes: nodes.len() as u32,
                edges: edge_count,
            },
            nodes,
        })
    }

    /// Returns construction accounting.
    #[must_use]
    pub const fn statistics(&self) -> PrmBuildStatistics {
        self.statistics
    }

    /// Runs an online query without rebuilding or growing the roadmap.
    pub fn query(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        start: &ArmJointVector<ArmPosition>,
        goal: &ArmJointVector<ArmPosition>,
        config: PlannerConfig,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
        workspace.reset(prepared, config)?;
        self.require_context(prepared)?;
        if workspace.maximum_nodes < self.nodes.len() {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.workspace_nodes",
                actual: self.nodes.len(),
                maximum: workspace.maximum_nodes,
            });
        }
        let start = copy_position(prepared, start)?;
        let goal = copy_position(prepared, goal)?;
        validate_state(prepared, &start, validator, 0)?;
        validate_state(prepared, &goal, validator, 1)?;
        let statistics =
            if edge_is_valid(prepared, &start, &goal, config.edge_resolution, validator) {
                interpolate_edge(
                    &start,
                    &goal,
                    config.step_size,
                    prepared.dof(),
                    &mut workspace.path,
                )?;
                PlannerStatistics::new(0, 0, 2, PlannerEvidence::DirectEdge)
            } else {
                self.query_states(prepared, start, goal, config, validator, workspace)?
            };
        build_plan(
            prepared,
            PlannerKind::Prm,
            self.seed,
            &workspace.path,
            statistics,
        )
    }

    fn require_context(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || &self.order != prepared.order()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "roadmap profile, group or joint order changed",
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
        Ok(())
    }

    fn query_states(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        start: State,
        goal: State,
        config: PlannerConfig,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<PlannerStatistics, ArmAlgorithmError> {
        let start_node = nearest_connectable(
            prepared,
            &self.nodes,
            &start,
            config.edge_resolution,
            validator,
        );
        let goal_node = nearest_connectable(
            prepared,
            &self.nodes,
            &goal,
            config.edge_resolution,
            validator,
        );
        let (Some(start_node), Some(goal_node)) = (start_node, goal_node) else {
            return Err(ArmAlgorithmError::WorkBudgetExhausted {
                planner: PlannerKind::Prm.name(),
                iterations: 0,
                samples: self.statistics.samples,
                nodes: self.statistics.accepted_nodes,
            });
        };

        workspace.scores.resize(self.nodes.len(), f64::INFINITY);
        workspace.parents.resize(self.nodes.len(), NO_NODE);
        workspace.density.resize(self.nodes.len(), 0);
        workspace.scores[start_node] = 0.0;
        let mut expanded = 0_u32;
        let mut lazy_edge_checks = 0_u32;

        for _ in 0..self.nodes.len() {
            let mut current = NO_NODE;
            let mut best_estimate = f64::INFINITY;
            for index in 0..self.nodes.len() {
                if workspace.density[index] != 0 || !workspace.scores[index].is_finite() {
                    continue;
                }
                let estimate = workspace.scores[index]
                    + squared_distance(&self.nodes[index].state, &goal, prepared.dof()).sqrt();
                if estimate < best_estimate {
                    best_estimate = estimate;
                    current = index;
                }
            }
            if current == NO_NODE {
                break;
            }
            expanded += 1;
            workspace.density[current] = 1;
            if current == goal_node {
                break;
            }
            for edge in self.nodes[current].edges() {
                if self.options.lazy_validation {
                    lazy_edge_checks += 1;
                    if !edge_is_valid(
                        prepared,
                        &self.nodes[current].state,
                        &self.nodes[edge.node].state,
                        config.edge_resolution,
                        validator,
                    ) {
                        continue;
                    }
                }
                let candidate = workspace.scores[current] + edge.cost;
                if candidate < workspace.scores[edge.node] {
                    workspace.scores[edge.node] = candidate;
                    workspace.parents[edge.node] = current;
                }
            }
        }
        if !workspace.scores[goal_node].is_finite() {
            return Err(ArmAlgorithmError::WorkBudgetExhausted {
                planner: PlannerKind::Prm.name(),
                iterations: expanded,
                samples: self.statistics.samples,
                nodes: self.statistics.accepted_nodes,
            });
        }

        workspace.path.clear();
        workspace.path.push(start);
        let trace_start = workspace.path.len();
        let mut current = goal_node;
        loop {
            if workspace.path.len() == workspace.path.capacity() {
                return Err(ArmAlgorithmError::Capacity {
                    field: "planner.waypoints",
                    actual: workspace.path.len() + 1,
                    maximum: workspace.path.capacity(),
                });
            }
            workspace.path.push(self.nodes[current].state);
            if current == start_node {
                break;
            }
            current = workspace.parents[current];
        }
        workspace.path[trace_start..].reverse();
        if workspace.path.len() == workspace.path.capacity() {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.waypoints",
                actual: workspace.path.len() + 1,
                maximum: workspace.path.capacity(),
            });
        }
        workspace.path.push(goal);
        Ok(PlannerStatistics::new(
            expanded,
            self.statistics.samples,
            self.statistics.accepted_nodes,
            PlannerEvidence::Prm {
                roadmap_nodes: self.statistics.accepted_nodes,
                roadmap_edges: self.statistics.edges,
                nodes_expanded: expanded,
                lazy_edge_checks,
            },
        ))
    }
}

pub(super) fn solve(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    let roadmap = PreparedRoadmap::build(
        prepared,
        config,
        PrmQueryOptions::default(),
        validator,
        workspace,
    )?;
    workspace.reset(prepared, config)?;
    roadmap.query_states(prepared, start, goal, config, validator, workspace)
}

fn insert_nearest(
    nodes: &[RoadmapNode],
    source: usize,
    candidate: usize,
    dof: usize,
    maximum: usize,
    output: &mut Vec<usize>,
) {
    let distance = squared_distance(&nodes[source].state, &nodes[candidate].state, dof);
    let position = output
        .iter()
        .position(|current| {
            distance < squared_distance(&nodes[source].state, &nodes[*current].state, dof)
        })
        .unwrap_or(output.len());
    if position < maximum {
        output.insert(position, candidate);
        if output.len() > maximum {
            output.pop();
        }
    } else if output.len() < maximum {
        output.push(candidate);
    }
}

fn nearest_connectable(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &[RoadmapNode],
    state: &State,
    resolution: f64,
    validator: &impl ArmStateValidator,
) -> Option<usize> {
    let mut best = None;
    let mut best_distance = f64::INFINITY;
    for (index, node) in nodes.iter().enumerate() {
        let distance = squared_distance(state, &node.state, prepared.dof());
        if distance < best_distance
            && edge_is_valid(prepared, state, &node.state, resolution, validator)
        {
            best = Some(index);
            best_distance = distance;
        }
    }
    best
}
