//! KINETIC-informed prepared graph-of-convex-sets planning.
//!
//! Source: `kinetic-planning/src/gcs.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `5e7ba1b7b66798628b70e1451d8b6b1757a5c8f84e9d97470bfbd9ffc6976aef`).
//! The native adaptation stores a bounded region graph, reuses caller-owned
//! A* scratch and revalidates every returned edge. Regions are semantic
//! constraint regions only; no collision-free claim is made.

use message_model::{ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};

use crate::{ArmAlgorithmError, CollisionUnchecked};

use super::{
    ArmPlan, ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning, build_plan,
    core::{
        PlannerWorkspace, State, copy_position, edge_is_valid, interpolate_edge, squared_distance,
        validate_state,
    },
    iris::{self, PreparedConstraintRegions},
};

const MAX_REGION_NEIGHBORS: usize = 12;
const NO_REGION: usize = usize::MAX;

#[derive(Clone, Copy, Debug)]
struct RegionEdge {
    target: usize,
    cost: f64,
}

/// Immutable constraint-region graph with allocation-stable online queries.
#[derive(Clone)]
pub struct PreparedRegionGraph {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    model_generation: u64,
    constraint_generation: u64,
    seed: [u8; 32],
    regions: PreparedConstraintRegions,
    offsets: Vec<usize>,
    edges: Vec<RegionEdge>,
}

impl PreparedRegionGraph {
    /// Builds a bounded directed adjacency graph from prepared semantic regions.
    pub fn build(
        prepared: &PreparedPlanning<'_, '_>,
        regions: &PreparedConstraintRegions,
        config: PlannerConfig,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<Self, ArmAlgorithmError> {
        workspace.reset(prepared, config)?;
        regions.require_context(prepared)?;
        if regions.regions().len() > config.maximum_nodes as usize {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.gcs_regions",
                actual: regions.regions().len(),
                maximum: config.maximum_nodes as usize,
            });
        }
        for (index, region) in regions.regions().iter().enumerate() {
            validate_state(prepared, &region.center_state(), validator, index)?;
        }

        let mut offsets = Vec::with_capacity(regions.regions().len() + 1);
        let mut edges =
            Vec::with_capacity(regions.regions().len().saturating_mul(MAX_REGION_NEIGHBORS));
        offsets.push(0);
        for source in 0..regions.regions().len() {
            workspace.neighbors.clear();
            for candidate in 0..regions.regions().len() {
                if source == candidate {
                    continue;
                }
                let source_region = &regions.regions()[source];
                let candidate_region = &regions.regions()[candidate];
                if !edge_is_valid(
                    prepared,
                    &source_region.center_state(),
                    &candidate_region.center_state(),
                    config.edge_resolution,
                    validator,
                ) {
                    continue;
                }
                insert_nearest_region(
                    regions,
                    source,
                    candidate,
                    prepared.dof(),
                    &mut workspace.neighbors,
                );
            }
            for &target in &workspace.neighbors {
                edges.push(RegionEdge {
                    target,
                    cost: squared_distance(
                        &regions.regions()[source].center_state(),
                        &regions.regions()[target].center_state(),
                        prepared.dof(),
                    )
                    .sqrt(),
                });
            }
            offsets.push(edges.len());
        }

        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            seed: config.seed,
            regions: regions.clone(),
            offsets,
            edges,
        })
    }

    /// Returns the number of prepared regions.
    #[must_use]
    pub fn region_count(&self) -> usize {
        self.regions.regions().len()
    }

    /// Returns the number of directed prepared graph edges.
    #[must_use]
    pub fn edge_count(&self) -> usize {
        self.edges.len()
    }

    /// Queries the prepared region graph without rebuilding it.
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
        if workspace.maximum_nodes < self.regions.regions().len() {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.workspace_nodes",
                actual: self.regions.regions().len(),
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
            PlannerKind::Gcs,
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
                reason: "GCS profile, group or order changed",
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
        self.regions.require_context(prepared)
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
        let Some(start_region) =
            self.nearest_connectable(prepared, &start, config.edge_resolution, validator)
        else {
            return self.exhausted(0);
        };
        let Some(goal_region) =
            self.nearest_connectable(prepared, &goal, config.edge_resolution, validator)
        else {
            return self.exhausted(0);
        };

        let region_count = self.regions.regions().len();
        workspace.scores.resize(region_count, f64::INFINITY);
        workspace.parents.resize(region_count, NO_REGION);
        workspace.density.resize(region_count, 0);
        workspace.scores[start_region] = 0.0;
        let mut expanded = 0_u32;

        for _ in 0..region_count {
            let mut current = NO_REGION;
            let mut best = f64::INFINITY;
            for index in 0..region_count {
                if workspace.density[index] != 0 || !workspace.scores[index].is_finite() {
                    continue;
                }
                let estimate = workspace.scores[index]
                    + squared_distance(
                        &self.regions.regions()[index].center_state(),
                        &goal,
                        prepared.dof(),
                    )
                    .sqrt();
                if estimate < best {
                    current = index;
                    best = estimate;
                }
            }
            if current == NO_REGION {
                break;
            }
            expanded += 1;
            workspace.density[current] = 1;
            if current == goal_region {
                break;
            }
            for edge in &self.edges[self.offsets[current]..self.offsets[current + 1]] {
                let candidate = workspace.scores[current] + edge.cost;
                if candidate < workspace.scores[edge.target] {
                    workspace.scores[edge.target] = candidate;
                    workspace.parents[edge.target] = current;
                }
            }
        }
        if !workspace.scores[goal_region].is_finite() {
            return self.exhausted(expanded);
        }

        workspace.path.clear();
        workspace.path.push(start);
        let trace_start = workspace.path.len();
        let mut current = goal_region;
        loop {
            push_path(
                &mut workspace.path,
                self.regions.regions()[current].center_state(),
            )?;
            if current == start_region {
                break;
            }
            current = workspace.parents[current];
            if current == NO_REGION {
                return self.exhausted(expanded);
            }
        }
        workspace.path[trace_start..].reverse();
        push_path(&mut workspace.path, goal)?;
        for edge in workspace.path.windows(2) {
            if !edge_is_valid(
                prepared,
                &edge[0],
                &edge[1],
                config.edge_resolution,
                validator,
            ) {
                return self.exhausted(expanded);
            }
        }
        Ok(PlannerStatistics::new(
            expanded,
            self.regions.samples(),
            region_count as u32,
            PlannerEvidence::Gcs {
                regions: region_count as u32,
                edges: self.edges.len() as u32,
                expanded_regions: expanded,
                refined_waypoints: workspace.path.len() as u32,
            },
        ))
    }

    fn nearest_connectable(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
        state: &State,
        resolution: f64,
        validator: &impl ArmStateValidator,
    ) -> Option<usize> {
        let mut best = None;
        let mut best_distance = f64::INFINITY;
        for (index, region) in self.regions.regions().iter().enumerate() {
            let distance = squared_distance(state, &region.center_state(), prepared.dof());
            if distance < best_distance
                && edge_is_valid(
                    prepared,
                    state,
                    &region.center_state(),
                    resolution,
                    validator,
                )
            {
                best = Some(index);
                best_distance = distance;
            }
        }
        best
    }

    fn exhausted<T>(&self, iterations: u32) -> Result<T, ArmAlgorithmError> {
        Err(ArmAlgorithmError::WorkBudgetExhausted {
            planner: PlannerKind::Gcs.name(),
            iterations,
            samples: self.regions.samples(),
            nodes: self.regions.regions().len() as u32,
        })
    }
}

fn push_path(output: &mut Vec<State>, state: State) -> Result<(), ArmAlgorithmError> {
    if output.len() == output.capacity() {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.waypoints",
            actual: output.len() + 1,
            maximum: output.capacity(),
        });
    }
    output.push(state);
    Ok(())
}

fn insert_nearest_region(
    regions: &PreparedConstraintRegions,
    source: usize,
    candidate: usize,
    dof: usize,
    output: &mut Vec<usize>,
) {
    let distance = squared_distance(
        &regions.regions()[source].center_state(),
        &regions.regions()[candidate].center_state(),
        dof,
    );
    let position = output
        .iter()
        .position(|current| {
            distance
                < squared_distance(
                    &regions.regions()[source].center_state(),
                    &regions.regions()[*current].center_state(),
                    dof,
                )
        })
        .unwrap_or(output.len());
    if position < MAX_REGION_NEIGHBORS {
        output.insert(position, candidate);
        if output.len() > MAX_REGION_NEIGHBORS {
            output.pop();
        }
    } else if output.len() < MAX_REGION_NEIGHBORS {
        output.push(candidate);
    }
}

pub(super) fn solve_gcs(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    let regions = iris::sample_point_regions(prepared, config, validator, workspace)?;
    let graph = PreparedRegionGraph::build(prepared, &regions, config, validator, workspace)?;
    workspace.reset(prepared, config)?;
    graph.query_states(prepared, start, goal, config, validator, workspace)
}

pub(super) fn solve_iris(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    let regions = iris::sample_point_regions(prepared, config, validator, workspace)?;
    let graph = PreparedRegionGraph::build(prepared, &regions, config, validator, workspace)?;
    workspace.reset(prepared, config)?;
    graph.query_states(prepared, start, goal, config, validator, workspace)?;
    Ok(iris::statistics(&regions))
}
