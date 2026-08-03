//! KINETIC-informed bounded bidirectional RRT* kernel.
//!
//! Source: `kinetic-planning/src/bi_rrt_star.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `d320ee29fde028cf9dae8fea0033f405d3fa5cf3a2729d967bb34750e359eeae`).
//! Adapted to deterministic budgets and two independently rewired prepared trees.

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, State, edge_is_valid, join_bidirectional, nearest, push_node,
        sample_uniform, squared_distance, steer, total_nodes,
    },
    rrt_star::{near_radius, propagate_cost},
};

const IMPROVEMENT_ITERATIONS: u32 = 256;

pub(super) fn solve(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    workspace.first.push(Node::root(start));
    workspace.second.push(Node::root(goal));
    let mut first_from_start = true;
    let mut samples = 0_u32;
    let mut parent_candidates = 0_u32;
    let mut rewires = 0_u32;
    let mut connection_candidates = 0_u32;
    let mut best_cost = f64::INFINITY;
    let mut first_solution = None;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || total_nodes(workspace) >= config.maximum_nodes as usize
        {
            break;
        }
        if first_solution.is_some_and(|first| iteration > first + IMPROVEMENT_ITERATIONS) {
            break;
        }
        let mut sample = State::zero();
        sample_uniform(prepared, &mut workspace.rng, &mut sample);
        samples += 1;
        let other_len = workspace.second.len();
        let extension = extend_star(
            prepared,
            &mut workspace.first,
            &mut workspace.neighbors,
            other_len,
            &sample,
            config,
            validator,
        );
        if let Some((new_index, candidates, changed)) = extension {
            parent_candidates += candidates;
            rewires += changed;
            let nearest_other = nearest(
                &workspace.second,
                &workspace.first[new_index].state,
                prepared.dof(),
            );
            connection_candidates += 1;
            if edge_is_valid(
                prepared,
                &workspace.first[new_index].state,
                &workspace.second[nearest_other].state,
                config.edge_resolution,
                validator,
            ) {
                let candidate_cost = workspace.first[new_index].cost
                    + workspace.second[nearest_other].cost
                    + squared_distance(
                        &workspace.first[new_index].state,
                        &workspace.second[nearest_other].state,
                        prepared.dof(),
                    )
                    .sqrt();
                if candidate_cost < best_cost {
                    best_cost = candidate_cost;
                    join_bidirectional(
                        &workspace.first,
                        new_index,
                        &workspace.second,
                        nearest_other,
                        first_from_start,
                        &mut workspace.path,
                    )?;
                    first_solution.get_or_insert(iteration);
                }
            }
        }
        std::mem::swap(&mut workspace.first, &mut workspace.second);
        first_from_start = !first_from_start;
    }

    if !best_cost.is_finite() {
        return Err(ArmAlgorithmError::WorkBudgetExhausted {
            planner: PlannerKind::BidirectionalRrtStar.name(),
            iterations: completed_iterations,
            samples,
            nodes: total_nodes(workspace) as u32,
        });
    }
    Ok(PlannerStatistics::new(
        completed_iterations,
        samples,
        total_nodes(workspace) as u32,
        PlannerEvidence::BidirectionalRrtStar {
            parent_candidates,
            rewires,
            connection_candidates,
            best_cost,
        },
    ))
}

#[allow(clippy::too_many_arguments)]
fn extend_star(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    neighbors: &mut Vec<usize>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Option<(usize, u32, u32)> {
    let nearest_index = nearest(nodes, target, prepared.dof());
    let mut next = State::zero();
    steer(
        &nodes[nearest_index].state,
        target,
        config.step_size,
        prepared.dof(),
        &mut next,
    );
    if !edge_is_valid(
        prepared,
        &nodes[nearest_index].state,
        &next,
        config.edge_resolution,
        validator,
    ) {
        return None;
    }
    let radius = near_radius(nodes.len(), prepared.dof(), config.step_size);
    neighbors.clear();
    for (index, node) in nodes.iter().enumerate() {
        if squared_distance(&node.state, &next, prepared.dof()) <= radius * radius {
            neighbors.push(index);
        }
    }
    let candidate_count = neighbors.len() as u32;
    let mut best_parent = nearest_index;
    let mut best_cost = nodes[nearest_index].cost
        + squared_distance(&nodes[nearest_index].state, &next, prepared.dof()).sqrt();
    for &candidate in neighbors.iter() {
        let cost = nodes[candidate].cost
            + squared_distance(&nodes[candidate].state, &next, prepared.dof()).sqrt();
        if cost < best_cost
            && edge_is_valid(
                prepared,
                &nodes[candidate].state,
                &next,
                config.edge_resolution,
                validator,
            )
        {
            best_parent = candidate;
            best_cost = cost;
        }
    }
    let new_index = push_node(
        nodes,
        Node::child(next, best_parent, best_cost),
        nodes.len() + other_nodes,
        config,
    )?;
    let mut rewires = 0;
    for &candidate in neighbors.iter() {
        if candidate == 0 || candidate == best_parent {
            continue;
        }
        let through_new = best_cost
            + squared_distance(
                &nodes[new_index].state,
                &nodes[candidate].state,
                prepared.dof(),
            )
            .sqrt();
        if through_new < nodes[candidate].cost
            && edge_is_valid(
                prepared,
                &nodes[new_index].state,
                &nodes[candidate].state,
                config.edge_resolution,
                validator,
            )
        {
            let delta = through_new - nodes[candidate].cost;
            nodes[candidate].parent = new_index;
            nodes[candidate].cost = through_new;
            propagate_cost(nodes, candidate, delta);
            rewires += 1;
        }
    }
    Some((new_index, candidate_count, rewires))
}
