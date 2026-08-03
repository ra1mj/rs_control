//! KINETIC-informed bounded RRT* kernel.
//!
//! Source: `kinetic-planning/src/rrt_star.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `1f450ec3a2d8e20ca4a93dbc10e7b7681db94bacaf0f4054cae7784d1d7e1c18`).
//! The project adaptation retains radius-based parent selection, rewiring and
//! informed sampling while replacing implicit entropy/timeouts and growable hot paths.

use rand::RngExt;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, State, edge_is_valid, nearest, push_node, sample_uniform,
        squared_distance, steer, trace_root_to,
    },
};

const GAMMA: f64 = 1.5;
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
    let mut samples = 0_u32;
    let mut parent_candidates = 0_u32;
    let mut rewires = 0_u32;
    let mut informed_samples = 0_u32;
    let mut best: Option<usize> = None;
    let mut best_cost = f64::INFINITY;
    let mut first_solution_iteration = None;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || workspace.first.len() >= config.maximum_nodes as usize
        {
            break;
        }
        if first_solution_iteration.is_some_and(|first| iteration > first + IMPROVEMENT_ITERATIONS)
        {
            break;
        }

        let mut sample = State::zero();
        if workspace.rng.random::<f64>() < config.goal_bias {
            sample = goal;
        } else if best.is_some() {
            sample_informed(
                prepared,
                &start,
                &goal,
                best_cost,
                &mut workspace.rng,
                &mut sample,
            );
            samples += 1;
            informed_samples += 1;
        } else {
            sample_uniform(prepared, &mut workspace.rng, &mut sample);
            samples += 1;
        }

        let nearest_index = nearest(&workspace.first, &sample, prepared.dof());
        let mut next = State::zero();
        steer(
            &workspace.first[nearest_index].state,
            &sample,
            config.step_size,
            prepared.dof(),
            &mut next,
        );
        if !edge_is_valid(
            prepared,
            &workspace.first[nearest_index].state,
            &next,
            config.edge_resolution,
            validator,
        ) {
            continue;
        }

        let radius = near_radius(workspace.first.len(), prepared.dof(), config.step_size);
        workspace.neighbors.clear();
        for (index, node) in workspace.first.iter().enumerate() {
            if squared_distance(&node.state, &next, prepared.dof()) <= radius * radius {
                workspace.neighbors.push(index);
            }
        }
        parent_candidates += workspace.neighbors.len() as u32;
        let mut best_parent = nearest_index;
        let mut new_cost = workspace.first[nearest_index].cost
            + squared_distance(&workspace.first[nearest_index].state, &next, prepared.dof()).sqrt();
        for &candidate in &workspace.neighbors {
            let candidate_cost = workspace.first[candidate].cost
                + squared_distance(&workspace.first[candidate].state, &next, prepared.dof()).sqrt();
            if candidate_cost < new_cost
                && edge_is_valid(
                    prepared,
                    &workspace.first[candidate].state,
                    &next,
                    config.edge_resolution,
                    validator,
                )
            {
                best_parent = candidate;
                new_cost = candidate_cost;
            }
        }
        let node_count = workspace.first.len();
        let Some(new_index) = push_node(
            &mut workspace.first,
            Node::child(next, best_parent, new_cost),
            node_count,
            config,
        ) else {
            break;
        };

        for neighbor_offset in 0..workspace.neighbors.len() {
            let candidate = workspace.neighbors[neighbor_offset];
            if candidate == best_parent || candidate == 0 {
                continue;
            }
            let through_new = new_cost
                + squared_distance(
                    &workspace.first[new_index].state,
                    &workspace.first[candidate].state,
                    prepared.dof(),
                )
                .sqrt();
            if through_new < workspace.first[candidate].cost
                && edge_is_valid(
                    prepared,
                    &workspace.first[new_index].state,
                    &workspace.first[candidate].state,
                    config.edge_resolution,
                    validator,
                )
            {
                let delta = through_new - workspace.first[candidate].cost;
                workspace.first[candidate].parent = new_index;
                workspace.first[candidate].cost = through_new;
                propagate_cost(&mut workspace.first, candidate, delta);
                rewires += 1;
            }
        }

        let goal_cost = new_cost
            + squared_distance(&workspace.first[new_index].state, &goal, prepared.dof()).sqrt();
        if goal_cost < best_cost
            && edge_is_valid(
                prepared,
                &workspace.first[new_index].state,
                &goal,
                config.edge_resolution,
                validator,
            )
        {
            best = Some(new_index);
            best_cost = goal_cost;
            first_solution_iteration.get_or_insert(iteration);
        }
    }

    let Some(best_index) = best else {
        return Err(ArmAlgorithmError::WorkBudgetExhausted {
            planner: PlannerKind::RrtStar.name(),
            iterations: completed_iterations,
            samples,
            nodes: workspace.first.len() as u32,
        });
    };
    trace_root_to(&workspace.first, best_index, &mut workspace.path)?;
    if workspace.path.len() == workspace.path.capacity() {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.waypoints",
            actual: workspace.path.len() + 1,
            maximum: workspace.path.capacity(),
        });
    }
    workspace.path.push(goal);
    Ok(PlannerStatistics::new(
        completed_iterations,
        samples,
        workspace.first.len() as u32,
        PlannerEvidence::RrtStar {
            parent_candidates,
            rewires,
            best_cost,
            informed_samples,
        },
    ))
}

pub(super) fn near_radius(nodes: usize, dof: usize, step_size: f64) -> f64 {
    if nodes <= 1 {
        return step_size * 3.0;
    }
    (GAMMA * ((nodes as f64).ln() / nodes as f64).powf(1.0 / dof as f64))
        .min(step_size * 3.0)
        .max(step_size)
}

fn sample_informed(
    prepared: &PreparedPlanning<'_, '_>,
    start: &State,
    goal: &State,
    best_cost: f64,
    rng: &mut rand_chacha::ChaCha8Rng,
    output: &mut State,
) {
    let dof = prepared.dof();
    let minimum_cost = squared_distance(start, goal, dof).sqrt();
    let mut unit = State::zero();
    let mut norm_squared = 0.0;
    for value in unit.as_mut_slice(dof) {
        *value = rng.random_range(-1.0..=1.0);
        norm_squared += *value * *value;
    }
    if norm_squared <= f64::EPSILON {
        unit.as_mut_slice(dof)[0] = 1.0;
        norm_squared = 1.0;
    }
    let radius = rng.random::<f64>().powf(1.0 / dof as f64) / norm_squared.sqrt();
    for value in unit.as_mut_slice(dof) {
        *value *= radius;
    }

    let major = best_cost * 0.5;
    let minor = ((best_cost * best_cost - minimum_cost * minimum_cost).max(0.0)).sqrt() * 0.5;
    unit.as_mut_slice(dof)[0] *= major;
    for value in &mut unit.as_mut_slice(dof)[1..] {
        *value *= minor;
    }

    let mut householder_norm = 0.0;
    let mut projection = 0.0;
    for index in 0..dof {
        let direction = (goal.as_slice(dof)[index] - start.as_slice(dof)[index])
            / minimum_cost.max(f64::EPSILON);
        let vector = if index == 0 {
            1.0 - direction
        } else {
            -direction
        };
        householder_norm += vector * vector;
        projection += vector * unit.as_slice(dof)[index];
    }
    for index in 0..dof {
        let direction = (goal.as_slice(dof)[index] - start.as_slice(dof)[index])
            / minimum_cost.max(f64::EPSILON);
        let vector = if index == 0 {
            1.0 - direction
        } else {
            -direction
        };
        let rotated = if householder_norm <= f64::EPSILON {
            unit.as_slice(dof)[index]
        } else {
            unit.as_slice(dof)[index] - 2.0 * vector * projection / householder_norm
        };
        let center = (start.as_slice(dof)[index] + goal.as_slice(dof)[index]) * 0.5;
        let [lower, upper] = prepared.kinematics.sampling_bound(index);
        output.as_mut_slice(dof)[index] = (center + rotated).clamp(lower, upper);
    }
}

pub(super) fn propagate_cost(nodes: &mut [Node], parent: usize, delta: f64) {
    for index in 0..nodes.len() {
        if index == parent {
            continue;
        }
        let mut ancestor = nodes[index].parent();
        while let Some(current) = ancestor {
            if current == parent {
                nodes[index].cost += delta;
                break;
            }
            ancestor = nodes[current].parent();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Node, State, propagate_cost};

    #[test]
    fn rewiring_propagates_cost_only_to_descendants() {
        let state = State::zero();
        let mut nodes = [
            Node::root(state),
            Node::child(state, 0, 1.0),
            Node::child(state, 1, 2.0),
            Node::child(state, 0, 4.0),
        ];
        nodes[1].cost = 0.5;
        propagate_cost(&mut nodes, 1, -0.5);
        assert_eq!(nodes[0].cost, 0.0);
        assert_eq!(nodes[1].cost, 0.5);
        assert_eq!(nodes[2].cost, 1.5);
        assert_eq!(nodes[3].cost, 4.0);
    }
}
