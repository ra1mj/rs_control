//! KINETIC-informed bounded Expansive Space Trees kernel.
//!
//! Source: `kinetic-planning/src/est.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `bd2ed10dc3927856280bb34f367bcf8e04bfd87082b263de96071652725c87ab`).
//! Retains inverse-neighborhood-density selection and bidirectional expansion.

use rand::RngExt;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, State, edge_is_valid, join_bidirectional, nearest, push_node,
        squared_distance, total_nodes,
    },
};

const DENSITY_RADIUS: f64 = 0.5;
const EXPANSION_RANGE: f64 = 0.3;

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
    let mut density_evaluations = 0_u32;
    let mut sparse_selections = 0_u32;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || total_nodes(workspace) >= config.maximum_nodes as usize
        {
            break;
        }
        refresh_density(
            &workspace.first,
            prepared.dof(),
            &mut workspace.density,
            &mut density_evaluations,
        );
        let selected = select_inverse_density(&workspace.density, &mut workspace.rng);
        let minimum_density = workspace.density.iter().copied().min().unwrap_or(0);
        if workspace.density[selected] == minimum_density {
            sparse_selections += 1;
        }
        let parent = workspace.first[selected];
        let mut next = parent.state;
        if workspace.rng.random::<f64>() < config.goal_bias {
            let target = workspace.second[0].state;
            super::core::steer(
                &parent.state,
                &target,
                EXPANSION_RANGE,
                prepared.dof(),
                &mut next,
            );
        } else {
            for index in 0..prepared.dof() {
                let [lower, upper] = prepared.kinematics.sampling_bound(index);
                next.as_mut_slice(prepared.dof())[index] = (parent.state.as_slice(prepared.dof())
                    [index]
                    + workspace
                        .rng
                        .random_range(-EXPANSION_RANGE..=EXPANSION_RANGE))
                .clamp(lower, upper);
            }
            samples += 1;
        }
        if !edge_is_valid(
            prepared,
            &parent.state,
            &next,
            config.edge_resolution,
            validator,
        ) {
            std::mem::swap(&mut workspace.first, &mut workspace.second);
            first_from_start = !first_from_start;
            continue;
        }
        let total = total_nodes(workspace);
        let Some(new_index) = push_node(
            &mut workspace.first,
            Node::child(
                next,
                selected,
                parent.cost + squared_distance(&parent.state, &next, prepared.dof()).sqrt(),
            ),
            total,
            config,
        ) else {
            break;
        };
        let other_index = nearest(&workspace.second, &next, prepared.dof());
        if edge_is_valid(
            prepared,
            &next,
            &workspace.second[other_index].state,
            config.edge_resolution,
            validator,
        ) {
            join_bidirectional(
                &workspace.first,
                new_index,
                &workspace.second,
                other_index,
                first_from_start,
                &mut workspace.path,
            )?;
            return Ok(PlannerStatistics::new(
                iteration,
                samples,
                total_nodes(workspace) as u32,
                PlannerEvidence::Est {
                    density_evaluations,
                    sparse_selections,
                },
            ));
        }
        std::mem::swap(&mut workspace.first, &mut workspace.second);
        first_from_start = !first_from_start;
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: PlannerKind::Est.name(),
        iterations: completed_iterations,
        samples,
        nodes: total_nodes(workspace) as u32,
    })
}

fn refresh_density(nodes: &[Node], dof: usize, density: &mut Vec<u32>, evaluations: &mut u32) {
    density.clear();
    let radius_squared = DENSITY_RADIUS * DENSITY_RADIUS;
    for (left_index, left) in nodes.iter().enumerate() {
        let mut count = 0;
        for (right_index, right) in nodes.iter().enumerate() {
            if left_index != right_index {
                *evaluations = evaluations.saturating_add(1);
                if squared_distance(&left.state, &right.state, dof) <= radius_squared {
                    count += 1;
                }
            }
        }
        density.push(count);
    }
}

fn select_inverse_density(density: &[u32], rng: &mut rand_chacha::ChaCha8Rng) -> usize {
    let total = density
        .iter()
        .map(|count| 1.0 / (1.0 + f64::from(*count)))
        .sum::<f64>();
    let mut selection = rng.random::<f64>() * total;
    for (index, count) in density.iter().enumerate() {
        selection -= 1.0 / (1.0 + f64::from(*count));
        if selection <= 0.0 {
            return index;
        }
    }
    density.len() - 1
}
