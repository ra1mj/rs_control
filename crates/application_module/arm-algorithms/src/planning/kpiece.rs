//! KINETIC-informed bounded KPIECE kernel.
//!
//! Source: `kinetic-planning/src/kpiece.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `8b7bc7c90e6aee9b78649f21fa6bc8c69b881f635fcf9492e25e1225755b4dfc`).
//! Retains low-dimensional projection, border-cell preference and cell outcome
//! accounting in a deterministic preallocated table.

use rand::RngExt;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, ProjectionCell, State, edge_is_valid, push_node, squared_distance,
        steer,
    },
};

const CELL_SIZE: f64 = 0.25;
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
    workspace.cells.push(ProjectionCell {
        key: project(&start, prepared.dof()),
        ..ProjectionCell::default()
    });
    let mut samples = 0_u32;
    let mut border_selections = 0_u32;
    let mut successful_expansions = 0_u32;
    let mut failed_expansions = 0_u32;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || workspace.first.len() >= config.maximum_nodes as usize
        {
            break;
        }
        update_neighbors(&mut workspace.cells);
        let cell_index = select_cell(&workspace.cells, &mut workspace.rng);
        if workspace.cells[cell_index].occupied_neighbors < 4 {
            border_selections += 1;
        }
        workspace.cells[cell_index].selections += 1;
        let node_index = select_node_in_cell(
            &workspace.first,
            workspace.cells[cell_index].key,
            prepared.dof(),
            &mut workspace.rng,
        );
        let parent = workspace.first[node_index];
        let mut next = parent.state;
        if workspace.rng.random::<f64>() < config.goal_bias {
            steer(
                &parent.state,
                &goal,
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
            workspace.cells[cell_index].failures += 1;
            failed_expansions += 1;
            continue;
        }
        let node_count = workspace.first.len();
        let Some(new_index) = push_node(
            &mut workspace.first,
            Node::child(
                next,
                node_index,
                parent.cost + squared_distance(&parent.state, &next, prepared.dof()).sqrt(),
            ),
            node_count,
            config,
        ) else {
            break;
        };
        workspace.cells[cell_index].successes += 1;
        successful_expansions += 1;
        let key = project(&next, prepared.dof());
        if !workspace.cells.iter().any(|cell| cell.key == key) {
            if workspace.cells.len() == workspace.cells.capacity() {
                return Err(ArmAlgorithmError::Capacity {
                    field: "planner.kpiece_cells",
                    actual: workspace.cells.len() + 1,
                    maximum: workspace.cells.capacity(),
                });
            }
            workspace.cells.push(ProjectionCell {
                key,
                ..ProjectionCell::default()
            });
        }
        if edge_is_valid(
            prepared,
            &workspace.first[new_index].state,
            &goal,
            config.edge_resolution,
            validator,
        ) {
            super::core::trace_root_to(&workspace.first, new_index, &mut workspace.path)?;
            if workspace.path.len() == workspace.path.capacity() {
                return Err(ArmAlgorithmError::Capacity {
                    field: "planner.waypoints",
                    actual: workspace.path.len() + 1,
                    maximum: workspace.path.capacity(),
                });
            }
            workspace.path.push(goal);
            return Ok(PlannerStatistics::new(
                iteration,
                samples,
                workspace.first.len() as u32,
                PlannerEvidence::Kpiece {
                    cells: workspace.cells.len() as u32,
                    border_selections,
                    successful_expansions,
                    failed_expansions,
                },
            ));
        }
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: PlannerKind::Kpiece.name(),
        iterations: completed_iterations,
        samples,
        nodes: workspace.first.len() as u32,
    })
}

fn project(state: &State, dof: usize) -> [i32; 2] {
    let first = (state.as_slice(dof)[0] / CELL_SIZE).floor() as i32;
    let second = if dof > 1 {
        (state.as_slice(dof)[1] / CELL_SIZE).floor() as i32
    } else {
        0
    };
    [first, second]
}

fn update_neighbors(cells: &mut [ProjectionCell]) {
    for index in 0..cells.len() {
        let key = cells[index].key;
        cells[index].occupied_neighbors = cells
            .iter()
            .filter(|other| {
                let dx = (other.key[0] - key[0]).abs();
                let dy = (other.key[1] - key[1]).abs();
                dx + dy == 1
            })
            .count()
            .min(4) as u8;
    }
}

fn importance(cell: &ProjectionCell) -> f64 {
    let border = if cell.occupied_neighbors < 4 {
        2.0
    } else {
        1.0
    };
    border * (1.0 + f64::from(cell.successes))
        / ((1.0 + f64::from(cell.failures)) * (1.0 + f64::from(cell.selections)))
}

fn select_cell(cells: &[ProjectionCell], rng: &mut rand_chacha::ChaCha8Rng) -> usize {
    let total = cells.iter().map(importance).sum::<f64>();
    let mut selected = rng.random::<f64>() * total;
    for (index, cell) in cells.iter().enumerate() {
        selected -= importance(cell);
        if selected <= 0.0 {
            return index;
        }
    }
    cells.len() - 1
}

fn select_node_in_cell(
    nodes: &[Node],
    key: [i32; 2],
    dof: usize,
    rng: &mut rand_chacha::ChaCha8Rng,
) -> usize {
    let count = nodes
        .iter()
        .filter(|node| project(&node.state, dof) == key)
        .count();
    let selected = rng.random_range(0..count);
    nodes
        .iter()
        .enumerate()
        .filter(|(_, node)| project(&node.state, dof) == key)
        .nth(selected)
        .map_or(0, |(index, _)| index)
}
