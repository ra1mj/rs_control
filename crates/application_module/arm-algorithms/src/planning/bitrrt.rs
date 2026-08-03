//! KINETIC-informed bounded BiTRRT kernel.
//!
//! Source: `kinetic-planning/src/bitrrt.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `c7423111e66bf85627ebb509fb9f57ff83c3544ea7af539af1bbfc71a2a369f1`).
//! Retains joint-center cost, Boltzmann transition tests, annealing and
//! frustration reheating with deterministic seeded randomness.

use rand::RngExt;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, State, edge_is_valid, join_bidirectional, joint_center_cost,
        nearest, push_node, sample_uniform, squared_distance, steer, total_nodes,
    },
};

const INITIAL_TEMPERATURE: f64 = 1.0;
const ALPHA: f64 = 0.95;
const MIN_TEMPERATURE: f64 = 0.01;
const FRUSTRATION_THRESHOLD: u32 = 100;
const FRUSTRATION_FACTOR: f64 = 2.0;

pub(super) fn solve(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    workspace.first.push(Node::child(
        start,
        usize::MAX,
        joint_center_cost(prepared, &start),
    ));
    workspace.second.push(Node::child(
        goal,
        usize::MAX,
        joint_center_cost(prepared, &goal),
    ));
    let mut first_from_start = true;
    let mut samples = 0_u32;
    let mut rejected_transitions = 0_u32;
    let mut frustration_resets = 0_u32;
    let mut rejection_streak = 0_u32;
    let mut temperature = INITIAL_TEMPERATURE;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || total_nodes(workspace) >= config.maximum_nodes as usize
        {
            break;
        }
        let mut sample = State::zero();
        if workspace.rng.random::<f64>() < config.goal_bias {
            sample = workspace.second[0].state;
        } else {
            sample_uniform(prepared, &mut workspace.rng, &mut sample);
            samples += 1;
        }
        let nearest_index = nearest(&workspace.first, &sample, prepared.dof());
        let parent = workspace.first[nearest_index];
        let mut next = State::zero();
        steer(
            &parent.state,
            &sample,
            config.step_size,
            prepared.dof(),
            &mut next,
        );
        let next_cost = joint_center_cost(prepared, &next);
        let accepted = next_cost <= parent.cost
            || workspace.rng.random::<f64>() < (-(next_cost - parent.cost) / temperature).exp();
        if !accepted
            || !edge_is_valid(
                prepared,
                &parent.state,
                &next,
                config.edge_resolution,
                validator,
            )
        {
            rejected_transitions += 1;
            rejection_streak += 1;
        } else {
            rejection_streak = 0;
            let total = total_nodes(workspace);
            let Some(new_index) = push_node(
                &mut workspace.first,
                Node::child(next, nearest_index, next_cost),
                total,
                config,
            ) else {
                break;
            };
            let target = workspace.first[new_index].state;
            let first_len = workspace.first.len();
            if let Some(connection) = connect_plain(
                prepared,
                &mut workspace.second,
                first_len,
                &target,
                config,
                validator,
            ) {
                join_bidirectional(
                    &workspace.first,
                    new_index,
                    &workspace.second,
                    connection,
                    first_from_start,
                    &mut workspace.path,
                )?;
                return Ok(PlannerStatistics::new(
                    iteration,
                    samples,
                    total_nodes(workspace) as u32,
                    PlannerEvidence::BiTrrt {
                        transitions: iteration,
                        rejected_transitions,
                        frustration_resets,
                        final_temperature: temperature,
                    },
                ));
            }
        }
        if rejection_streak >= FRUSTRATION_THRESHOLD {
            temperature *= FRUSTRATION_FACTOR;
            rejection_streak = 0;
            frustration_resets += 1;
        } else {
            temperature = (temperature * ALPHA).max(MIN_TEMPERATURE);
        }
        std::mem::swap(&mut workspace.first, &mut workspace.second);
        first_from_start = !first_from_start;
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: PlannerKind::BiTrrt.name(),
        iterations: completed_iterations,
        samples,
        nodes: total_nodes(workspace) as u32,
    })
}

#[allow(clippy::too_many_arguments)]
fn connect_plain(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Option<usize> {
    loop {
        let nearest_index = nearest(nodes, target, prepared.dof());
        let parent = nodes[nearest_index];
        let mut next = State::zero();
        steer(
            &parent.state,
            target,
            config.step_size,
            prepared.dof(),
            &mut next,
        );
        if !edge_is_valid(
            prepared,
            &parent.state,
            &next,
            config.edge_resolution,
            validator,
        ) {
            return None;
        }
        let reached = squared_distance(&next, target, prepared.dof()) <= f64::EPSILON;
        let index = push_node(
            nodes,
            Node::child(next, nearest_index, joint_center_cost(prepared, &next)),
            nodes.len() + other_nodes,
            config,
        )?;
        if reached {
            return Some(index);
        }
    }
}
