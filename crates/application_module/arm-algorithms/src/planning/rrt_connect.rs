//! KINETIC-informed bounded RRT-Connect kernel.
//!
//! Source: `kinetic-planning/src/rrt.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `fb3e09df4cfa1d53803cdf10241cb3e0b1bde3310dea772cb8a7ff915e0b56fc`).
//! Adapted to project-owned prepared identities, deterministic ChaCha8 seeds,
//! caller validators, aggregate node ceilings and allocation-bounded storage.

use rand::RngExt;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{
        Node, PlannerWorkspace, State, edge_is_valid, join_bidirectional, nearest, push_node,
        sample_uniform, squared_distance, steer, total_nodes,
    },
};

enum Extend {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

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
    let mut connect_attempts = 0_u32;
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
        let other_len = workspace.second.len();
        if let Extend::Reached(first_index) | Extend::Advanced(first_index) = extend(
            prepared,
            &mut workspace.first,
            other_len,
            &sample,
            config,
            validator,
        ) {
            connect_attempts += 1;
            let target = workspace.first[first_index].state;
            let first_len = workspace.first.len();
            if let Some(second_index) = connect(
                prepared,
                &mut workspace.second,
                first_len,
                &target,
                config,
                validator,
            ) {
                join_bidirectional(
                    &workspace.first,
                    first_index,
                    &workspace.second,
                    second_index,
                    first_from_start,
                    &mut workspace.path,
                )?;
                return Ok(PlannerStatistics::new(
                    iteration,
                    samples,
                    total_nodes(workspace) as u32,
                    PlannerEvidence::RrtConnect { connect_attempts },
                ));
            }
        }
        std::mem::swap(&mut workspace.first, &mut workspace.second);
        first_from_start = !first_from_start;
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: PlannerKind::RrtConnect.name(),
        iterations: completed_iterations,
        samples,
        nodes: total_nodes(workspace) as u32,
    })
}

fn extend(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Extend {
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
        return Extend::Trapped;
    }
    let reached = squared_distance(&next, target, prepared.dof()) <= f64::EPSILON;
    let Some(index) = push_node(
        nodes,
        Node::child(
            next,
            nearest_index,
            parent.cost + squared_distance(&parent.state, &next, prepared.dof()).sqrt(),
        ),
        nodes.len() + other_nodes,
        config,
    ) else {
        return Extend::Trapped;
    };
    if reached {
        Extend::Reached(index)
    } else {
        Extend::Advanced(index)
    }
}

fn connect(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Option<usize> {
    loop {
        match extend(prepared, nodes, other_nodes, target, config, validator) {
            Extend::Reached(index) => return Some(index),
            Extend::Advanced(_) => {}
            Extend::Trapped => return None,
        }
        if nodes.len() + other_nodes >= config.maximum_nodes as usize {
            return None;
        }
    }
}
