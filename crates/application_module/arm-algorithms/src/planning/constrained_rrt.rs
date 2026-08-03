//! KINETIC-informed projected constrained RRT-Connect.
//!
//! Source: `kinetic-planning/src/constrained_rrt.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `7aef1ff7b5b0902b02ec0bd1b961c86504cbd2c5de7c5eee9a345dfa59d4e857`).
//! Every random sample and tree extension is projected through the explicit
//! caller-owned manifold contract before exact semantic validation.

use rand::RngExt;

use crate::{ArmAlgorithmError, CollisionUnchecked};

use super::{
    ArmPlan, ArmStateValidator, IdentityProjection, ManifoldProjector, PlannerConfig,
    PlannerEvidence, PlannerKind, PlannerStatistics, PreparedPlanning, build_plan,
    core::{
        Node, PlannerWorkspace, State, copy_position, edge_is_valid, join_bidirectional, nearest,
        push_node, sample_uniform, squared_distance, steer, total_nodes,
    },
};

enum Extend {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

#[derive(Default)]
struct ProjectionEvidence {
    calls: u32,
    iterations: u32,
    rejections: u32,
    connect_attempts: u32,
}

impl PreparedPlanning<'_, '_> {
    /// Plans in a caller-defined manifold using bounded deterministic projection.
    #[allow(clippy::too_many_arguments)]
    pub fn plan_constrained(
        &self,
        start: &message_model::ArmJointVector<message_model::ArmPosition>,
        goal: &message_model::ArmJointVector<message_model::ArmPosition>,
        config: PlannerConfig,
        projector: &impl ManifoldProjector,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
        workspace.reset(self, config)?;
        require_generation(self, projector)?;
        let start = copy_position(self, start)?;
        let goal = copy_position(self, goal)?;
        let mut evidence = ProjectionEvidence::default();
        require_endpoint(self, projector, start, 0, validator, &mut evidence)?;
        require_endpoint(self, projector, goal, 1, validator, &mut evidence)?;

        let statistics = if edge_is_valid(self, &start, &goal, config.edge_resolution, validator) {
            super::core::interpolate_edge(
                &start,
                &goal,
                config.step_size,
                self.dof(),
                &mut workspace.path,
            )?;
            PlannerStatistics::new(0, 0, 2, PlannerEvidence::DirectEdge)
        } else {
            solve_projected(
                self, start, goal, config, projector, validator, workspace, evidence,
            )?
        };
        build_plan(
            self,
            PlannerKind::ConstrainedRrt,
            config.seed,
            &workspace.path,
            statistics,
        )
    }
}

pub(super) fn solve_identity(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    let projector = IdentityProjection::new(prepared.constraint_generation())?;
    solve_projected(
        prepared,
        start,
        goal,
        config,
        &projector,
        validator,
        workspace,
        ProjectionEvidence::default(),
    )
}

#[allow(clippy::too_many_arguments)]
fn solve_projected(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    config: PlannerConfig,
    projector: &impl ManifoldProjector,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
    mut evidence: ProjectionEvidence,
) -> Result<PlannerStatistics, ArmAlgorithmError> {
    require_generation(prepared, projector)?;
    workspace.first.push(Node::root(start));
    workspace.second.push(Node::root(goal));
    let mut first_from_start = true;
    let mut samples = 0_u32;
    let mut completed_iterations = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        completed_iterations = iteration;
        if samples >= config.maximum_samples
            || total_nodes(workspace) >= config.maximum_nodes as usize
        {
            break;
        }
        let mut raw = State::zero();
        if workspace.rng.random::<f64>() < config.goal_bias {
            raw = workspace.second[0].state;
        } else {
            sample_uniform(prepared, &mut workspace.rng, &mut raw);
            samples += 1;
        }
        let mut sample = State::zero();
        if !project_search_sample(
            prepared,
            projector,
            &raw,
            &mut sample,
            samples as usize,
            validator,
            &mut evidence,
        ) {
            continue;
        }

        let other_len = workspace.second.len();
        if let Extend::Reached(first_index) | Extend::Advanced(first_index) = extend(
            prepared,
            &mut workspace.first,
            other_len,
            &sample,
            config,
            projector,
            validator,
            &mut evidence,
        ) {
            evidence.connect_attempts = evidence.connect_attempts.saturating_add(1);
            let target = workspace.first[first_index].state;
            let first_len = workspace.first.len();
            if let Some(second_index) = connect(
                prepared,
                &mut workspace.second,
                first_len,
                &target,
                config,
                projector,
                validator,
                &mut evidence,
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
                    PlannerEvidence::ConstrainedRrt {
                        projection_calls: evidence.calls,
                        projection_iterations: evidence.iterations,
                        projection_rejections: evidence.rejections,
                        connect_attempts: evidence.connect_attempts,
                    },
                ));
            }
        }
        std::mem::swap(&mut workspace.first, &mut workspace.second);
        first_from_start = !first_from_start;
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: PlannerKind::ConstrainedRrt.name(),
        iterations: completed_iterations,
        samples,
        nodes: total_nodes(workspace) as u32,
    })
}

#[allow(clippy::too_many_arguments)]
fn extend(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    projector: &impl ManifoldProjector,
    validator: &impl ArmStateValidator,
    evidence: &mut ProjectionEvidence,
) -> Extend {
    let nearest_index = nearest(nodes, target, prepared.dof());
    let parent = nodes[nearest_index];
    let mut raw = State::zero();
    steer(
        &parent.state,
        target,
        config.step_size,
        prepared.dof(),
        &mut raw,
    );
    let mut next = State::zero();
    if !project_search_sample(
        prepared,
        projector,
        &raw,
        &mut next,
        nodes.len(),
        validator,
        evidence,
    ) || squared_distance(&parent.state, &next, prepared.dof()) <= f64::EPSILON
        || !edge_is_valid(
            prepared,
            &parent.state,
            &next,
            config.edge_resolution,
            validator,
        )
    {
        return Extend::Trapped;
    }
    let reached = squared_distance(&next, target, prepared.dof()) <= 1.0e-18;
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

#[allow(clippy::too_many_arguments)]
fn connect(
    prepared: &PreparedPlanning<'_, '_>,
    nodes: &mut Vec<Node>,
    other_nodes: usize,
    target: &State,
    config: PlannerConfig,
    projector: &impl ManifoldProjector,
    validator: &impl ArmStateValidator,
    evidence: &mut ProjectionEvidence,
) -> Option<usize> {
    loop {
        match extend(
            prepared,
            nodes,
            other_nodes,
            target,
            config,
            projector,
            validator,
            evidence,
        ) {
            Extend::Reached(index) => return Some(index),
            Extend::Advanced(_) => {}
            Extend::Trapped => return None,
        }
        if nodes.len() + other_nodes >= config.maximum_nodes as usize {
            return None;
        }
    }
}

fn require_generation(
    prepared: &PreparedPlanning<'_, '_>,
    projector: &impl ManifoldProjector,
) -> Result<(), ArmAlgorithmError> {
    if projector.generation() != prepared.constraint_generation() {
        return Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: prepared.constraint_generation(),
            actual: projector.generation(),
        });
    }
    Ok(())
}

fn require_endpoint(
    prepared: &PreparedPlanning<'_, '_>,
    projector: &impl ManifoldProjector,
    endpoint: State,
    sample_index: usize,
    validator: &impl ArmStateValidator,
    evidence: &mut ProjectionEvidence,
) -> Result<(), ArmAlgorithmError> {
    let mut projected = State::zero();
    evidence.calls = evidence.calls.saturating_add(1);
    let iterations = projector
        .project(
            endpoint.as_slice(prepared.dof()),
            projected.as_mut_slice(prepared.dof()),
        )
        .map_err(|reason| ArmAlgorithmError::ProjectionFailed {
            sample_index,
            iterations: 0,
            reason,
        })?;
    evidence.iterations = evidence.iterations.saturating_add(iterations);
    if squared_distance(&endpoint, &projected, prepared.dof()) > 1.0e-18 {
        return Err(ArmAlgorithmError::ProjectionFailed {
            sample_index,
            iterations,
            reason: "endpoint_not_on_manifold",
        });
    }
    super::core::validate_state(prepared, &projected, validator, sample_index).map_err(|_| {
        ArmAlgorithmError::ProjectionFailed {
            sample_index,
            iterations,
            reason: "projected_endpoint_invalid",
        }
    })
}

fn project_search_sample(
    prepared: &PreparedPlanning<'_, '_>,
    projector: &impl ManifoldProjector,
    input: &State,
    output: &mut State,
    sample_index: usize,
    validator: &impl ArmStateValidator,
    evidence: &mut ProjectionEvidence,
) -> bool {
    evidence.calls = evidence.calls.saturating_add(1);
    let Ok(iterations) = projector.project(
        input.as_slice(prepared.dof()),
        output.as_mut_slice(prepared.dof()),
    ) else {
        evidence.rejections = evidence.rejections.saturating_add(1);
        return false;
    };
    evidence.iterations = evidence.iterations.saturating_add(iterations);
    if super::core::validate_state(prepared, output, validator, sample_index).is_err() {
        evidence.rejections = evidence.rejections.saturating_add(1);
        return false;
    }
    true
}
