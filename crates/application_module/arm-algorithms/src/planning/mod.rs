//! Deterministic bounded joint-space planning with collision-unchecked typestate.

use std::marker::PhantomData;

use message_model::{
    ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId,
    MAX_TRAJECTORY_POINTS,
};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

use crate::{ArmAlgorithmError, CollisionUnchecked, kinematics::KinematicChain};

/// Project planning topology request.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlanningTopology {
    /// One named fixed-base open-chain group.
    SingleGroup,
    /// Coordinated dual-arm composite request.
    CoordinatedDualArm,
    /// Floating-base request.
    FloatingBase,
    /// Closed-chain or parallel-mechanism request.
    ParallelMechanism,
}

impl PlanningTopology {
    /// Rejects deferred morphologies without reducing them to a single chain.
    pub fn require_supported(self) -> Result<(), ArmAlgorithmError> {
        match self {
            Self::SingleGroup => Ok(()),
            Self::CoordinatedDualArm => Err(ArmAlgorithmError::UnsupportedCapability {
                capability: "coordinated_dual_arm_planning",
            }),
            Self::FloatingBase => Err(ArmAlgorithmError::UnsupportedCapability {
                capability: "floating_base_planning",
            }),
            Self::ParallelMechanism => Err(ArmAlgorithmError::UnsupportedCapability {
                capability: "parallel_mechanism_planning",
            }),
        }
    }
}

/// Stable planner surface.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PlannerKind {
    /// Bidirectional rapidly exploring random trees.
    RrtConnect,
    /// Rewiring RRT surface.
    RrtStar,
    /// Bidirectional rewiring RRT surface.
    BidirectionalRrtStar,
    /// Bidirectional transition-test RRT surface.
    BiTrrt,
    /// Expansive-space-tree surface.
    Est,
    /// Projection/cell exploration surface.
    Kpiece,
    /// Probabilistic-roadmap surface.
    Prm,
    /// Graph-of-convex-sets surface.
    Gcs,
    /// Iterative regional-inflation surface.
    Iris,
    /// Covariant trajectory-optimization surface.
    Chomp,
    /// Stochastic trajectory-optimization surface.
    Stomp,
    /// Explicit constrained RRT surface.
    ConstrainedRrt,
}

impl PlannerKind {
    /// Returns a stable audit name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::RrtConnect => "rrt_connect",
            Self::RrtStar => "rrt_star",
            Self::BidirectionalRrtStar => "bidirectional_rrt_star",
            Self::BiTrrt => "bitrrt",
            Self::Est => "est",
            Self::Kpiece => "kpiece",
            Self::Prm => "prm",
            Self::Gcs => "gcs",
            Self::Iris => "iris",
            Self::Chomp => "chomp",
            Self::Stomp => "stomp",
            Self::ConstrainedRrt => "constrained_rrt",
        }
    }

    const fn uses_sampling(self) -> bool {
        matches!(
            self,
            Self::RrtConnect
                | Self::RrtStar
                | Self::BidirectionalRrtStar
                | Self::BiTrrt
                | Self::Est
                | Self::Kpiece
                | Self::Prm
                | Self::ConstrainedRrt
                | Self::Stomp
        )
    }
}

/// Explicit deterministic planner work and interpolation bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlannerConfig {
    seed: [u8; 32],
    maximum_iterations: u32,
    maximum_samples: u32,
    maximum_nodes: u32,
    maximum_duration_ns: u64,
    step_size: f64,
    edge_resolution: f64,
    goal_bias: f64,
}

impl PlannerConfig {
    /// Validates all work and geometric bounds.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        seed: [u8; 32],
        maximum_iterations: u32,
        maximum_samples: u32,
        maximum_nodes: u32,
        maximum_duration_ns: u64,
        step_size: f64,
        edge_resolution: f64,
        goal_bias: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if maximum_iterations == 0
            || maximum_samples == 0
            || maximum_nodes < 2
            || maximum_iterations > 10_000_000
            || maximum_samples > 10_000_000
            || maximum_nodes > 1_000_000
        {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.work",
            });
        }
        if maximum_duration_ns == 0 {
            return Err(ArmAlgorithmError::InvalidTime {
                field: "planner.maximum_duration_ns",
            });
        }
        if !step_size.is_finite()
            || step_size <= 0.0
            || !edge_resolution.is_finite()
            || edge_resolution <= 0.0
            || !goal_bias.is_finite()
            || !(0.0..=1.0).contains(&goal_bias)
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.geometry",
            });
        }
        Ok(Self {
            seed,
            maximum_iterations,
            maximum_samples,
            maximum_nodes,
            maximum_duration_ns,
            step_size,
            edge_resolution,
            goal_bias,
        })
    }

    /// Returns deterministic host defaults.
    #[must_use]
    pub const fn host_default(seed: [u8; 32]) -> Self {
        Self {
            seed,
            maximum_iterations: 5_000,
            maximum_samples: 5_000,
            maximum_nodes: 10_000,
            maximum_duration_ns: 5_000_000_000,
            step_size: 0.15,
            edge_resolution: 0.03,
            goal_bias: 0.1,
        }
    }

    /// Returns the explicit seed.
    #[must_use]
    pub const fn seed(self) -> [u8; 32] {
        self.seed
    }

    /// Returns the wall-duration admission budget.
    ///
    /// The deterministic kernel does not read a clock; the caller enforces
    /// this independent deadline around the bounded work loop.
    #[must_use]
    pub const fn maximum_duration_ns(self) -> u64 {
        self.maximum_duration_ns
    }
}

/// Project state constraint independent of collision checking.
pub trait ArmStateValidator {
    /// Returns a static evidence category for an invalid state.
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str>;
}

/// Validator that adds no constraint beyond compiled joint limits.
#[derive(Clone, Copy, Debug, Default)]
pub struct JointLimitsOnly;

impl ArmStateValidator for JointLimitsOnly {
    fn validate(&self, _state_si: &[f64]) -> Result<(), &'static str> {
        Ok(())
    }
}

/// Deterministic planner accounting.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PlannerStatistics {
    iterations: u32,
    samples: u32,
    nodes: u32,
}

impl PlannerStatistics {
    /// Returns completed main-loop iterations.
    #[must_use]
    pub const fn iterations(self) -> u32 {
        self.iterations
    }

    /// Returns consumed random samples.
    #[must_use]
    pub const fn samples(self) -> u32 {
        self.samples
    }

    /// Returns created tree/roadmap nodes.
    #[must_use]
    pub const fn nodes(self) -> u32 {
        self.nodes
    }
}

/// Bounded morphology-bound plan carrying collision evidence as typestate.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmPlan<Evidence> {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    planner: PlannerKind,
    seed: [u8; 32],
    waypoints: Vec<ArmJointVector<ArmPosition>>,
    statistics: PlannerStatistics,
    _evidence: PhantomData<Evidence>,
}

impl<Evidence> ArmPlan<Evidence> {
    /// Returns the immutable profile.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the selected group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns the requested planner surface.
    #[must_use]
    pub const fn planner(&self) -> PlannerKind {
        self.planner
    }

    /// Returns the exact deterministic seed.
    #[must_use]
    pub const fn seed(&self) -> &[u8; 32] {
        &self.seed
    }

    /// Returns bounded joint-space waypoints.
    #[must_use]
    pub fn waypoints(&self) -> &[ArmJointVector<ArmPosition>] {
        &self.waypoints
    }

    /// Returns deterministic accounting.
    #[must_use]
    pub const fn statistics(&self) -> PlannerStatistics {
        self.statistics
    }
}

/// Plans one collision-unchecked single-group path.
pub fn plan_joint_space(
    chain: KinematicChain<'_>,
    planner: PlannerKind,
    start: &ArmJointVector<ArmPosition>,
    goal: &ArmJointVector<ArmPosition>,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
    PlanningTopology::SingleGroup.require_supported()?;
    let start_values = chain.values(start)?;
    let goal_values = chain.values(goal)?;
    validate_state(chain, &start_values, validator, 0)?;
    validate_state(chain, &goal_values, validator, 1)?;

    if edge_is_valid(
        chain,
        &start_values,
        &goal_values,
        config.edge_resolution,
        validator,
    ) {
        let raw = interpolate_edge(&start_values, &goal_values, config.step_size);
        return build_plan(
            chain,
            planner,
            config.seed,
            raw,
            PlannerStatistics {
                iterations: 0,
                samples: 0,
                nodes: 2,
            },
        );
    }
    if !planner.uses_sampling() {
        return Err(ArmAlgorithmError::ConstraintViolation {
            sample_index: 0,
            reason: "direct optimization seed violates a constraint",
        });
    }
    bidirectional_expand(
        chain,
        planner,
        &start_values,
        &goal_values,
        config,
        validator,
    )
}

impl ArmPlan<CollisionUnchecked> {
    /// Deterministically removes unnecessary waypoints using the same validator.
    pub fn shortcut(
        &self,
        chain: KinematicChain<'_>,
        edge_resolution: f64,
        validator: &impl ArmStateValidator,
    ) -> Result<Self, ArmAlgorithmError> {
        ensure_plan_context(self, chain)?;
        if !edge_resolution.is_finite() || edge_resolution <= 0.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.shortcut_resolution",
            });
        }
        let raw = self
            .waypoints
            .iter()
            .map(|point| chain.values(point))
            .collect::<Result<Vec<_>, _>>()?;
        let mut shortened = Vec::new();
        let mut current = 0;
        shortened.push(raw[0].clone());
        while current + 1 < raw.len() {
            let mut next = raw.len() - 1;
            while next > current + 1
                && !edge_is_valid(chain, &raw[current], &raw[next], edge_resolution, validator)
            {
                next -= 1;
            }
            shortened.push(raw[next].clone());
            current = next;
        }
        build_plan(chain, self.planner, self.seed, shortened, self.statistics)
    }
}

#[derive(Clone)]
struct TreeNode {
    state: Vec<f64>,
    parent: Option<usize>,
}

struct SearchTree {
    nodes: Vec<TreeNode>,
    from_start: bool,
}

fn bidirectional_expand(
    chain: KinematicChain<'_>,
    planner: PlannerKind,
    start: &[f64],
    goal: &[f64],
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
    let bounds = sampling_bounds(chain)?;
    let mut rng = ChaCha8Rng::from_seed(config.seed);
    let mut first = SearchTree {
        nodes: vec![TreeNode {
            state: start.to_vec(),
            parent: None,
        }],
        from_start: true,
    };
    let mut second = SearchTree {
        nodes: vec![TreeNode {
            state: goal.to_vec(),
            parent: None,
        }],
        from_start: false,
    };
    let mut samples = 0_u32;

    for iteration in 1..=config.maximum_iterations {
        if samples >= config.maximum_samples
            || first.nodes.len() + second.nodes.len() >= config.maximum_nodes as usize
        {
            break;
        }
        let target_root = &second.nodes[0].state;
        let sample = if rng.random::<f64>() < config.goal_bias {
            target_root.clone()
        } else {
            samples += 1;
            bounds
                .iter()
                .map(|[lower, upper]| rng.random_range(*lower..=*upper))
                .collect()
        };
        if let Some(first_index) = extend_tree(
            chain,
            &mut first,
            &sample,
            config.step_size,
            config.edge_resolution,
            validator,
        ) {
            let connection = first.nodes[first_index].state.clone();
            if let Some(second_index) = connect_tree(
                chain,
                &mut second,
                &connection,
                config.step_size,
                config.edge_resolution,
                validator,
                config.maximum_nodes as usize - first.nodes.len(),
            ) {
                let raw = join_trees(&first, first_index, &second, second_index);
                return build_plan(
                    chain,
                    planner,
                    config.seed,
                    raw,
                    PlannerStatistics {
                        iterations: iteration,
                        samples,
                        nodes: (first.nodes.len() + second.nodes.len()) as u32,
                    },
                );
            }
        }
        std::mem::swap(&mut first, &mut second);
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: planner.name(),
        iterations: config.maximum_iterations,
        samples,
        nodes: (first.nodes.len() + second.nodes.len()) as u32,
    })
}

fn extend_tree(
    chain: KinematicChain<'_>,
    tree: &mut SearchTree,
    target: &[f64],
    step_size: f64,
    edge_resolution: f64,
    validator: &impl ArmStateValidator,
) -> Option<usize> {
    let nearest = tree
        .nodes
        .iter()
        .enumerate()
        .min_by(|(_, left), (_, right)| {
            squared_distance(&left.state, target).total_cmp(&squared_distance(&right.state, target))
        })?
        .0;
    let next = steer(&tree.nodes[nearest].state, target, step_size);
    if !edge_is_valid(
        chain,
        &tree.nodes[nearest].state,
        &next,
        edge_resolution,
        validator,
    ) {
        return None;
    }
    tree.nodes.push(TreeNode {
        state: next,
        parent: Some(nearest),
    });
    Some(tree.nodes.len() - 1)
}

#[allow(clippy::too_many_arguments)]
fn connect_tree(
    chain: KinematicChain<'_>,
    tree: &mut SearchTree,
    target: &[f64],
    step_size: f64,
    edge_resolution: f64,
    validator: &impl ArmStateValidator,
    remaining_capacity: usize,
) -> Option<usize> {
    for _ in 0..remaining_capacity {
        let index = extend_tree(chain, tree, target, step_size, edge_resolution, validator)?;
        if squared_distance(&tree.nodes[index].state, target).sqrt() <= step_size
            && edge_is_valid(
                chain,
                &tree.nodes[index].state,
                target,
                edge_resolution,
                validator,
            )
        {
            tree.nodes.push(TreeNode {
                state: target.to_vec(),
                parent: Some(index),
            });
            return Some(tree.nodes.len() - 1);
        }
    }
    None
}

fn join_trees(
    first: &SearchTree,
    first_index: usize,
    second: &SearchTree,
    second_index: usize,
) -> Vec<Vec<f64>> {
    let first_path = trace(first, first_index);
    let second_path = trace(second, second_index);
    let (mut start_path, goal_path) = if first.from_start {
        (first_path, second_path)
    } else {
        (second_path, first_path)
    };
    start_path.extend(goal_path.into_iter().rev().skip(1));
    start_path
}

fn trace(tree: &SearchTree, mut index: usize) -> Vec<Vec<f64>> {
    let mut path = Vec::new();
    loop {
        path.push(tree.nodes[index].state.clone());
        let Some(parent) = tree.nodes[index].parent else {
            break;
        };
        index = parent;
    }
    path.reverse();
    path
}

fn build_plan(
    chain: KinematicChain<'_>,
    planner: PlannerKind,
    seed: [u8; 32],
    raw: Vec<Vec<f64>>,
    statistics: PlannerStatistics,
) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
    if raw.len() < 2 || raw.len() > MAX_TRAJECTORY_POINTS {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.waypoints",
            actual: raw.len(),
            maximum: MAX_TRAJECTORY_POINTS,
        });
    }
    let waypoints = raw
        .iter()
        .map(|state| chain.positions_from_values(state))
        .collect::<Result<Vec<_>, _>>()?;
    let _ = CollisionUnchecked::new();
    Ok(ArmPlan {
        profile_id: chain.model().profile_id(),
        group_id: chain.group().id(),
        order: chain.group().joint_order().clone(),
        planner,
        seed,
        waypoints,
        statistics,
        _evidence: PhantomData,
    })
}

fn ensure_plan_context(
    plan: &ArmPlan<CollisionUnchecked>,
    chain: KinematicChain<'_>,
) -> Result<(), ArmAlgorithmError> {
    if plan.profile_id != chain.model().profile_id()
        || plan.group_id != chain.group().id()
        || &plan.order != chain.group().joint_order()
    {
        Err(ArmAlgorithmError::JointOrderMismatch {
            group: chain.group().id(),
        })
    } else {
        Ok(())
    }
}

fn validate_state(
    chain: KinematicChain<'_>,
    state: &[f64],
    validator: &impl ArmStateValidator,
    sample_index: usize,
) -> Result<(), ArmAlgorithmError> {
    let positions = chain.positions_from_values(state)?;
    chain.forward(&positions)?;
    validator
        .validate(state)
        .map_err(|reason| ArmAlgorithmError::ConstraintViolation {
            sample_index,
            reason,
        })
}

fn edge_is_valid(
    chain: KinematicChain<'_>,
    start: &[f64],
    goal: &[f64],
    resolution: f64,
    validator: &impl ArmStateValidator,
) -> bool {
    let distance = squared_distance(start, goal).sqrt();
    let steps = (distance / resolution).ceil().max(1.0) as usize;
    (0..=steps).all(|index| {
        let alpha = index as f64 / steps as f64;
        let state = interpolate(start, goal, alpha);
        validate_state(chain, &state, validator, index).is_ok()
    })
}

fn interpolate_edge(start: &[f64], goal: &[f64], step_size: f64) -> Vec<Vec<f64>> {
    let distance = squared_distance(start, goal).sqrt();
    let steps = (distance / step_size).ceil().max(1.0) as usize;
    (0..=steps)
        .map(|index| interpolate(start, goal, index as f64 / steps as f64))
        .collect()
}

fn interpolate(start: &[f64], goal: &[f64], alpha: f64) -> Vec<f64> {
    start
        .iter()
        .zip(goal)
        .map(|(start, goal)| (goal - start).mul_add(alpha, *start))
        .collect()
}

fn steer(start: &[f64], target: &[f64], maximum_step: f64) -> Vec<f64> {
    let distance = squared_distance(start, target).sqrt();
    if distance <= maximum_step {
        target.to_vec()
    } else {
        interpolate(start, target, maximum_step / distance)
    }
}

fn squared_distance(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| (right - left).powi(2))
        .sum()
}

fn sampling_bounds(chain: KinematicChain<'_>) -> Result<Vec<[f64; 2]>, ArmAlgorithmError> {
    chain
        .group()
        .joint_order()
        .joints()
        .iter()
        .map(|descriptor| {
            let joint = chain.model().joint(descriptor.joint_id()).ok_or(
                ArmAlgorithmError::InvalidModel {
                    reason: "planning group references missing joint",
                },
            )?;
            Ok(joint
                .limits()
                .and_then(|limits| limits.position())
                .unwrap_or([-std::f64::consts::PI, std::f64::consts::PI]))
        })
        .collect()
}
