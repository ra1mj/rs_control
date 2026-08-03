//! Deterministic bounded joint-space planning with collision-unchecked typestate.

mod bi_rrt_star;
mod bitrrt;
mod core;
mod est;
mod kpiece;
mod prm;
mod rrt_connect;
mod rrt_star;

use std::marker::PhantomData;

use message_model::{
    ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId,
    MAX_TRAJECTORY_POINTS,
};

use crate::{
    ArmAlgorithmError, CollisionUnchecked,
    kinematics::{KinematicChain, PreparedKinematics},
};

pub use core::{PlannerWorkspace, PreparedPlanning};
pub use prm::{PreparedRoadmap, PrmBuildStatistics, PrmQueryOptions};

use core::{State, copy_position, edge_is_valid, interpolate_edge, validate_state};

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

    const fn is_delivered_sampling_kernel(self) -> bool {
        matches!(
            self,
            Self::RrtConnect
                | Self::RrtStar
                | Self::BidirectionalRrtStar
                | Self::BiTrrt
                | Self::Est
                | Self::Kpiece
                | Self::Prm
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

    /// Returns the iteration ceiling.
    #[must_use]
    pub const fn maximum_iterations(self) -> u32 {
        self.maximum_iterations
    }

    /// Returns the random-sample ceiling.
    #[must_use]
    pub const fn maximum_samples(self) -> u32 {
        self.maximum_samples
    }

    /// Returns the aggregate search-node ceiling.
    #[must_use]
    pub const fn maximum_nodes(self) -> u32 {
        self.maximum_nodes
    }

    /// Returns the wall-duration admission budget.
    ///
    /// Kernels do not read a clock; orchestration enforces this deadline.
    #[must_use]
    pub const fn maximum_duration_ns(self) -> u64 {
        self.maximum_duration_ns
    }

    /// Returns maximum extension distance in joint-space SI coordinates.
    #[must_use]
    pub const fn step_size(self) -> f64 {
        self.step_size
    }

    /// Returns state-check resolution along an edge.
    #[must_use]
    pub const fn edge_resolution(self) -> f64 {
        self.edge_resolution
    }

    /// Returns goal-directed sampling probability.
    #[must_use]
    pub const fn goal_bias(self) -> f64 {
        self.goal_bias
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

/// Auditable proof that the requested family, rather than a shared alias, ran.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PlannerEvidence {
    /// Common valid-edge optimization; the named kernel was not needed.
    DirectEdge,
    /// Alternating extend/connect search.
    RrtConnect { connect_attempts: u32 },
    /// Single-tree parent selection and rewiring.
    RrtStar {
        parent_candidates: u32,
        rewires: u32,
        best_cost: f64,
        informed_samples: u32,
    },
    /// Two independently rewired trees.
    BidirectionalRrtStar {
        parent_candidates: u32,
        rewires: u32,
        connection_candidates: u32,
        best_cost: f64,
    },
    /// Boltzmann transition-test state.
    BiTrrt {
        transitions: u32,
        rejected_transitions: u32,
        frustration_resets: u32,
        final_temperature: f64,
    },
    /// Inverse-density expansion state.
    Est {
        density_evaluations: u32,
        sparse_selections: u32,
    },
    /// Projected-cell exploration state.
    Kpiece {
        cells: u32,
        border_selections: u32,
        successful_expansions: u32,
        failed_expansions: u32,
    },
    /// Prepared graph construction/query state.
    Prm {
        roadmap_nodes: u32,
        roadmap_edges: u32,
        nodes_expanded: u32,
        lazy_edge_checks: u32,
    },
}

/// Deterministic planner accounting.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlannerStatistics {
    iterations: u32,
    samples: u32,
    nodes: u32,
    evidence: PlannerEvidence,
}

impl PlannerStatistics {
    pub(crate) const fn new(
        iterations: u32,
        samples: u32,
        nodes: u32,
        evidence: PlannerEvidence,
    ) -> Self {
        Self {
            iterations,
            samples,
            nodes,
            evidence,
        }
    }

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

    /// Returns family-specific execution evidence.
    #[must_use]
    pub const fn evidence(self) -> PlannerEvidence {
        self.evidence
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

impl PreparedPlanning<'_, '_> {
    /// Executes a named planner using caller-reused prepared storage.
    pub fn plan(
        &self,
        planner: PlannerKind,
        start: &ArmJointVector<ArmPosition>,
        goal: &ArmJointVector<ArmPosition>,
        config: PlannerConfig,
        validator: &impl ArmStateValidator,
        workspace: &mut PlannerWorkspace,
    ) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
        PlanningTopology::SingleGroup.require_supported()?;
        workspace.reset(self, config)?;
        let start = copy_position(self, start)?;
        let goal = copy_position(self, goal)?;
        validate_state(self, &start, validator, 0)?;
        validate_state(self, &goal, validator, 1)?;

        let statistics = if edge_is_valid(self, &start, &goal, config.edge_resolution, validator) {
            interpolate_edge(
                &start,
                &goal,
                config.step_size,
                self.dof(),
                &mut workspace.path,
            )?;
            PlannerStatistics::new(0, 0, 2, PlannerEvidence::DirectEdge)
        } else {
            if !planner.is_delivered_sampling_kernel() {
                return Err(ArmAlgorithmError::UnsupportedCapability {
                    capability: "optimization_planner_kernel",
                });
            }
            match planner {
                PlannerKind::RrtConnect => {
                    rrt_connect::solve(self, start, goal, config, validator, workspace)?
                }
                PlannerKind::RrtStar => {
                    rrt_star::solve(self, start, goal, config, validator, workspace)?
                }
                PlannerKind::BidirectionalRrtStar => {
                    bi_rrt_star::solve(self, start, goal, config, validator, workspace)?
                }
                PlannerKind::BiTrrt => {
                    bitrrt::solve(self, start, goal, config, validator, workspace)?
                }
                PlannerKind::Est => est::solve(self, start, goal, config, validator, workspace)?,
                PlannerKind::Kpiece => {
                    kpiece::solve(self, start, goal, config, validator, workspace)?
                }
                PlannerKind::Prm => prm::solve(self, start, goal, config, validator, workspace)?,
                _ => unreachable!("non-delivered planner rejected above"),
            }
        };
        build_plan(self, planner, config.seed, &workspace.path, statistics)
    }
}

/// Compatibility one-shot planner. Repeated callers should retain prepared objects.
pub fn plan_joint_space(
    chain: KinematicChain<'_>,
    planner: PlannerKind,
    start: &ArmJointVector<ArmPosition>,
    goal: &ArmJointVector<ArmPosition>,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
    let kinematics = PreparedKinematics::new(chain, 1)?;
    let prepared = PreparedPlanning::new(&kinematics, 1)?;
    let mut workspace = PlannerWorkspace::new(&prepared, config)?;
    prepared.plan(planner, start, goal, config, validator, &mut workspace)
}

impl ArmPlan<CollisionUnchecked> {
    /// Deterministically removes unnecessary waypoints using the same validator.
    pub fn shortcut(
        &self,
        chain: KinematicChain<'_>,
        edge_resolution: f64,
        validator: &impl ArmStateValidator,
    ) -> Result<Self, ArmAlgorithmError> {
        if !edge_resolution.is_finite() || edge_resolution <= 0.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.shortcut_resolution",
            });
        }
        let kinematics = PreparedKinematics::new(chain, 1)?;
        let prepared = PreparedPlanning::new(&kinematics, 1)?;
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || &self.order != prepared.order()
        {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: prepared.group_id(),
            });
        }
        let raw = self
            .waypoints
            .iter()
            .map(|point| copy_position(&prepared, point))
            .collect::<Result<Vec<_>, _>>()?;
        let mut shortened = Vec::with_capacity(raw.len());
        let mut current = 0;
        shortened.push(raw[0]);
        while current + 1 < raw.len() {
            let mut next = raw.len() - 1;
            while next > current + 1
                && !edge_is_valid(
                    &prepared,
                    &raw[current],
                    &raw[next],
                    edge_resolution,
                    validator,
                )
            {
                next -= 1;
            }
            shortened.push(raw[next]);
            current = next;
        }
        build_plan(
            &prepared,
            self.planner,
            self.seed,
            &shortened,
            self.statistics,
        )
    }
}

fn build_plan(
    prepared: &PreparedPlanning<'_, '_>,
    planner: PlannerKind,
    seed: [u8; 32],
    raw: &[State],
    statistics: PlannerStatistics,
) -> Result<ArmPlan<CollisionUnchecked>, ArmAlgorithmError> {
    if raw.len() < 2 || raw.len() > MAX_TRAJECTORY_POINTS {
        return Err(ArmAlgorithmError::Capacity {
            field: "planner.waypoints",
            actual: raw.len(),
            maximum: MAX_TRAJECTORY_POINTS,
        });
    }
    let chain = prepared.kinematics.chain();
    let waypoints = raw
        .iter()
        .map(|state| chain.positions_from_values(state.as_slice(prepared.dof())))
        .collect::<Result<Vec<_>, _>>()?;
    let _ = CollisionUnchecked::new();
    Ok(ArmPlan {
        profile_id: prepared.profile_id(),
        group_id: prepared.group_id(),
        order: prepared.order().clone(),
        planner,
        seed,
        waypoints,
        statistics,
        _evidence: PhantomData,
    })
}
