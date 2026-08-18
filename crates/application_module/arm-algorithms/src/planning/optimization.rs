//! Prepared contracts and fixed-capacity storage for trajectory optimization.

use std::mem::size_of;

use message_model::{ArmProfileId, KinematicGroupId, MAX_TRAJECTORY_POINTS};
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PreparedPlanning,
    core::{State, edge_is_valid, sample_uniform, validate_state},
};

/// Caller-owned deterministic point cost and gradient contract.
pub trait TrajectoryCost {
    /// Returns the semantic constraint/cost generation.
    fn generation(&self) -> u64;

    /// Evaluates one finite joint-space state without allocation.
    fn evaluate(&self, state_si: &[f64]) -> Result<f64, &'static str>;

    /// Writes one state gradient without allocating.
    fn gradient(&self, state_si: &[f64], output: &mut [f64]) -> Result<(), &'static str>;
}

/// Zero additional point cost; optimizers still minimize their smoothness term.
#[derive(Clone, Copy, Debug)]
pub struct SmoothnessOnly {
    generation: u64,
}

impl SmoothnessOnly {
    /// Binds the objective to the same non-zero semantic generation as planning.
    pub fn new(generation: u64) -> Result<Self, ArmAlgorithmError> {
        if generation == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.cost_generation",
            });
        }
        Ok(Self { generation })
    }
}

impl TrajectoryCost for SmoothnessOnly {
    fn generation(&self) -> u64 {
        self.generation
    }

    fn evaluate(&self, _state_si: &[f64]) -> Result<f64, &'static str> {
        Ok(0.0)
    }

    fn gradient(&self, _state_si: &[f64], output: &mut [f64]) -> Result<(), &'static str> {
        output.fill(0.0);
        Ok(())
    }
}

/// Caller-owned bounded projection onto a semantic manifold.
pub trait ManifoldProjector {
    /// Returns the constraint generation represented by this projector.
    fn generation(&self) -> u64;

    /// Projects `input` into caller-owned `output`, returning work iterations.
    fn project(&self, input: &[f64], output: &mut [f64]) -> Result<u32, &'static str>;
}

/// Identity manifold useful when the validator itself defines feasibility.
#[derive(Clone, Copy, Debug)]
pub struct IdentityProjection {
    generation: u64,
}

impl IdentityProjection {
    /// Creates an identity projection with a non-zero semantic generation.
    pub fn new(generation: u64) -> Result<Self, ArmAlgorithmError> {
        if generation == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.projection_generation",
            });
        }
        Ok(Self { generation })
    }
}

impl ManifoldProjector for IdentityProjection {
    fn generation(&self) -> u64 {
        self.generation
    }

    fn project(&self, input: &[f64], output: &mut [f64]) -> Result<u32, &'static str> {
        if input.len() != output.len() {
            return Err("projection_dimension");
        }
        output.copy_from_slice(input);
        Ok(0)
    }
}

/// Shared deterministic CHOMP/STOMP preparation configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OptimizationConfig {
    pub(crate) maximum_iterations: u32,
    pub(crate) waypoint_count: usize,
    pub(crate) learning_rate: f64,
    pub(crate) tolerance: f64,
    pub(crate) population_size: usize,
    pub(crate) noise_magnitude: f64,
    pub(crate) temperature: f64,
    pub(crate) seed: [u8; 32],
}

impl OptimizationConfig {
    /// Validates work, shape and numerical bounds.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        maximum_iterations: u32,
        waypoint_count: usize,
        learning_rate: f64,
        tolerance: f64,
        population_size: usize,
        noise_magnitude: f64,
        temperature: f64,
        seed: [u8; 32],
    ) -> Result<Self, ArmAlgorithmError> {
        if maximum_iterations == 0 || maximum_iterations > 1_000_000 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.optimization_iterations",
            });
        }
        if !(3..=MAX_TRAJECTORY_POINTS).contains(&waypoint_count) {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.optimization_waypoints",
                actual: waypoint_count,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        if !(2..=256).contains(&population_size) {
            return Err(ArmAlgorithmError::Capacity {
                field: "planner.stomp_population",
                actual: population_size,
                maximum: 256,
            });
        }
        if !learning_rate.is_finite()
            || learning_rate <= 0.0
            || !tolerance.is_finite()
            || tolerance <= 0.0
            || !noise_magnitude.is_finite()
            || noise_magnitude <= 0.0
            || !temperature.is_finite()
            || temperature <= 0.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.optimization_config",
            });
        }
        Ok(Self {
            maximum_iterations,
            waypoint_count,
            learning_rate,
            tolerance,
            population_size,
            noise_magnitude,
            temperature,
            seed,
        })
    }

    /// Returns conservative nominal defaults.
    #[must_use]
    pub const fn host_default(seed: [u8; 32]) -> Self {
        Self {
            maximum_iterations: 32,
            waypoint_count: 12,
            learning_rate: 0.05,
            tolerance: 1.0e-5,
            population_size: 12,
            noise_magnitude: 0.08,
            temperature: 10.0,
            seed,
        }
    }

    /// Returns the bounded iteration count.
    #[must_use]
    pub const fn maximum_iterations(self) -> u32 {
        self.maximum_iterations
    }

    /// Returns the exact waypoint shape.
    #[must_use]
    pub const fn waypoint_count(self) -> usize {
        self.waypoint_count
    }

    /// Returns the deterministic seed.
    #[must_use]
    pub const fn seed(self) -> [u8; 32] {
        self.seed
    }
}

/// Reusable prepared scratch for CHOMP and STOMP calls.
pub struct OptimizationWorkspace {
    pub(crate) profile_id: ArmProfileId,
    pub(crate) group_id: KinematicGroupId,
    pub(crate) model_generation: u64,
    pub(crate) constraint_generation: u64,
    pub(crate) dof: usize,
    pub(crate) waypoint_count: usize,
    pub(crate) population_size: usize,
    pub(crate) trajectory: Vec<State>,
    pub(crate) candidate: Vec<State>,
    pub(crate) gradient: Vec<State>,
    pub(crate) noises: Vec<State>,
    pub(crate) sample_costs: Vec<f64>,
    pub(crate) weights: Vec<f64>,
    pub(crate) rng: ChaCha8Rng,
}

impl OptimizationWorkspace {
    /// Allocates exact shape/population scratch once.
    pub fn new(
        prepared: &PreparedPlanning<'_, '_>,
        config: OptimizationConfig,
    ) -> Result<Self, ArmAlgorithmError> {
        let noise_count = config
            .waypoint_count
            .checked_mul(config.population_size)
            .ok_or(ArmAlgorithmError::InvalidBudget {
                field: "planner.optimization_workspace",
            })?;
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            dof: prepared.dof(),
            waypoint_count: config.waypoint_count,
            population_size: config.population_size,
            trajectory: Vec::with_capacity(config.waypoint_count),
            candidate: Vec::with_capacity(config.waypoint_count),
            gradient: Vec::with_capacity(config.waypoint_count),
            noises: Vec::with_capacity(noise_count),
            sample_costs: Vec::with_capacity(config.population_size),
            weights: Vec::with_capacity(config.population_size),
            rng: ChaCha8Rng::from_seed(config.seed),
        })
    }

    /// Returns the storage reserved during optimizer workspace preparation.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.trajectory.capacity() * size_of::<State>()
            + self.candidate.capacity() * size_of::<State>()
            + self.gradient.capacity() * size_of::<State>()
            + self.noises.capacity() * size_of::<State>()
            + self.sample_costs.capacity() * size_of::<f64>()
            + self.weights.capacity() * size_of::<f64>()
    }

    pub(crate) fn reset(
        &mut self,
        prepared: &PreparedPlanning<'_, '_>,
        config: OptimizationConfig,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || self.dof != prepared.dof()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "optimizer profile, group or shape changed",
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
        if self.waypoint_count != config.waypoint_count
            || self.population_size < config.population_size
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "optimizer waypoint or population shape changed",
            });
        }
        self.trajectory.clear();
        self.candidate.clear();
        self.gradient.clear();
        self.noises.clear();
        self.sample_costs.clear();
        self.weights.clear();
        self.rng = ChaCha8Rng::from_seed(config.seed);
        Ok(())
    }
}

pub(crate) fn require_cost_generation(
    prepared: &PreparedPlanning<'_, '_>,
    cost: &impl TrajectoryCost,
) -> Result<(), ArmAlgorithmError> {
    if cost.generation() != prepared.constraint_generation() {
        return Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: prepared.constraint_generation(),
            actual: cost.generation(),
        });
    }
    Ok(())
}

pub(crate) fn trajectory_cost(
    prepared: &PreparedPlanning<'_, '_>,
    trajectory: &[State],
    cost: &impl TrajectoryCost,
) -> Result<f64, ArmAlgorithmError> {
    let mut total = 0.0;
    for state in trajectory {
        let value = cost
            .evaluate(state.as_slice(prepared.dof()))
            .map_err(|reason| ArmAlgorithmError::ConstraintViolation {
                sample_index: 0,
                reason,
            })?;
        if !value.is_finite() || value < 0.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.trajectory_cost",
            });
        }
        total += value;
    }
    for window in trajectory.windows(3) {
        for joint in 0..prepared.dof() {
            let acceleration = window[2].as_slice(prepared.dof())[joint]
                - 2.0 * window[1].as_slice(prepared.dof())[joint]
                + window[0].as_slice(prepared.dof())[joint];
            total += acceleration * acceleration;
        }
    }
    Ok(total)
}

pub(crate) fn trajectory_valid(
    prepared: &PreparedPlanning<'_, '_>,
    trajectory: &[State],
    resolution: f64,
    validator: &impl ArmStateValidator,
) -> bool {
    trajectory
        .iter()
        .enumerate()
        .all(|(index, state)| validate_state(prepared, state, validator, index).is_ok())
        && trajectory
            .windows(2)
            .all(|window| edge_is_valid(prepared, &window[0], &window[1], resolution, validator))
}

pub(crate) fn prepare_feasible_seed(
    prepared: &PreparedPlanning<'_, '_>,
    start: State,
    goal: State,
    planner_config: PlannerConfig,
    optimization_config: OptimizationConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut OptimizationWorkspace,
) -> Result<u32, ArmAlgorithmError> {
    let count = optimization_config.waypoint_count;
    let mut samples = 0_u32;
    while samples < planner_config.maximum_samples {
        let mut middle = State::zero();
        sample_uniform(prepared, &mut workspace.rng, &mut middle);
        samples += 1;
        workspace.trajectory.clear();
        for index in 0..count {
            let alpha = index as f64 / (count - 1) as f64;
            let mut state = State::zero();
            for joint in 0..prepared.dof() {
                let start_value = start.as_slice(prepared.dof())[joint];
                let middle_value = middle.as_slice(prepared.dof())[joint];
                let goal_value = goal.as_slice(prepared.dof())[joint];
                state.as_mut_slice(prepared.dof())[joint] = (1.0 - alpha).powi(2) * start_value
                    + 2.0 * (1.0 - alpha) * alpha * middle_value
                    + alpha.powi(2) * goal_value;
            }
            workspace.trajectory.push(state);
        }
        if trajectory_valid(
            prepared,
            &workspace.trajectory,
            planner_config.edge_resolution,
            validator,
        ) {
            return Ok(samples);
        }
    }
    Err(ArmAlgorithmError::WorkBudgetExhausted {
        planner: "optimization_seed",
        iterations: 0,
        samples,
        nodes: 0,
    })
}
