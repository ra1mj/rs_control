// SPDX-License-Identifier: Apache-2.0
//
// Project-native API and admission layer for IK kernels adapted exclusively
// from softmata/kinetic commit 3dbaa61579c4359ce4c35f246fc83e1d3110b646.

//! Deterministic prepared inverse kinematics.

mod analytical;
mod bio;
mod cache;
mod dls;
mod fabrik;
mod opw;
mod paden_kahan;
mod prepared;
mod redundant;
mod sqp;

use message_model::{ArmJointVector, ArmPosition, MAX_JOINTS};
use nalgebra::Isometry3;

pub use cache::{IkCache, MAX_IK_CACHE_ENTRIES};
pub use prepared::{
    GeometryCapabilities, IkWorkspace, MAX_IK_CANDIDATES, MAX_IK_POPULATION, PreparedKinematics,
};

use super::{ArmPose, KinematicChain};
use crate::ArmAlgorithmError;

/// Stable inverse-kinematics solver selection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IkSolverKind {
    /// Deterministically choose the fastest compatible KINETIC-derived solver.
    Auto,
    /// Damped least-squares numerical IK.
    DampedLeastSquares,
    /// Forward/backward reaching followed by bounded orientation refinement.
    Fabrik,
    /// Sequential quadratic-programming IK.
    Sqp,
    /// Seeded multi-objective evolutionary IK.
    BioIk,
    /// OPW analytical six-axis IK.
    Opw,
    /// Paden-Kahan/subproblem analytical six-axis IK.
    PadenKahan,
    /// Redundant seven-axis subproblem path.
    RedundantSevenDof,
}

impl IkSolverKind {
    /// Returns the stable audit/log name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::Auto => "auto",
            Self::DampedLeastSquares => "damped_least_squares",
            Self::Fabrik => "fabrik",
            Self::Sqp => "sqp",
            Self::BioIk => "bio_ik",
            Self::Opw => "opw",
            Self::PadenKahan => "paden_kahan",
            Self::RedundantSevenDof => "redundant_7dof",
        }
    }
}

/// Truthful implementation and geometry capability report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SolverCapabilities {
    solver: IkSolverKind,
    implemented: bool,
    minimum_dof: usize,
    maximum_dof: usize,
    supports_prismatic: bool,
    requires_specialized_geometry: bool,
    requires_explicit_seed: bool,
}

impl SolverCapabilities {
    /// Returns static capabilities for one solver.
    #[must_use]
    pub const fn for_solver(solver: IkSolverKind) -> Self {
        match solver {
            IkSolverKind::Auto => Self {
                solver,
                implemented: true,
                minimum_dof: 1,
                maximum_dof: MAX_JOINTS,
                supports_prismatic: true,
                requires_specialized_geometry: false,
                requires_explicit_seed: false,
            },
            IkSolverKind::DampedLeastSquares | IkSolverKind::Sqp => Self {
                solver,
                implemented: true,
                minimum_dof: 1,
                maximum_dof: MAX_JOINTS,
                supports_prismatic: true,
                requires_specialized_geometry: false,
                requires_explicit_seed: false,
            },
            IkSolverKind::Fabrik => Self {
                solver,
                implemented: true,
                minimum_dof: 2,
                maximum_dof: MAX_JOINTS,
                supports_prismatic: false,
                requires_specialized_geometry: false,
                requires_explicit_seed: false,
            },
            IkSolverKind::BioIk => Self {
                solver,
                implemented: true,
                minimum_dof: 2,
                maximum_dof: MAX_JOINTS,
                supports_prismatic: true,
                requires_specialized_geometry: false,
                requires_explicit_seed: true,
            },
            IkSolverKind::Opw | IkSolverKind::PadenKahan => Self {
                solver,
                implemented: true,
                minimum_dof: 6,
                maximum_dof: 6,
                supports_prismatic: false,
                requires_specialized_geometry: true,
                requires_explicit_seed: false,
            },
            IkSolverKind::RedundantSevenDof => Self {
                solver,
                implemented: true,
                minimum_dof: 7,
                maximum_dof: 7,
                supports_prismatic: false,
                requires_specialized_geometry: true,
                requires_explicit_seed: true,
            },
        }
    }

    /// Returns the solver.
    #[must_use]
    pub const fn solver(self) -> IkSolverKind {
        self.solver
    }

    /// Returns whether this crate contains the named implementation.
    #[must_use]
    pub const fn implemented(self) -> bool {
        self.implemented
    }

    /// Returns the inclusive supported DOF interval.
    #[must_use]
    pub const fn dof_range(self) -> [usize; 2] {
        [self.minimum_dof, self.maximum_dof]
    }

    /// Returns whether prismatic coordinates are supported.
    #[must_use]
    pub const fn supports_prismatic(self) -> bool {
        self.supports_prismatic
    }

    /// Returns whether geometry analysis is mandatory.
    #[must_use]
    pub const fn requires_specialized_geometry(self) -> bool {
        self.requires_specialized_geometry
    }

    /// Returns whether a caller-recorded random seed is mandatory.
    #[must_use]
    pub const fn requires_explicit_seed(self) -> bool {
        self.requires_explicit_seed
    }
}

/// Backward-compatible validated DLS configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DampedLeastSquaresConfig {
    max_iterations: u32,
    position_tolerance_m: f64,
    orientation_tolerance_rad: f64,
    damping: f64,
    max_step_norm: f64,
}

impl DampedLeastSquaresConfig {
    /// Creates a finite, positive and explicitly bounded work configuration.
    pub fn new(
        max_iterations: u32,
        position_tolerance_m: f64,
        orientation_tolerance_rad: f64,
        damping: f64,
        max_step_norm: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        validate_common_config(
            max_iterations,
            position_tolerance_m,
            orientation_tolerance_rad,
            damping,
            max_step_norm,
        )?;
        Ok(Self {
            max_iterations,
            position_tolerance_m,
            orientation_tolerance_rad,
            damping,
            max_step_norm,
        })
    }

    /// Returns a conservative deterministic host default.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            max_iterations: 250,
            position_tolerance_m: 1.0e-5,
            orientation_tolerance_rad: 1.0e-5,
            damping: 1.0e-3,
            max_step_norm: 0.2,
        }
    }
}

/// Complete deterministic configuration for prepared KINETIC-derived solvers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IkSolveConfig {
    max_iterations: u32,
    position_tolerance_m: f64,
    orientation_tolerance_rad: f64,
    damping: f64,
    max_step_norm: f64,
    seed: [u8; 32],
    population_size: usize,
    elite_count: usize,
    redundant_samples: usize,
}

impl IkSolveConfig {
    /// Validates numerical, work, population and analytical-sampling bounds.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        max_iterations: u32,
        position_tolerance_m: f64,
        orientation_tolerance_rad: f64,
        damping: f64,
        max_step_norm: f64,
        seed: [u8; 32],
        population_size: usize,
        elite_count: usize,
        redundant_samples: usize,
    ) -> Result<Self, ArmAlgorithmError> {
        validate_common_config(
            max_iterations,
            position_tolerance_m,
            orientation_tolerance_rad,
            damping,
            max_step_norm,
        )?;
        if !(2..=MAX_IK_POPULATION).contains(&population_size) {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.population_size",
                actual: population_size,
                maximum: MAX_IK_POPULATION,
            });
        }
        if elite_count == 0 || elite_count >= population_size {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "ik.elite_count",
            });
        }
        if redundant_samples == 0 || redundant_samples > 4_096 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "ik.redundant_samples",
            });
        }
        Ok(Self {
            max_iterations,
            position_tolerance_m,
            orientation_tolerance_rad,
            damping,
            max_step_norm,
            seed,
            population_size,
            elite_count,
            redundant_samples,
        })
    }

    /// Returns deterministic host defaults with an explicit caller seed.
    #[must_use]
    pub const fn host_default(seed: [u8; 32]) -> Self {
        Self {
            max_iterations: 250,
            position_tolerance_m: 1.0e-5,
            orientation_tolerance_rad: 1.0e-5,
            damping: 1.0e-3,
            max_step_norm: 0.2,
            seed,
            population_size: 50,
            elite_count: 2,
            redundant_samples: 36,
        }
    }

    const fn from_dls(config: DampedLeastSquaresConfig) -> Self {
        Self {
            max_iterations: config.max_iterations,
            position_tolerance_m: config.position_tolerance_m,
            orientation_tolerance_rad: config.orientation_tolerance_rad,
            damping: config.damping,
            max_step_norm: config.max_step_norm,
            seed: [0; 32],
            population_size: 50,
            elite_count: 2,
            redundant_samples: 36,
        }
    }

    /// Returns the recorded deterministic seed.
    #[must_use]
    pub const fn seed(self) -> [u8; 32] {
        self.seed
    }

    /// Returns the preparation-time Bio-IK population capacity.
    #[must_use]
    pub const fn population_size(self) -> usize {
        self.population_size
    }
}

/// Reproducible solver work evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IkWorkEvidence {
    iterations: u32,
    evaluations: u32,
    generations: u32,
    candidates: u32,
    seed: [u8; 32],
    cache_hit: bool,
}

impl IkWorkEvidence {
    /// Returns completed solver iterations.
    #[must_use]
    pub const fn iterations(self) -> u32 {
        self.iterations
    }

    /// Returns FK/objective evaluations.
    #[must_use]
    pub const fn evaluations(self) -> u32 {
        self.evaluations
    }

    /// Returns completed evolutionary generations.
    #[must_use]
    pub const fn generations(self) -> u32 {
        self.generations
    }

    /// Returns analytical/population candidates considered.
    #[must_use]
    pub const fn candidates(self) -> u32 {
        self.candidates
    }

    /// Returns the exact request seed.
    #[must_use]
    pub const fn seed(self) -> [u8; 32] {
        self.seed
    }

    /// Returns whether this solution came from the bounded prepared cache.
    #[must_use]
    pub const fn cache_hit(self) -> bool {
        self.cache_hit
    }
}

/// Converged inverse-kinematics solution with reproducible evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct IkSolution {
    solver: IkSolverKind,
    positions: ArmJointVector<ArmPosition>,
    position_error_m: f64,
    orientation_error_rad: f64,
    degraded: bool,
    work: IkWorkEvidence,
}

impl IkSolution {
    /// Returns the actual solver selected by `Auto` or requested explicitly.
    #[must_use]
    pub const fn solver(&self) -> IkSolverKind {
        self.solver
    }

    /// Returns stable-order tagged joint positions.
    #[must_use]
    pub const fn positions(&self) -> &ArmJointVector<ArmPosition> {
        &self.positions
    }

    /// Returns completed iterations.
    #[must_use]
    pub const fn iterations(&self) -> u32 {
        self.work.iterations
    }

    /// Returns final translation residual in metres.
    #[must_use]
    pub const fn position_error_m(&self) -> f64 {
        self.position_error_m
    }

    /// Returns final rotation residual in radians.
    #[must_use]
    pub const fn orientation_error_rad(&self) -> f64 {
        self.orientation_error_rad
    }

    /// Returns whether a numerical fallback degraded the solve.
    #[must_use]
    pub const fn degraded(&self) -> bool {
        self.degraded
    }

    /// Returns deterministic work and seed evidence.
    #[must_use]
    pub const fn work(&self) -> IkWorkEvidence {
        self.work
    }
}

pub(super) struct KernelOutcome {
    pub coordinates: [f64; MAX_JOINTS],
    pub iterations: u32,
    pub evaluations: u32,
    pub generations: u32,
    pub candidates: u32,
    pub position_error_m: f64,
    pub orientation_error_rad: f64,
    pub degraded: bool,
}

impl PreparedKinematics<'_> {
    /// Runs one warm, context-checked prepared solve.
    pub fn solve(
        &self,
        solver: IkSolverKind,
        target: ArmPose,
        initial: &ArmJointVector<ArmPosition>,
        config: IkSolveConfig,
        workspace: &mut IkWorkspace,
    ) -> Result<IkSolution, ArmAlgorithmError> {
        workspace.require_context(self)?;
        let mut seed_coordinates = [0.0; MAX_JOINTS];
        self.copy_positions(initial, &mut seed_coordinates)?;
        if config.population_size > workspace.maximum_population() {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.population_size",
                actual: config.population_size,
                maximum: workspace.maximum_population(),
            });
        }
        let selected = select_solver(self, solver)?;
        let outcome = match selected {
            IkSolverKind::DampedLeastSquares => {
                dls::solve(self, workspace, target, &seed_coordinates, config)?
            }
            IkSolverKind::Fabrik => {
                fabrik::solve(self, workspace, target, &seed_coordinates, config)?
            }
            IkSolverKind::Sqp => sqp::solve(self, workspace, target, &seed_coordinates, config)?,
            IkSolverKind::BioIk => bio::solve(self, workspace, target, &seed_coordinates, config)?,
            IkSolverKind::Opw => opw::solve(self, workspace, target, &seed_coordinates, config)?,
            IkSolverKind::PadenKahan => {
                paden_kahan::solve(self, workspace, target, &seed_coordinates, config)?
            }
            IkSolverKind::RedundantSevenDof => {
                redundant::solve(self, workspace, target, &seed_coordinates, config)?
            }
            IkSolverKind::Auto => {
                return Err(ArmAlgorithmError::InvalidModel {
                    reason: "auto IK selection did not resolve",
                });
            }
        };
        let positions = self
            .chain
            .positions_from_values(&outcome.coordinates[..self.dof()])?;
        Ok(IkSolution {
            solver: selected,
            positions,
            position_error_m: outcome.position_error_m,
            orientation_error_rad: outcome.orientation_error_rad,
            degraded: outcome.degraded,
            work: IkWorkEvidence {
                iterations: outcome.iterations,
                evaluations: outcome.evaluations,
                generations: outcome.generations,
                candidates: outcome.candidates,
                seed: config.seed,
                cache_hit: false,
            },
        })
    }

    /// Returns a deterministic cache hit or solves and inserts without
    /// implicit eviction. A full cache fails with typed capacity evidence.
    pub fn solve_cached(
        &self,
        solver: IkSolverKind,
        target: ArmPose,
        initial: &ArmJointVector<ArmPosition>,
        config: IkSolveConfig,
        workspace: &mut IkWorkspace,
        cache: &mut IkCache,
    ) -> Result<IkSolution, ArmAlgorithmError> {
        if let Some(solution) = cache.lookup(self, target)? {
            return Ok(solution);
        }
        let solution = self.solve(solver, target, initial, config, workspace)?;
        cache.insert(self, target, &solution)?;
        Ok(solution)
    }

    /// Solves a caller-owned stable-order batch without growing `output`.
    pub fn solve_batch(
        &self,
        solver: IkSolverKind,
        targets: &[ArmPose],
        initials: &[ArmJointVector<ArmPosition>],
        config: IkSolveConfig,
        workspace: &mut IkWorkspace,
        output: &mut Vec<IkSolution>,
    ) -> Result<(), ArmAlgorithmError> {
        if targets.len() != initials.len() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "ik.batch_initials",
                expected: targets.len(),
                actual: initials.len(),
            });
        }
        if output.capacity() < targets.len() {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.batch_output",
                actual: targets.len(),
                maximum: output.capacity(),
            });
        }
        output.clear();
        for (&target, initial) in targets.iter().zip(initials) {
            match self.solve(solver, target, initial, config, workspace) {
                Ok(solution) => output.push(solution),
                Err(error) => {
                    output.clear();
                    return Err(error);
                }
            }
        }
        Ok(())
    }
}

impl KinematicChain<'_> {
    /// Compatibility wrapper. Prepared callers should reuse
    /// [`PreparedKinematics`] and [`IkWorkspace`] directly.
    pub fn solve_ik(
        self,
        solver: IkSolverKind,
        target: ArmPose,
        initial: &ArmJointVector<ArmPosition>,
        config: DampedLeastSquaresConfig,
    ) -> Result<IkSolution, ArmAlgorithmError> {
        if matches!(
            solver,
            IkSolverKind::BioIk | IkSolverKind::RedundantSevenDof
        ) {
            return Err(ArmAlgorithmError::UnsupportedCapability {
                capability: "explicit_seed_required",
            });
        }
        let prepared = PreparedKinematics::new(self, 1)?;
        let solve_config = IkSolveConfig::from_dls(config);
        let mut workspace = prepared.workspace(solve_config.population_size)?;
        prepared.solve(solver, target, initial, solve_config, &mut workspace)
    }
}

fn select_solver(
    prepared: &PreparedKinematics<'_>,
    requested: IkSolverKind,
) -> Result<IkSolverKind, ArmAlgorithmError> {
    let selected = match requested {
        IkSolverKind::Auto => {
            if prepared.geometry().supports_six_axis_analysis()
                && SolverCapabilities::for_solver(IkSolverKind::Opw).implemented()
                && prepared.geometry().supports_opw()
            {
                IkSolverKind::Opw
            } else if prepared.geometry().supports_paden_kahan()
                && SolverCapabilities::for_solver(IkSolverKind::PadenKahan).implemented()
            {
                IkSolverKind::PadenKahan
            } else if prepared.geometry().supports_seven_axis_analysis()
                && SolverCapabilities::for_solver(IkSolverKind::RedundantSevenDof).implemented()
            {
                IkSolverKind::RedundantSevenDof
            } else {
                IkSolverKind::DampedLeastSquares
            }
        }
        explicit => explicit,
    };
    let capabilities = SolverCapabilities::for_solver(selected);
    if !capabilities.implemented() {
        return Err(ArmAlgorithmError::UnsupportedCapability {
            capability: selected.name(),
        });
    }
    let [minimum, maximum] = capabilities.dof_range();
    if !(minimum..=maximum).contains(&prepared.dof()) {
        return Err(ArmAlgorithmError::UnsupportedGeometry {
            solver: selected.name(),
            reason: "degree of freedom is outside the solver range",
        });
    }
    if !capabilities.supports_prismatic() && !prepared.geometry().all_rotary() {
        return Err(ArmAlgorithmError::UnsupportedGeometry {
            solver: selected.name(),
            reason: "solver requires rotary joints",
        });
    }
    let geometry_matches = match selected {
        IkSolverKind::Opw => prepared.geometry().supports_opw(),
        IkSolverKind::PadenKahan => prepared.geometry().supports_paden_kahan(),
        IkSolverKind::RedundantSevenDof => prepared.geometry().supports_seven_axis_analysis(),
        _ => true,
    };
    if !geometry_matches {
        return Err(ArmAlgorithmError::UnsupportedGeometry {
            solver: selected.name(),
            reason: "prepared analytical axis convention does not match",
        });
    }
    Ok(selected)
}

pub(super) fn pose_error(current: &Isometry3<f64>, target: ArmPose) -> ([f64; 6], f64, f64) {
    let target = target.to_isometry();
    let translation = target.translation.vector - current.translation.vector;
    let rotation = (target.rotation * current.rotation.inverse()).scaled_axis();
    let error = [
        translation.x,
        translation.y,
        translation.z,
        rotation.x,
        rotation.y,
        rotation.z,
    ];
    (error, translation.norm(), rotation.norm())
}

fn validate_common_config(
    max_iterations: u32,
    position_tolerance_m: f64,
    orientation_tolerance_rad: f64,
    damping: f64,
    max_step_norm: f64,
) -> Result<(), ArmAlgorithmError> {
    if max_iterations == 0 || max_iterations > 1_000_000 {
        return Err(ArmAlgorithmError::InvalidBudget {
            field: "ik.max_iterations",
        });
    }
    for (field, value) in [
        ("ik.position_tolerance_m", position_tolerance_m),
        ("ik.orientation_tolerance_rad", orientation_tolerance_rad),
        ("ik.damping", damping),
        ("ik.max_step_norm", max_step_norm),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(ArmAlgorithmError::NonFinite { field });
        }
    }
    Ok(())
}
