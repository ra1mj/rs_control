//! Deterministic inverse-kinematics selection and damped least squares.

use message_model::{ArmJointVector, ArmPosition};
use nalgebra::{DMatrix, DVector};

use super::{ArmPose, KinematicChain};
use crate::ArmAlgorithmError;

/// Stable inverse-kinematics solver selection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IkSolverKind {
    /// Deterministically choose an available compatible solver.
    Auto,
    /// Generic damped least-squares numerical IK.
    DampedLeastSquares,
    /// Geometry-based forward/backward reaching.
    Fabrik,
    /// Sequential quadratic-programming IK.
    Sqp,
    /// Multi-objective bio-inspired IK.
    BioIk,
    /// OPW analytical six-axis IK.
    Opw,
    /// Paden-Kahan/subproblem analytical six-axis IK.
    PadenKahan,
    /// Compatible redundant seven-axis analytical path.
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
                maximum_dof: message_model::MAX_JOINTS,
                supports_prismatic: true,
                requires_specialized_geometry: false,
            },
            IkSolverKind::DampedLeastSquares => Self {
                solver,
                implemented: true,
                minimum_dof: 1,
                maximum_dof: message_model::MAX_JOINTS,
                supports_prismatic: true,
                requires_specialized_geometry: false,
            },
            IkSolverKind::Fabrik | IkSolverKind::Sqp | IkSolverKind::BioIk => Self {
                solver,
                implemented: false,
                minimum_dof: 2,
                maximum_dof: message_model::MAX_JOINTS,
                supports_prismatic: false,
                requires_specialized_geometry: false,
            },
            IkSolverKind::Opw | IkSolverKind::PadenKahan => Self {
                solver,
                implemented: false,
                minimum_dof: 6,
                maximum_dof: 6,
                supports_prismatic: false,
                requires_specialized_geometry: true,
            },
            IkSolverKind::RedundantSevenDof => Self {
                solver,
                implemented: false,
                minimum_dof: 7,
                maximum_dof: 7,
                supports_prismatic: false,
                requires_specialized_geometry: true,
            },
        }
    }

    /// Returns the solver.
    #[must_use]
    pub const fn solver(self) -> IkSolverKind {
        self.solver
    }

    /// Returns whether this crate currently contains the implementation.
    #[must_use]
    pub const fn implemented(self) -> bool {
        self.implemented
    }

    /// Returns the inclusive supported DOF interval.
    #[must_use]
    pub const fn dof_range(self) -> [usize; 2] {
        [self.minimum_dof, self.maximum_dof]
    }

    /// Returns whether prismatic joints are supported.
    #[must_use]
    pub const fn supports_prismatic(self) -> bool {
        self.supports_prismatic
    }

    /// Returns whether a specific geometry analysis is required.
    #[must_use]
    pub const fn requires_specialized_geometry(self) -> bool {
        self.requires_specialized_geometry
    }
}

/// Validated bounded configuration for deterministic DLS IK.
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

/// Converged inverse-kinematics solution with reproducible evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct IkSolution {
    solver: IkSolverKind,
    positions: ArmJointVector<ArmPosition>,
    iterations: u32,
    position_error_m: f64,
    orientation_error_rad: f64,
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
        self.iterations
    }

    /// Returns final translation residual in metres.
    #[must_use]
    pub const fn position_error_m(&self) -> f64 {
        self.position_error_m
    }

    /// Returns final orientation residual in radians.
    #[must_use]
    pub const fn orientation_error_rad(&self) -> f64 {
        self.orientation_error_rad
    }
}

impl KinematicChain<'_> {
    /// Solves one pose target with deterministic solver selection.
    pub fn solve_ik(
        self,
        solver: IkSolverKind,
        target: ArmPose,
        initial: &ArmJointVector<ArmPosition>,
        config: DampedLeastSquaresConfig,
    ) -> Result<IkSolution, ArmAlgorithmError> {
        let selected = match solver {
            IkSolverKind::Auto => IkSolverKind::DampedLeastSquares,
            explicit => explicit,
        };
        let capabilities = SolverCapabilities::for_solver(selected);
        if !capabilities.implemented() {
            return Err(ArmAlgorithmError::UnsupportedCapability {
                capability: selected.name(),
            });
        }
        self.solve_damped_least_squares(target, initial, config, selected)
    }

    fn solve_damped_least_squares(
        self,
        target: ArmPose,
        initial: &ArmJointVector<ArmPosition>,
        config: DampedLeastSquaresConfig,
        selected: IkSolverKind,
    ) -> Result<IkSolution, ArmAlgorithmError> {
        let mut coordinates = self.values(initial)?;
        let target_transform = target.to_isometry();
        let mut last_position_error = f64::MAX;
        let mut last_orientation_error = f64::MAX;

        for iteration in 0..=config.max_iterations {
            let positions = self.positions_from_values(&coordinates)?;
            let current = self.evaluate(&positions)?.tool_transform;
            let translation_error =
                target_transform.translation.vector - current.translation.vector;
            let orientation_error =
                (target_transform.rotation * current.rotation.inverse()).scaled_axis();
            last_position_error = translation_error.norm();
            last_orientation_error = orientation_error.norm();
            if last_position_error <= config.position_tolerance_m
                && last_orientation_error <= config.orientation_tolerance_rad
            {
                return Ok(IkSolution {
                    solver: selected,
                    positions,
                    iterations: iteration,
                    position_error_m: last_position_error,
                    orientation_error_rad: last_orientation_error,
                });
            }
            if iteration == config.max_iterations {
                break;
            }
            let jacobian = self.jacobian(&positions)?.as_matrix();
            let error = DVector::from_iterator(
                6,
                translation_error
                    .iter()
                    .chain(orientation_error.iter())
                    .copied(),
            );
            let regularized =
                &jacobian * jacobian.transpose() + DMatrix::identity(6, 6) * config.damping.powi(2);
            let solved = regularized.lu().solve(&error).ok_or_else(|| {
                let analysis = self
                    .jacobian(&positions)
                    .and_then(|value| value.analyze(config.damping))
                    .ok();
                ArmAlgorithmError::Singular {
                    rank: analysis.map_or(0, |value| value.rank()),
                    required_rank: analysis.map_or(6, |value| value.required_rank()),
                }
            })?;
            let mut step = jacobian.transpose() * solved;
            let norm = step.norm();
            if norm > config.max_step_norm {
                step *= config.max_step_norm / norm;
            }
            if step.iter().any(|value| !value.is_finite()) {
                return Err(ArmAlgorithmError::NonFinite { field: "ik.step" });
            }
            for (coordinate, delta) in coordinates.iter_mut().zip(step.iter().copied()) {
                *coordinate += delta;
            }
            project_position_limits(self, &mut coordinates)?;
        }

        Err(ArmAlgorithmError::NonConvergent {
            iterations: config.max_iterations,
            position_error_m: last_position_error,
            orientation_error_rad: last_orientation_error,
        })
    }
}

fn project_position_limits(
    chain: KinematicChain<'_>,
    coordinates: &mut [f64],
) -> Result<(), ArmAlgorithmError> {
    if coordinates.len() != chain.dof() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "ik.coordinates",
            expected: chain.dof(),
            actual: coordinates.len(),
        });
    }
    for (descriptor, coordinate) in chain.group().joint_order().joints().iter().zip(coordinates) {
        let joint =
            chain
                .model()
                .joint(descriptor.joint_id())
                .ok_or(ArmAlgorithmError::InvalidModel {
                    reason: "group order references missing joint",
                })?;
        if let Some([lower, upper]) = joint.limits().and_then(|limits| limits.position()) {
            *coordinate = coordinate.clamp(lower, upper);
        }
    }
    Ok(())
}
