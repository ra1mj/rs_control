//! Typed algorithm failures with bounded, actionable evidence.

use message_model::{ArmJointId, KinematicGroupId};
use thiserror::Error;

/// Mechanical-arm algorithm failure.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ArmAlgorithmError {
    /// The selected group is not present in the compiled profile.
    #[error("kinematic group {group} does not exist")]
    UnknownGroup {
        /// Rejected group.
        group: KinematicGroupId,
    },
    /// Input profile/group/order does not match the compiled model.
    #[error("joint order does not match compiled group {group}")]
    JointOrderMismatch {
        /// Expected group.
        group: KinematicGroupId,
    },
    /// A vector or matrix had the wrong bounded dimension.
    #[error("{field} dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch {
        /// Rejected value class.
        field: &'static str,
        /// Required size.
        expected: usize,
        /// Observed size.
        actual: usize,
    },
    /// A scalar input or numerical result was NaN or infinite.
    #[error("{field} contains a non-finite value")]
    NonFinite {
        /// Rejected value class.
        field: &'static str,
    },
    /// A tagged SI value did not match its joint coordinate family.
    #[error("joint {joint} uses the wrong SI coordinate family")]
    UnitMismatch {
        /// Rejected joint.
        joint: ArmJointId,
    },
    /// A joint position exceeded the compiled finite interval.
    #[error("joint {joint} position {value} is outside [{lower}, {upper}]")]
    JointLimit {
        /// Rejected joint.
        joint: ArmJointId,
        /// Rejected coordinate.
        value: f64,
        /// Inclusive lower bound.
        lower: f64,
        /// Inclusive upper bound.
        upper: f64,
    },
    /// A mimic dependency could not be resolved in the selected group.
    #[error("mimic joint {joint} references unavailable source {source_joint}")]
    MimicSourceUnavailable {
        /// Dependent joint.
        joint: Box<ArmJointId>,
        /// Missing independent source.
        source_joint: Box<ArmJointId>,
    },
    /// A model index or path invariant was violated.
    #[error("compiled model invariant failed: {reason}")]
    InvalidModel {
        /// Static invariant category.
        reason: &'static str,
    },
    /// The requested algorithm does not support this geometry.
    #[error("solver {solver} is incompatible with the selected geometry: {reason}")]
    UnsupportedGeometry {
        /// Stable solver name.
        solver: &'static str,
        /// Static compatibility evidence.
        reason: &'static str,
    },
    /// A deliberately deferred morphology/capability was requested.
    #[error("capability {capability} is not implemented in this delivery")]
    UnsupportedCapability {
        /// Stable capability name.
        capability: &'static str,
    },
    /// Iterative work ended without meeting tolerances.
    #[error(
        "solver did not converge after {iterations} iterations (position {position_error_m} m, orientation {orientation_error_rad} rad)"
    )]
    NonConvergent {
        /// Completed iterations.
        iterations: u32,
        /// Final translation residual.
        position_error_m: f64,
        /// Final rotation residual.
        orientation_error_rad: f64,
    },
    /// A matrix required by the numerical method was singular.
    #[error("numerical system is singular at rank {rank} (required {required_rank})")]
    Singular {
        /// Observed numerical rank.
        rank: usize,
        /// Required rank.
        required_rank: usize,
    },
    /// Explicit sampling/iteration/node work budget is zero or invalid.
    #[error("{field} work budget must be positive and bounded")]
    InvalidBudget {
        /// Rejected budget.
        field: &'static str,
    },
}
