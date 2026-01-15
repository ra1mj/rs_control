//! Typed algorithm failures with bounded, actionable evidence.

use message_model::{ArmJointId, KinematicGroupId};
use thiserror::Error;

/// Exact trajectory constraint class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrajectoryViolationKind {
    /// Position interval.
    Position,
    /// Absolute velocity.
    Velocity,
    /// Absolute acceleration.
    Acceleration,
    /// Absolute jerk.
    Jerk,
    /// Torque or force.
    Effort,
}

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
    /// A bounded collection exceeded its accepted capacity.
    #[error("{field} count {actual} exceeds maximum {maximum}")]
    Capacity {
        /// Rejected collection.
        field: &'static str,
        /// Observed count.
        actual: usize,
        /// Accepted maximum.
        maximum: usize,
    },
    /// A timestamp/duration was zero, unordered or outside its bound.
    #[error("{field} contains an invalid time value")]
    InvalidTime {
        /// Rejected time class.
        field: &'static str,
    },
    /// A generated or supplied trajectory violates an exact joint constraint.
    #[error(
        "trajectory point {point_index} joint {joint} violates {kind:?}: |{value}| exceeds {limit}"
    )]
    TrajectoryViolation {
        /// Constraint class.
        kind: TrajectoryViolationKind,
        /// Offending waypoint/sample.
        point_index: usize,
        /// Stable joint identity.
        joint: Box<ArmJointId>,
        /// Signed observed value.
        value: f64,
        /// Accepted absolute bound (or nearest position endpoint).
        limit: f64,
    },
    /// Required inertial model data is absent.
    #[error("dynamics unavailable: {reason}")]
    DynamicsUnavailable {
        /// Static fail-closed cause.
        reason: &'static str,
    },
    /// A deterministic planner exhausted its explicit work budget.
    #[error(
        "planner {planner} exhausted work budget after {iterations} iterations, {samples} samples and {nodes} nodes"
    )]
    WorkBudgetExhausted {
        /// Stable planner name.
        planner: &'static str,
        /// Consumed iterations.
        iterations: u32,
        /// Consumed random samples.
        samples: u32,
        /// Created tree/roadmap nodes.
        nodes: u32,
    },
    /// A supplied state or interpolated edge failed an explicit constraint.
    #[error("state constraint rejected sample {sample_index}: {reason}")]
    ConstraintViolation {
        /// Stable sample/waypoint index.
        sample_index: usize,
        /// Static validator evidence.
        reason: &'static str,
    },
    /// A collision-dependent policy was requested without a provider.
    #[error("collision capability is unavailable for {surface}")]
    CollisionCapabilityUnavailable {
        /// Rejected surface.
        surface: &'static str,
    },
}
