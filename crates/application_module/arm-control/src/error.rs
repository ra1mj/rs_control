//! Typed failures owned by the arm-control admission boundary.

use arm_model::TransmissionError;
use control_core::LeaseCacheError;
use message_model::{ArmActuatorId, SemanticError};
use thiserror::Error;

/// Arm proposal admission or host-execution failure.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ArmControlError {
    /// The configured group is absent or lacks a complete actuator mapping.
    #[error("arm-control configuration is invalid: {reason}")]
    InvalidConfiguration {
        /// Static configuration failure class.
        reason: &'static str,
    },
    /// Profile, group, order, source, boot or generation identity differs.
    #[error("arm-control identity mismatch for {field}")]
    IdentityMismatch {
        /// Rejected identity field.
        field: &'static str,
    },
    /// Observation sequence did not advance within the expected boot.
    #[error("arm observation sequence {actual} did not advance past {previous}")]
    StaleSequence {
        /// Last admitted sequence.
        previous: u64,
        /// Rejected sequence.
        actual: u64,
    },
    /// The source-local observation validity window is closed.
    #[error("arm observation is stale at {now_monotonic_ns} ns")]
    StaleObservation {
        /// Admission time in the local monotonic domain.
        now_monotonic_ns: u64,
    },
    /// Observation quality or clock domain cannot authorize control.
    #[error("arm observation is not a valid local-clock sample")]
    InvalidObservation,
    /// Robot or action lifecycle does not permit a new control output.
    #[error("arm execution is closed by {field}")]
    ExecutionClosed {
        /// Lifecycle gate that is not open.
        field: &'static str,
    },
    /// Proposal payload mode or degree of freedom is inconsistent.
    #[error("arm proposal payload is invalid: {reason}")]
    InvalidProposal {
        /// Static payload failure class.
        reason: &'static str,
    },
    /// An actuator field exceeded its immutable magnitude bound.
    #[error("actuator {actuator} field {field} value {value} exceeds magnitude {maximum}")]
    LimitExceeded {
        /// Stable actuator identity.
        actuator: ArmActuatorId,
        /// Rejected output field.
        field: &'static str,
        /// Rejected value.
        value: f64,
        /// Accepted magnitude.
        maximum: f64,
    },
    /// An actuator field exceeded its time-normalized slew bound.
    #[error("actuator {actuator} field {field} slew {actual} exceeds {maximum}")]
    SlewExceeded {
        /// Stable actuator identity.
        actuator: ArmActuatorId,
        /// Rejected output field.
        field: &'static str,
        /// Observed absolute change per second.
        actual: f64,
        /// Accepted absolute change per second.
        maximum: f64,
    },
    /// Caller-owned execution storage is full.
    #[error("arm execution log capacity {capacity} is exhausted")]
    Capacity {
        /// Fixed caller-owned capacity.
        capacity: usize,
    },
    /// Joint-to-actuator projection failed.
    #[error(transparent)]
    Transmission(#[from] TransmissionError),
    /// Semantic value construction failed.
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    /// Existing local lease/deadline enforcement rejected the command.
    #[error(transparent)]
    Lease(#[from] LeaseCacheError),
}
