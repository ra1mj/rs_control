//! Typed arm-model compilation and transmission failures.

use message_model::{ArmProfileIdError, IdError};
use thiserror::Error;

use crate::UrdfParseError;

/// Failure while parsing or compiling an immutable arm profile.
#[derive(Debug, Error)]
pub enum ArmModelError {
    /// The source document exceeded the bounded parse budget.
    #[error("URDF byte length {actual} exceeds maximum {maximum}")]
    UrdfTooLarge {
        /// Observed source length.
        actual: usize,
        /// Accepted source length.
        maximum: usize,
    },
    /// The project-owned bounded parser rejected the document.
    #[error("URDF parse failed")]
    UrdfParse(#[from] UrdfParseError),
    /// A model collection exceeded its project-owned bound.
    #[error("{field} count {actual} exceeds maximum {maximum}")]
    Capacity {
        /// Rejected collection.
        field: &'static str,
        /// Observed count.
        actual: usize,
        /// Supported count.
        maximum: usize,
    },
    /// A required model collection was empty.
    #[error("{field} cannot be empty")]
    Empty {
        /// Rejected field.
        field: &'static str,
    },
    /// A name was not a portable bounded project identifier.
    #[error("{field} name {name:?} is invalid")]
    Identifier {
        /// Rejected name class.
        field: &'static str,
        /// Original source name.
        name: String,
        /// Identifier validation source.
        #[source]
        error: IdError,
    },
    /// A stable identity appeared more than once.
    #[error("duplicate {field} identifier {name}")]
    Duplicate {
        /// Rejected identity class.
        field: &'static str,
        /// Duplicate value.
        name: String,
    },
    /// The profile schema is not implemented.
    #[error("arm profile schema version {actual} is unsupported; expected {expected}")]
    UnsupportedSchema {
        /// Expected schema.
        expected: u32,
        /// Rejected schema.
        actual: u32,
    },
    /// The URDF used a topology not supported by the current model.
    #[error("joint {joint} uses unsupported type {joint_type}")]
    UnsupportedJointType {
        /// Rejected joint.
        joint: String,
        /// Stable type spelling.
        joint_type: &'static str,
    },
    /// A referenced link does not exist.
    #[error("joint or group {owner} references missing link {link}")]
    MissingLink {
        /// Referencing joint/group.
        owner: String,
        /// Missing link name.
        link: String,
    },
    /// A joint referenced the same link as parent and child.
    #[error("joint {joint} cannot connect link {link} to itself")]
    SelfEdge {
        /// Rejected joint.
        joint: String,
        /// Reused link.
        link: String,
    },
    /// A link had more than one parent joint.
    #[error("link {link} has multiple parent joints")]
    MultipleParents {
        /// Rejected child link.
        link: String,
    },
    /// The model did not contain exactly one root.
    #[error("arm model must contain exactly one root link, found {actual}")]
    RootCount {
        /// Observed roots.
        actual: usize,
    },
    /// The model contained a cycle.
    #[error("arm model contains a cycle at link {link}")]
    Cycle {
        /// Link where traversal repeated.
        link: String,
    },
    /// The model tree was not connected.
    #[error("arm model contains {unreachable} unreachable links")]
    Disconnected {
        /// Links not reachable from the root.
        unreachable: usize,
    },
    /// A numeric model field contained NaN or infinity.
    #[error("{field} for {owner} must be finite")]
    NonFinite {
        /// Rejected field.
        field: &'static str,
        /// Owning model object.
        owner: String,
    },
    /// An active joint axis was zero or invalid.
    #[error("joint {joint} axis must be finite and nonzero")]
    InvalidAxis {
        /// Rejected joint.
        joint: String,
    },
    /// A limit was missing, nonpositive, or internally inconsistent.
    #[error("joint {joint} has invalid {field}")]
    InvalidJointLimit {
        /// Rejected joint.
        joint: String,
        /// Limit class.
        field: &'static str,
    },
    /// A required acceleration limit is absent.
    #[error("active joint {joint} has no explicit acceleration limit")]
    MissingAccelerationLimit {
        /// Rejected joint.
        joint: String,
    },
    /// The profile supplied a limit for an unknown or fixed joint.
    #[error("acceleration limit references unsupported joint {joint}")]
    UnknownAccelerationJoint {
        /// Rejected joint.
        joint: String,
    },
    /// Link inertia was incomplete or physically invalid.
    #[error("link {link} has invalid inertia")]
    InvalidInertia {
        /// Rejected link.
        link: String,
    },
    /// A mimic declaration referenced an invalid source.
    #[error("joint {joint} has invalid mimic source {source_joint}")]
    InvalidMimic {
        /// Mimic joint.
        joint: String,
        /// Rejected source.
        source_joint: String,
    },
    /// Mimic relations formed a dependency cycle.
    #[error("mimic relation contains a cycle at joint {joint}")]
    MimicCycle {
        /// Repeated joint.
        joint: String,
    },
    /// A group did not describe a valid open path.
    #[error("group {group} has no open path from {base} to {tip}")]
    InvalidGroupPath {
        /// Rejected group.
        group: String,
        /// Requested base.
        base: String,
        /// Requested tip.
        tip: String,
    },
    /// The explicit active-joint order did not match the path.
    #[error("group {group} active-joint order does not match its base-to-tip path")]
    GroupOrderMismatch {
        /// Rejected group.
        group: String,
    },
    /// A transmission referenced an invalid joint or repeated an identity.
    #[error("invalid transmission for joint {joint}: {reason}")]
    InvalidTransmission {
        /// Rejected joint.
        joint: String,
        /// Static failure class.
        reason: &'static str,
    },
    /// Hash construction unexpectedly produced the reserved identity.
    #[error("compiled profile identity is invalid")]
    ProfileIdentity(#[from] ArmProfileIdError),
}

/// Failure while projecting between joint and rotary-actuator SI values.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum TransmissionError {
    /// A value used the wrong coordinate or effort family.
    #[error("transmission input has an incompatible SI unit")]
    UnitMismatch,
    /// Conversion produced NaN or infinity.
    #[error("transmission conversion produced a non-finite value")]
    NonFinite,
}
