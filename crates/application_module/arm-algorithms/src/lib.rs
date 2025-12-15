//! Project-owned, morphology-aware mechanical-arm algorithms.
//!
//! Algorithms consume immutable [`arm_model::ArmModel`] profiles and semantic
//! SI values. They do not grant authority, perform I/O, or emit hardware
//! frames.

mod error;
pub mod kinematics;

pub use error::*;
