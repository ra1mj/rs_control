//! Project-owned, morphology-aware mechanical-arm algorithms.
//!
//! Algorithms consume immutable [`arm_model::ArmModel`] profiles and semantic
//! SI values. They do not grant authority, perform I/O, or emit hardware
//! frames.

pub mod dynamics;
mod error;
mod evidence;
pub mod grasp;
pub mod kinematics;
pub mod planning;
pub mod reactive;
pub mod task;
pub mod trajectory;

pub use error::*;
pub use evidence::*;
