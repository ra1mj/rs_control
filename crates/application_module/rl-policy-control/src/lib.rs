//! Fail-closed host runtime for immutable trained arm-policy artifacts.
//!
//! Inference runs on a named non-realtime worker. The control-facing side only
//! exchanges bounded semantic observations and collision-unchecked proposals;
//! this crate has no lease, lifecycle, protocol or hardware authority.

mod action;
mod artifact;
mod engine;
mod error;
mod graph;
mod manifest;
mod observation;
mod worker;

pub use action::*;
pub use artifact::*;
pub use engine::*;
pub use error::*;
pub use manifest::*;
pub use observation::*;
pub use worker::*;

/// Maximum production policy artifact size.
pub const MAX_POLICY_MODEL_BYTES: u64 = 64 * 1024 * 1024;
/// Maximum manifest size.
pub const MAX_POLICY_MANIFEST_BYTES: u64 = 128 * 1024;
/// Maximum tensor elements exchanged with the control-facing API.
pub const MAX_POLICY_ELEMENTS: usize = message_model::MAX_JOINTS * 3;
/// Maximum flattened recurrent state elements.
pub const MAX_POLICY_RECURRENT_ELEMENTS: usize = 1024;
/// Maximum total graph inputs or outputs.
pub const MAX_POLICY_TENSORS: usize = 16;
/// Maximum bounded request/result queue capacity.
pub const MAX_POLICY_QUEUE_CAPACITY: usize = 64;
/// Maximum dedicated RTen threads.
pub const MAX_POLICY_THREADS: usize = 32;
