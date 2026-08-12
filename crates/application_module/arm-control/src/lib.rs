//! Morphology-aware arm proposal admission and host-only execution surfaces.
//!
//! This application-layer crate never constructs a hardware frame. It
//! validates semantic proposals against immutable model identity, coherent
//! observations, motion limits, local authority and action lifecycle.

mod admission;
mod config;
mod error;
mod execution;
mod feed_forward;

pub use admission::*;
pub use config::*;
pub use error::*;
pub use execution::*;
pub use feed_forward::*;
