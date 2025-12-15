//! Validated project-owned arm models and compiled multi-morphology profiles.
//!
//! The bounded XML parse tree is private to this crate. Public values are
//! finite, SI-valued, bounded at construction, and bound to an immutable
//! profile identity.

mod error;
mod model;
mod profile;
mod transmission;
mod urdf;

pub use error::*;
pub use model::*;
pub use profile::*;
pub use transmission::*;
pub use urdf::UrdfParseError;
