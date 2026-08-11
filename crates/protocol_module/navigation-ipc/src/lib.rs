//! Fixed-capacity local transport for the crash-isolated navigation plugin.
//!
//! The public values remain backend-neutral `message-model` contracts. These
//! codecs project them into explicit, versioned little-endian shared-memory
//! representations rather than sharing Rust object layouts.

mod action;
mod channel;
mod codec;
mod diagnostics;
mod guard;
mod participation;
mod wire;

pub use action::*;
pub use channel::*;
pub use codec::*;
pub use diagnostics::*;
pub use guard::*;
pub use participation::*;

#[cfg(test)]
mod tests;

use thiserror::Error;

/// Navigation IPC validation or semantic conversion failure.
#[derive(Debug, Error)]
pub enum NavigationIpcError {
    /// Fixed representation contained an invalid discriminator, length or padding.
    #[error("invalid navigation IPC representation: {0}")]
    InvalidRepresentation(&'static str),
    /// A shared-memory header is expired in the receiving monotonic domain.
    #[error("navigation IPC value expired at {deadline_ns}, observed at {now_ns}")]
    Expired {
        /// Exclusive validity deadline.
        deadline_ns: u64,
        /// Local receive time.
        now_ns: u64,
    },
    /// An action payload exceeded its fixed local transport bound.
    #[error("navigation action packet length {actual} exceeds maximum {maximum}")]
    ActionPacketTooLarge {
        /// Serialized packet length.
        actual: usize,
        /// Fixed packet capacity.
        maximum: usize,
    },
    /// A navigation action carried the wrong registered action or payload type.
    #[error("unexpected navigation action identity in {0}")]
    ActionIdentity(&'static str),
    /// A fixed navigation participation snapshot was malformed.
    #[error("invalid navigation participation snapshot: {0}")]
    Participation(&'static str),
    /// A fixed navigation transport diagnostic snapshot was malformed.
    #[error("invalid navigation transport diagnostics snapshot: {0}")]
    TransportDiagnostics(&'static str),
    /// A transform batch does not match the declared dynamic TF ownership.
    #[error("invalid navigation TF batch: {0}")]
    TransformOwnership(&'static str),
    /// Backend-neutral semantic validation failed.
    #[error(transparent)]
    Semantic(#[from] message_model::SemanticError),
    /// A fixed identifier was malformed.
    #[error(transparent)]
    Identifier(#[from] message_model::IdError),
    /// A fixed 128-bit identity was malformed.
    #[error(transparent)]
    Identity(#[from] message_model::Id128Error),
    /// Protobuf action mapping failed.
    #[error(transparent)]
    Codec(#[from] contract_codec::CodecError),
    /// Shared-memory layout or lifecycle operation failed.
    #[error(transparent)]
    SharedMemory(#[from] middleware_shm::ShmError),
    /// A channel name or capacity violated the fixed SHM startup contract.
    #[error(transparent)]
    SharedMemoryContract(#[from] middleware_shm::ContractError),
}
