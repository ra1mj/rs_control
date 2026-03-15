//! Typed policy readiness, inference and fallback failures.

use thiserror::Error;

/// The only supported v1 fallback never refreshes command authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolicyFallback {
    /// Remove the policy proposal and let downstream deadlines expire.
    WithdrawProposal,
}

/// Fail-closed policy runtime error.
#[derive(Debug, Error)]
pub enum PolicyError {
    /// Artifact directory or file contract failed.
    #[error("invalid policy artifact: {0}")]
    Artifact(String),
    /// Strict manifest validation failed.
    #[error("invalid policy manifest: {0}")]
    Manifest(String),
    /// ONNX or RTen graph contract failed.
    #[error("invalid policy graph: {0}")]
    Graph(String),
    /// Compiled morphology identity/order differs.
    #[error("policy binding mismatch: {0}")]
    Binding(String),
    /// Observation is unavailable and must not be replaced with zeros.
    #[error("policy observation unavailable: {0}")]
    Observation(String),
    /// Model execution or output validation failed.
    #[error("policy inference failed: {0}")]
    Inference(String),
    /// Nonblocking request or result queue is full.
    #[error("policy queue is full")]
    QueueFull,
    /// No result is currently available.
    #[error("no policy result is available")]
    ResultUnavailable,
    /// Worker is not alive.
    #[error("policy worker is unavailable")]
    WorkerUnavailable,
    /// Result missed its exclusive deadline.
    #[error("policy inference deadline expired")]
    DeadlineExpired,
    /// Result belongs to another request or policy generation.
    #[error("policy result identity is stale")]
    StaleResult,
    /// Semantic proposal construction failed.
    #[error(transparent)]
    Semantic(#[from] message_model::SemanticError),
}

impl PolicyError {
    /// Every runtime failure has the same non-authorizing v1 fallback.
    #[must_use]
    pub const fn fallback(&self) -> PolicyFallback {
        PolicyFallback::WithdrawProposal
    }
}
