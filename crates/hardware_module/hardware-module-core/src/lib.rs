//! Allocation-conscious contracts owned by the robot `hardware_module`.
//!
//! These traits are synchronous and caller-buffered. Implementations used by a
//! realtime thread must additionally honour the non-blocking guarantees stated
//! on each method; this crate itself performs no I/O.

use arrayvec::ArrayVec;
use std::error::Error;
use thiserror::Error;

/// A timestamp in one local monotonic clock domain, expressed in nanoseconds.
#[derive(Clone, Copy, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct MonoTime(u64);

impl MonoTime {
    /// Creates a timestamp from raw monotonic nanoseconds.
    #[must_use]
    pub const fn from_nanos(nanos: u64) -> Self {
        Self(nanos)
    }

    /// Returns the timestamp as nanoseconds.
    #[must_use]
    pub const fn as_nanos(self) -> u64 {
        self.0
    }

    /// Returns elapsed nanoseconds, or `None` when `earlier` belongs to the
    /// future or a different/reset clock domain.
    #[must_use]
    pub const fn checked_elapsed_since(self, earlier: Self) -> Option<u64> {
        self.0.checked_sub(earlier.0)
    }
}

/// Hardware-module device lifecycle.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum LifecycleState {
    /// Driver has not been configured.
    Unconfigured,
    /// Driver is configured and produces no output.
    Disabled,
    /// Driver is executing a bounded enable handshake.
    Enabling,
    /// Driver may accept valid commands.
    Enabled,
    /// Driver is executing a bounded disable handshake.
    Disabling,
    /// Driver is enforcing a safe stop while retaining diagnostics.
    SafeStop,
    /// Driver requires explicit fault handling before it may be enabled again.
    Faulted,
}

/// Validates and stores hardware-module lifecycle transitions.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Lifecycle {
    state: LifecycleState,
}

impl Default for Lifecycle {
    fn default() -> Self {
        Self::new()
    }
}

impl Lifecycle {
    /// Creates an unconfigured lifecycle.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: LifecycleState::Unconfigured,
        }
    }

    /// Returns the current lifecycle state.
    #[must_use]
    pub const fn state(self) -> LifecycleState {
        self.state
    }

    /// Applies one explicitly permitted transition.
    pub fn transition(&mut self, next: LifecycleState) -> Result<(), TransitionError> {
        let allowed = matches!(
            (self.state, next),
            (LifecycleState::Unconfigured, LifecycleState::Disabled)
                | (LifecycleState::Unconfigured, LifecycleState::Faulted)
                | (LifecycleState::Disabled, LifecycleState::Enabling)
                | (LifecycleState::Disabled, LifecycleState::Faulted)
                | (LifecycleState::Enabling, LifecycleState::Enabled)
                | (LifecycleState::Enabling, LifecycleState::Disabled)
                | (LifecycleState::Enabling, LifecycleState::SafeStop)
                | (LifecycleState::Enabling, LifecycleState::Faulted)
                | (LifecycleState::Enabled, LifecycleState::Disabling)
                | (LifecycleState::Enabled, LifecycleState::SafeStop)
                | (LifecycleState::Enabled, LifecycleState::Faulted)
                | (LifecycleState::Disabling, LifecycleState::Disabled)
                | (LifecycleState::Disabling, LifecycleState::SafeStop)
                | (LifecycleState::Disabling, LifecycleState::Faulted)
                | (LifecycleState::SafeStop, LifecycleState::Disabling)
                | (LifecycleState::SafeStop, LifecycleState::Disabled)
                | (LifecycleState::SafeStop, LifecycleState::Faulted)
                | (LifecycleState::Faulted, LifecycleState::Disabled)
        );

        if !allowed {
            return Err(TransitionError {
                from: self.state,
                to: next,
            });
        }
        self.state = next;
        Ok(())
    }
}

/// An attempted lifecycle transition that violates the hardware-module state model.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
#[error("invalid device lifecycle transition from {from:?} to {to:?}")]
pub struct TransitionError {
    /// State before the rejected transition.
    pub from: LifecycleState,
    /// Requested state.
    pub to: LifecycleState,
}

/// Quality attached to a hardware-module device sample.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum SampleQuality {
    /// Sample is valid for its documented use.
    Valid,
    /// Sample remains usable only by an explicitly declared degraded policy.
    Degraded,
    /// Sample has exceeded its freshness limit.
    Stale,
    /// Device is calibrating and the sample must not be treated as nominal.
    Calibrating,
    /// Device has reported or inferred a fault.
    Faulted,
}

/// Fixed-capacity frame storage which never allocates after construction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrameBatch<F, const N: usize> {
    frames: ArrayVec<F, N>,
}

impl<F, const N: usize> Default for FrameBatch<F, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<F, const N: usize> FrameBatch<F, N> {
    /// Creates an empty batch backed by inline storage.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            frames: ArrayVec::new_const(),
        }
    }

    /// Appends a frame, returning ownership when the fixed capacity is full.
    pub fn push(&mut self, frame: F) -> Result<(), F> {
        if self.frames.len() == N {
            return Err(frame);
        }
        self.frames.push(frame);
        Ok(())
    }

    /// Removes all frames without changing capacity.
    pub fn clear(&mut self) {
        self.frames.clear();
    }

    /// Returns the initialized frames.
    #[must_use]
    pub fn as_slice(&self) -> &[F] {
        self.frames.as_slice()
    }

    /// Returns the number of initialized frames.
    #[must_use]
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// Returns whether the batch contains no frames.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Returns the compile-time frame capacity.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        N
    }
}

/// Transport link state independent of a concrete bus implementation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum LinkState {
    /// Link has not been configured.
    Down,
    /// Link can transmit and receive.
    Up,
    /// Link is still operating but reports reduced capability.
    Degraded,
    /// Link cannot carry traffic until an explicit recovery succeeds.
    Faulted,
}

/// Snapshot of transport health; counters are monotonic within one process boot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransportHealth {
    /// Current link state.
    pub link_state: LinkState,
    /// Received frames dropped before delivery to the driver.
    pub rx_dropped: u64,
    /// Transmit failures reported by the transport.
    pub tx_errors: u64,
    /// Bus- or link-specific error events.
    pub bus_errors: u64,
}

/// Synchronous, caller-buffered frame transport.
pub trait FrameTransport {
    /// One transport frame.
    type Frame: Copy;
    /// Typed transport error.
    type Error: Error + Send + Sync + 'static;

    /// Receives at most `output.len()` frames and returns their count.
    ///
    /// Realtime implementations must be non-blocking and must not allocate.
    fn receive(&mut self, output: &mut [Self::Frame]) -> Result<usize, Self::Error>;

    /// Attempts to transmit the supplied frames and returns the accepted count.
    ///
    /// Realtime implementations must be non-blocking and must not allocate.
    fn transmit(&mut self, frames: &[Self::Frame]) -> Result<usize, Self::Error>;

    /// Returns a non-blocking health snapshot.
    fn health(&self) -> TransportHealth;
}

/// Error shared by hardware-module device-driver lifecycle and command handling.
#[derive(Debug, Error)]
pub enum DeviceError<Fault>
where
    Fault: Error + 'static,
{
    /// Operation is invalid in the current lifecycle state.
    #[error("operation requires {required:?}, current state is {actual:?}")]
    InvalidLifecycle {
        /// Required state.
        required: LifecycleState,
        /// Actual state.
        actual: LifecycleState,
    },
    /// A command could not fit in the caller-provided frame batch.
    #[error("encoded command exceeds fixed frame capacity {capacity}")]
    OutputBatchFull {
        /// Fixed batch capacity.
        capacity: usize,
    },
    /// Feedback exceeded its declared freshness limit.
    #[error("feedback is stale: age {age_ns} ns exceeds limit {limit_ns} ns")]
    StaleFeedback {
        /// Observed sample age.
        age_ns: u64,
        /// Configured freshness limit.
        limit_ns: u64,
    },
    /// Command failed semantic validation.
    #[error("invalid command: {0}")]
    InvalidCommand(&'static str),
    /// Device-protocol-specific fault.
    #[error("device protocol fault: {0}")]
    Protocol(#[source] Fault),
}

/// Synchronous device protocol/lifecycle contract.
pub trait DeviceDriver<F, const MAX_OUTPUT_FRAMES: usize>
where
    F: Copy,
{
    /// Standardized command accepted by this device.
    type Command;
    /// Standardized sample produced by this device.
    type Sample;
    /// Protocol/device-specific typed fault.
    type Fault: Error + 'static;

    /// Configures the driver without enabling physical output.
    fn configure(&mut self) -> Result<(), DeviceError<Self::Fault>>;

    /// Starts the bounded enable handshake.
    fn enable(&mut self) -> Result<(), DeviceError<Self::Fault>>;

    /// Starts the bounded disable/safe-stop handshake.
    fn disable(&mut self) -> Result<(), DeviceError<Self::Fault>>;

    /// Consumes already received input and optionally emits one standardized sample.
    ///
    /// Realtime implementations must not block or allocate.
    fn handle_input(
        &mut self,
        now: MonoTime,
    ) -> Result<Option<Self::Sample>, DeviceError<Self::Fault>>;

    /// Encodes one validated command into caller-owned fixed-capacity storage.
    fn encode_command(
        &mut self,
        command: &Self::Command,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;
}

#[cfg(test)]
mod tests {
    use super::{DeviceError, FrameBatch, Lifecycle, LifecycleState, MonoTime};
    use std::error::Error as _;
    use thiserror::Error;

    #[derive(Debug, Error)]
    #[error("fixture protocol fault")]
    struct FixtureFault;

    #[test]
    fn monotonic_elapsed_rejects_time_reversal() {
        let earlier = MonoTime::from_nanos(10);
        assert_eq!(
            MonoTime::from_nanos(25).checked_elapsed_since(earlier),
            Some(15)
        );
        assert_eq!(
            earlier.checked_elapsed_since(MonoTime::from_nanos(25)),
            None
        );
    }

    #[test]
    fn frame_batch_returns_frame_when_full() {
        let mut batch = FrameBatch::<u8, 2>::new();
        assert_eq!(batch.push(1), Ok(()));
        assert_eq!(batch.push(2), Ok(()));
        assert_eq!(batch.push(3), Err(3));
        assert_eq!(batch.as_slice(), &[1, 2]);
    }

    #[test]
    fn lifecycle_requires_explicit_safe_transitions() {
        let mut lifecycle = Lifecycle::new();
        assert!(lifecycle.transition(LifecycleState::Enabled).is_err());
        lifecycle.transition(LifecycleState::Disabled).unwrap();
        lifecycle.transition(LifecycleState::Enabling).unwrap();
        lifecycle.transition(LifecycleState::Enabled).unwrap();
        lifecycle.transition(LifecycleState::SafeStop).unwrap();
        lifecycle.transition(LifecycleState::Disabled).unwrap();
        assert_eq!(lifecycle.state(), LifecycleState::Disabled);
    }

    #[test]
    fn typed_protocol_fault_preserves_error_source() {
        let error = DeviceError::Protocol(FixtureFault);
        assert_eq!(
            error.to_string(),
            "device protocol fault: fixture protocol fault"
        );
        assert!(error.source().is_some());
    }
}
