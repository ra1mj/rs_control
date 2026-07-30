//! Allocation-conscious contracts owned by the robot `hardware_module`.
//!
//! These traits are synchronous and caller-buffered. Implementations used by a
//! realtime thread must additionally honour the non-blocking guarantees stated
//! on each method; this crate itself performs no I/O.

use arrayvec::ArrayVec;
use std::error::Error;
use thiserror::Error;

/// Largest valid standard (11-bit) CAN identifier.
pub const MAX_STANDARD_CAN_ID: u16 = 0x07ff;

/// Maximum payload length of a classic CAN data frame.
pub const MAX_CLASSIC_CAN_DATA_LEN: usize = 8;

/// Maximum diagnostic timestamps retained with one received CAN frame.
pub const MAX_CAN_DIAGNOSTIC_TIMESTAMPS: usize = 3;

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

/// Validated standard (11-bit) CAN identifier.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct StandardCanId(u16);

impl StandardCanId {
    /// Validates a standard CAN identifier.
    pub const fn new(raw: u16) -> Result<Self, StandardCanIdError> {
        if raw <= MAX_STANDARD_CAN_ID {
            Ok(Self(raw))
        } else {
            Err(StandardCanIdError { raw })
        }
    }

    /// Returns the raw 11-bit identifier.
    #[must_use]
    pub const fn as_raw(self) -> u16 {
        self.0
    }
}

impl TryFrom<u16> for StandardCanId {
    type Error = StandardCanIdError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

/// A value outside the standard 11-bit CAN identifier range.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
#[error("standard CAN identifier {raw:#x} exceeds 0x7ff")]
pub struct StandardCanIdError {
    /// Rejected raw identifier.
    pub raw: u16,
}

/// Fixed-storage standard classic-CAN data frame.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct ClassicCanFrame {
    id: StandardCanId,
    data: [u8; MAX_CLASSIC_CAN_DATA_LEN],
    dlc: u8,
}

impl ClassicCanFrame {
    /// Copies one payload into fixed storage and validates classic-CAN DLC.
    pub fn new(id: StandardCanId, data: &[u8]) -> Result<Self, ClassicCanFrameError> {
        if data.len() > MAX_CLASSIC_CAN_DATA_LEN {
            return Err(ClassicCanFrameError::PayloadTooLong { len: data.len() });
        }

        let mut storage = [0_u8; MAX_CLASSIC_CAN_DATA_LEN];
        storage[..data.len()].copy_from_slice(data);
        Ok(Self {
            id,
            data: storage,
            dlc: data.len() as u8,
        })
    }

    /// Returns the standard arbitration identifier.
    #[must_use]
    pub const fn id(self) -> StandardCanId {
        self.id
    }

    /// Returns the validated payload length.
    #[must_use]
    pub const fn dlc(self) -> u8 {
        self.dlc
    }

    /// Returns only initialized payload bytes.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        &self.data[..usize::from(self.dlc)]
    }

    /// Returns the complete zero-padded storage.
    #[must_use]
    pub const fn padded_data(self) -> [u8; MAX_CLASSIC_CAN_DATA_LEN] {
        self.data
    }
}

/// Invalid classic-CAN frame construction.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum ClassicCanFrameError {
    /// Payload exceeds eight classic-CAN bytes.
    #[error("classic CAN payload length {len} exceeds 8 bytes")]
    PayloadTooLong {
        /// Rejected payload length.
        len: usize,
    },
}

/// Origin and clock domain of one diagnostic receive timestamp.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum DiagnosticTimestampOrigin {
    /// Socket-level `SO_TIMESTAMPNS` realtime timestamp.
    SocketRealtime,
    /// Kernel software receive timestamp in the realtime domain.
    KernelSoftwareRealtime,
    /// Hardware/PHC timestamp whose clock correlation is not established here.
    HardwareClock,
}

/// One diagnostic timestamp which must never be compared directly to `MonoTime`.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct DiagnosticTimestamp {
    /// Timestamp source and clock domain.
    pub origin: DiagnosticTimestampOrigin,
    /// Nanoseconds in the source clock domain.
    pub nanos: u64,
}

/// Fixed-storage diagnostic timestamps attached to one receive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanDiagnosticTimestamps {
    entries: [Option<DiagnosticTimestamp>; MAX_CAN_DIAGNOSTIC_TIMESTAMPS],
}

impl Default for CanDiagnosticTimestamps {
    fn default() -> Self {
        Self::new()
    }
}

impl CanDiagnosticTimestamps {
    /// Creates empty diagnostic timestamp storage.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: [None; MAX_CAN_DIAGNOSTIC_TIMESTAMPS],
        }
    }

    /// Appends one timestamp without allocating, returning it when full.
    pub fn push(&mut self, timestamp: DiagnosticTimestamp) -> Result<(), DiagnosticTimestamp> {
        for entry in &mut self.entries {
            if entry.is_none() {
                *entry = Some(timestamp);
                return Ok(());
            }
        }
        Err(timestamp)
    }

    /// Returns all fixed slots; unused slots are `None`.
    #[must_use]
    pub const fn entries(&self) -> &[Option<DiagnosticTimestamp>; MAX_CAN_DIAGNOSTIC_TIMESTAMPS] {
        &self.entries
    }
}

/// Standard data frame plus freshness and diagnostic receive metadata.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ReceivedCanFrame {
    /// Validated classic-CAN data frame.
    pub frame: ClassicCanFrame,
    /// Local monotonic observation time used for freshness.
    pub observed_monotonic: MonoTime,
    /// Optional diagnostic timestamps in explicitly tagged clock domains.
    pub diagnostic_timestamps: CanDiagnosticTimestamps,
}

/// Raw Linux-compatible CAN error event without a Linux crate dependency.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanErrorEvent {
    /// Raw CAN error-class bitset.
    pub error_bits: u32,
    /// Raw eight error payload bytes.
    pub raw_data: [u8; MAX_CLASSIC_CAN_DATA_LEN],
    /// Local monotonic observation time.
    pub observed_monotonic: MonoTime,
}

/// Receive event produced by a classic-CAN transport.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CanBusEvent {
    /// Valid standard classic-CAN data frame.
    Data(ReceivedCanFrame),
    /// Raw CAN error event.
    Error(CanErrorEvent),
}

/// Hardware-module device lifecycle.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum LifecycleState {
    /// Driver has not been configured.
    Unconfigured,
    /// Driver is executing a bounded read/verify configuration handshake.
    Configuring,
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
            (LifecycleState::Unconfigured, LifecycleState::Configuring)
                | (LifecycleState::Unconfigured, LifecycleState::Faulted)
                | (LifecycleState::Configuring, LifecycleState::Disabled)
                | (LifecycleState::Configuring, LifecycleState::Faulted)
                | (LifecycleState::Disabled, LifecycleState::Configuring)
                | (LifecycleState::Disabled, LifecycleState::Enabling)
                | (LifecycleState::Disabled, LifecycleState::Faulted)
                | (LifecycleState::Enabling, LifecycleState::Enabled)
                | (LifecycleState::Enabling, LifecycleState::SafeStop)
                | (LifecycleState::Enabling, LifecycleState::Faulted)
                | (LifecycleState::Enabled, LifecycleState::Disabling)
                | (LifecycleState::Enabled, LifecycleState::SafeStop)
                | (LifecycleState::Enabled, LifecycleState::Faulted)
                | (LifecycleState::Disabling, LifecycleState::Disabled)
                | (LifecycleState::Disabling, LifecycleState::SafeStop)
                | (LifecycleState::Disabling, LifecycleState::Faulted)
                | (LifecycleState::SafeStop, LifecycleState::Disabling)
                | (LifecycleState::SafeStop, LifecycleState::Faulted)
                | (LifecycleState::Faulted, LifecycleState::Configuring)
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

/// Whether a transport metric is measured by the backend.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum MetricValue<T> {
    /// Backend measured the supplied value.
    Measured(T),
    /// Backend cannot observe this metric.
    Unavailable,
}

impl MetricValue<u64> {
    /// Adds to a measured counter with saturation; unavailable remains unavailable.
    #[must_use]
    pub const fn saturating_add(self, value: u64) -> Self {
        match self {
            Self::Measured(current) => Self::Measured(current.saturating_add(value)),
            Self::Unavailable => Self::Unavailable,
        }
    }
}

/// Snapshot of transport health; counters are monotonic within one process boot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TransportHealth {
    /// Current link state.
    pub link_state: LinkState,
    /// Received frames dropped before delivery to the driver.
    pub rx_dropped: MetricValue<u64>,
    /// Transmit failures reported by the transport.
    pub tx_errors: MetricValue<u64>,
    /// Bus- or link-specific error events.
    pub bus_errors: MetricValue<u64>,
}

/// Synchronous, caller-buffered frame transport.
pub trait FrameTransport {
    /// One received transport event.
    type RxFrame: Copy;
    /// One frame accepted for transmission.
    type TxFrame: Copy;
    /// Typed transport error.
    type Error: Error + Send + Sync + 'static;

    /// Receives at most `output.len()` frames and returns their count.
    ///
    /// Realtime implementations must be non-blocking and must not allocate.
    fn receive(&mut self, output: &mut [Self::RxFrame]) -> Result<usize, Self::Error>;

    /// Attempts to transmit the supplied frames and returns the accepted count.
    ///
    /// Realtime implementations must be non-blocking and must not allocate.
    fn transmit(&mut self, frames: &[Self::TxFrame]) -> Result<usize, Self::Error>;

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

    /// Returns the current device lifecycle state.
    fn lifecycle(&self) -> LifecycleState;

    /// Starts bounded read-only configuration without enabling physical output.
    fn start_configure(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;

    /// Starts the bounded enable handshake.
    fn start_enable(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;

    /// Starts the bounded disable/safe-stop handshake.
    fn start_disable(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;

    /// Consumes already received input and optionally emits one standardized sample.
    ///
    /// Realtime implementations must not block or allocate.
    fn handle_input(
        &mut self,
        frame: F,
        received_at: MonoTime,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<Option<Self::Sample>, DeviceError<Self::Fault>>;

    /// Polls bounded transition/freshness deadlines and may emit lifecycle frames.
    fn poll(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;

    /// Encodes one validated command into caller-owned fixed-capacity storage.
    fn encode_command(
        &mut self,
        command: &Self::Command,
        now: MonoTime,
        output: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>,
    ) -> Result<(), DeviceError<Self::Fault>>;

    /// Reports a transport failure so the device faults closed.
    fn report_transport_fault(&mut self, now: MonoTime) -> Result<(), DeviceError<Self::Fault>>;
}

#[cfg(test)]
mod tests {
    use super::{
        CanDiagnosticTimestamps, ClassicCanFrame, ClassicCanFrameError, DeviceError,
        DiagnosticTimestamp, DiagnosticTimestampOrigin, FrameBatch, Lifecycle, LifecycleState,
        MetricValue, MonoTime, StandardCanId,
    };
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
    fn standard_can_id_and_classic_frame_validate_boundaries() {
        assert_eq!(StandardCanId::new(0).unwrap().as_raw(), 0);
        let maximum = StandardCanId::new(0x7ff).unwrap();
        assert_eq!(maximum.as_raw(), 0x7ff);
        assert!(StandardCanId::new(0x800).is_err());

        let frame = ClassicCanFrame::new(maximum, &[1, 2, 3]).unwrap();
        assert_eq!(frame.id(), maximum);
        assert_eq!(frame.dlc(), 3);
        assert_eq!(frame.data(), &[1, 2, 3]);
        assert_eq!(frame.padded_data(), [1, 2, 3, 0, 0, 0, 0, 0]);
        assert_eq!(
            ClassicCanFrame::new(maximum, &[0; 9]),
            Err(ClassicCanFrameError::PayloadTooLong { len: 9 })
        );
    }

    #[test]
    fn diagnostic_timestamps_are_fixed_capacity() {
        let mut timestamps = CanDiagnosticTimestamps::new();
        for (origin, nanos) in [
            (DiagnosticTimestampOrigin::SocketRealtime, 1),
            (DiagnosticTimestampOrigin::KernelSoftwareRealtime, 2),
            (DiagnosticTimestampOrigin::HardwareClock, 3),
        ] {
            timestamps
                .push(DiagnosticTimestamp { origin, nanos })
                .unwrap();
        }
        let overflow = DiagnosticTimestamp {
            origin: DiagnosticTimestampOrigin::HardwareClock,
            nanos: 4,
        };
        assert_eq!(timestamps.push(overflow), Err(overflow));
        assert_eq!(timestamps.entries()[1].unwrap().nanos, 2);
    }

    #[test]
    fn lifecycle_requires_explicit_safe_transitions() {
        let mut lifecycle = Lifecycle::new();
        assert!(lifecycle.transition(LifecycleState::Enabled).is_err());
        lifecycle.transition(LifecycleState::Configuring).unwrap();
        lifecycle.transition(LifecycleState::Disabled).unwrap();
        lifecycle.transition(LifecycleState::Enabling).unwrap();
        lifecycle.transition(LifecycleState::Enabled).unwrap();
        lifecycle.transition(LifecycleState::SafeStop).unwrap();
        lifecycle.transition(LifecycleState::Disabling).unwrap();
        lifecycle.transition(LifecycleState::Disabled).unwrap();
        lifecycle.transition(LifecycleState::Faulted).unwrap();
        assert!(lifecycle.transition(LifecycleState::Disabled).is_err());
        lifecycle.transition(LifecycleState::Configuring).unwrap();
        lifecycle.transition(LifecycleState::Disabled).unwrap();
        assert_eq!(lifecycle.state(), LifecycleState::Disabled);
    }

    #[test]
    fn metric_value_distinguishes_unavailable_and_saturates() {
        assert_eq!(
            MetricValue::Measured(u64::MAX).saturating_add(1),
            MetricValue::Measured(u64::MAX)
        );
        assert_eq!(
            MetricValue::Unavailable.saturating_add(1),
            MetricValue::Unavailable
        );
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
