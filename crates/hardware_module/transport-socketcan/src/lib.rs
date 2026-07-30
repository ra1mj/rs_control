//! Bounded Linux classic-SocketCAN transport for robot hardware modules.
//!
//! The production constructor only opens an explicitly named interface. It
//! never creates, configures, restarts, or changes the bitrate of that
//! interface. Filter changes and startup draining are initialization-only;
//! realtime receive and transmit paths are synchronous, bounded, and do not
//! allocate or retry.

use hardware_module_core::{
    CanBusEvent, CanDiagnosticTimestamps, CanErrorEvent, ClassicCanFrame, DiagnosticTimestamp,
    DiagnosticTimestampOrigin, FrameTransport, LinkState, MetricValue, MonoTime, ReceivedCanFrame,
    StandardCanId, TransportHealth,
};
use socketcan::{
    CanFilter, CanFrame, CanSocket, CanTimestamps, EmbeddedFrame, Id as SocketCanId, Socket,
    SocketOptions,
};
use std::{io, time::Instant};
use thiserror::Error;

/// Default maximum planned CAN utilization percentage.
pub const DEFAULT_BUS_BUDGET_PERCENT: u8 = 70;

/// Linux CAN error bit: transmit timeout.
pub const CAN_ERROR_TX_TIMEOUT: u32 = 0x0000_0001;
/// Linux CAN error bit: controller problem.
pub const CAN_ERROR_CONTROLLER: u32 = 0x0000_0004;
/// Linux CAN error bit: transceiver problem.
pub const CAN_ERROR_TRANSCEIVER: u32 = 0x0000_0010;
/// Linux CAN error bit: missing acknowledgement.
pub const CAN_ERROR_ACK: u32 = 0x0000_0020;
/// Linux CAN error bit: bus-off state.
pub const CAN_ERROR_BUS_OFF: u32 = 0x0000_0040;
/// Linux CAN error bit: controller restarted.
pub const CAN_ERROR_RESTARTED: u32 = 0x0000_0100;

/// Safety-relevant CAN error classes subscribed by the control socket.
pub const CONTROL_ERROR_MASK: u32 = CAN_ERROR_TX_TIMEOUT
    | CAN_ERROR_CONTROLLER
    | CAN_ERROR_TRANSCEIVER
    | CAN_ERROR_ACK
    | CAN_ERROR_BUS_OFF
    | CAN_ERROR_RESTARTED;

/// Exact-match mask for a standard, non-RTR data frame.
pub const EXACT_STANDARD_DATA_MASK: u32 = socketcan::frame::CAN_SFF_MASK
    | socketcan::frame::CAN_EFF_FLAG
    | socketcan::frame::CAN_RTR_FLAG;

const STANDARD_DATA_DISCOVERY_MASK: u32 =
    socketcan::frame::CAN_EFF_FLAG | socketcan::frame::CAN_RTR_FLAG;
const LINUX_ENOBUFS: i32 = 105;

/// Raw, dependency-neutral filter description used during disarmed setup.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanFilterSpec {
    /// CAN identifier bits compared by the filter.
    pub id: u32,
    /// CAN identifier/flag bits participating in the comparison.
    pub mask: u32,
}

/// Broad discovery filter accepting only standard non-RTR data frames.
#[must_use]
pub const fn discovery_filter_spec() -> CanFilterSpec {
    CanFilterSpec {
        id: 0,
        mask: STANDARD_DATA_DISCOVERY_MASK,
    }
}

/// Exact standard-data filter for one validated feedback identifier.
#[must_use]
pub const fn exact_feedback_filter_spec(id: StandardCanId) -> CanFilterSpec {
    CanFilterSpec {
        id: id.as_raw() as u32,
        mask: EXACT_STANDARD_DATA_MASK,
    }
}

/// Initialization/realtime filter phase.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FilterPhase {
    /// Broad standard-data discovery is active while all motors are disarmed.
    Discovery,
    /// Exact validated feedback filters are installed for normal ownership.
    Feedback,
}

/// Detailed monotonic health snapshot for the SocketCAN adapter.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SocketCanHealth {
    /// Current link state.
    pub link_state: LinkState,
    /// Data frames delivered to callers.
    pub rx_frames: u64,
    /// CAN error frames delivered to callers.
    pub rx_error_events: u64,
    /// Unsupported extended or RTR frames discarded.
    pub rx_unsupported_frames: u64,
    /// Structurally malformed frames discarded.
    pub rx_malformed_frames: u64,
    /// Receive system-call failures.
    pub rx_io_errors: u64,
    /// Calls that consumed their configured receive ceiling.
    pub rx_drain_limit_hits: u64,
    /// Kernel receive drops, unavailable through the selected safe API.
    pub kernel_rx_dropped: MetricValue<u64>,
    /// Frames accepted by the kernel for transmission.
    pub tx_frames: u64,
    /// Would-block or ENOBUFS transmit failures.
    pub tx_backpressure: u64,
    /// Other transmit failures.
    pub tx_errors: u64,
    /// Transmit-timeout error events.
    pub can_tx_timeout_events: u64,
    /// Controller error events.
    pub can_controller_events: u64,
    /// Transceiver error events.
    pub can_transceiver_events: u64,
    /// Missing-acknowledgement events.
    pub can_ack_events: u64,
    /// Bus-off events, including combined bitsets.
    pub can_bus_off_events: u64,
    /// Controller restart events; these never authorize automatic resume.
    pub can_restarted_events: u64,
}

impl Default for SocketCanHealth {
    fn default() -> Self {
        Self {
            link_state: LinkState::Down,
            rx_frames: 0,
            rx_error_events: 0,
            rx_unsupported_frames: 0,
            rx_malformed_frames: 0,
            rx_io_errors: 0,
            rx_drain_limit_hits: 0,
            kernel_rx_dropped: MetricValue::Unavailable,
            tx_frames: 0,
            tx_backpressure: 0,
            tx_errors: 0,
            can_tx_timeout_events: 0,
            can_controller_events: 0,
            can_transceiver_events: 0,
            can_ack_events: 0,
            can_bus_off_events: 0,
            can_restarted_events: 0,
        }
    }
}

impl SocketCanHealth {
    fn generic(self) -> TransportHealth {
        TransportHealth {
            link_state: self.link_state,
            rx_dropped: self.kernel_rx_dropped,
            tx_errors: MetricValue::Measured(self.tx_errors.saturating_add(self.tx_backpressure)),
            bus_errors: MetricValue::Measured(self.rx_error_events),
        }
    }

    fn record_error_bits(&mut self, bits: u32) {
        self.rx_error_events = self.rx_error_events.saturating_add(1);
        if bits & CAN_ERROR_TX_TIMEOUT != 0 {
            self.can_tx_timeout_events = self.can_tx_timeout_events.saturating_add(1);
        }
        if bits & CAN_ERROR_CONTROLLER != 0 {
            self.can_controller_events = self.can_controller_events.saturating_add(1);
        }
        if bits & CAN_ERROR_TRANSCEIVER != 0 {
            self.can_transceiver_events = self.can_transceiver_events.saturating_add(1);
        }
        if bits & CAN_ERROR_ACK != 0 {
            self.can_ack_events = self.can_ack_events.saturating_add(1);
        }
        if bits & CAN_ERROR_BUS_OFF != 0 {
            self.can_bus_off_events = self.can_bus_off_events.saturating_add(1);
            self.link_state = LinkState::Faulted;
        }
        if bits & CAN_ERROR_RESTARTED != 0 {
            self.can_restarted_events = self.can_restarted_events.saturating_add(1);
        }
    }
}

/// Transport construction, receive, or transmit failure.
#[derive(Debug, Error)]
pub enum SocketCanError {
    /// Invalid disarmed setup request.
    #[error("invalid SocketCAN setup: {0}")]
    InvalidSetup(&'static str),
    /// Interface/socket operation failed.
    #[error("SocketCAN {operation} failed: {source}")]
    Io {
        /// Operation that failed.
        operation: &'static str,
        /// Original system error.
        #[source]
        source: io::Error,
    },
    /// A prefix was accepted before a one-shot transmit failure.
    #[error("SocketCAN transmit failed after accepting {accepted} frame(s): {source}")]
    PartialTransmit {
        /// Number of frames accepted before the failure.
        accepted: usize,
        /// Whether the failure was WouldBlock or ENOBUFS backpressure.
        backpressure: bool,
        /// Original system error.
        #[source]
        source: io::Error,
    },
    /// Normal transmission is inhibited after BUS-OFF.
    #[error("SocketCAN link is bus-off; normal transmission is inhibited")]
    BusOff,
}

/// One scheduled standard classic-CAN flow for bus-load planning.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanBudgetFlow {
    /// Classic-CAN data length code, `0..=8`.
    pub dlc: u8,
    /// Planned frames per second for this flow.
    pub frames_per_second: u64,
}

/// Checked bus-budget result.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CanBusBudget {
    /// Sum of worst-case planned bits per second.
    pub planned_bits_per_second: u64,
    /// Configured physical CAN bitrate.
    pub bitrate: u64,
    /// Maximum planned utilization percentage.
    pub threshold_percent: u8,
    /// Whether the exact rational threshold comparison passed.
    pub fits: bool,
}

/// Invalid or overflowing bus-budget input.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum CanBudgetError {
    /// Classic CAN supports only DLC `0..=8`.
    #[error("classic CAN DLC {dlc} exceeds 8")]
    InvalidDlc {
        /// Rejected DLC.
        dlc: u8,
    },
    /// Bitrate must be nonzero.
    #[error("CAN bitrate must be nonzero")]
    ZeroBitrate,
    /// Threshold must be an inclusive percentage.
    #[error("CAN budget threshold {percent}% is outside 1..=100")]
    InvalidThreshold {
        /// Rejected threshold.
        percent: u8,
    },
    /// Checked arithmetic could not represent the plan.
    #[error("CAN bus budget arithmetic overflowed")]
    ArithmeticOverflow,
}

/// Conservative worst-case standard classic-CAN frame bits, including intermission.
pub const fn classic_frame_worst_case_bits(dlc: u8) -> Result<u64, CanBudgetError> {
    if dlc > 8 {
        return Err(CanBudgetError::InvalidDlc { dlc });
    }
    let payload_bits = (dlc as u64) * 8;
    Ok(47 + payload_bits + (33 + payload_bits) / 4)
}

/// Evaluates an already summed planned bit rate against an exact percentage gate.
pub fn evaluate_bus_budget(
    planned_bits_per_second: u64,
    bitrate: u64,
    threshold_percent: u8,
) -> Result<CanBusBudget, CanBudgetError> {
    if bitrate == 0 {
        return Err(CanBudgetError::ZeroBitrate);
    }
    if !(1..=100).contains(&threshold_percent) {
        return Err(CanBudgetError::InvalidThreshold {
            percent: threshold_percent,
        });
    }
    let planned_scaled = planned_bits_per_second
        .checked_mul(100)
        .ok_or(CanBudgetError::ArithmeticOverflow)?;
    let allowed_scaled = bitrate
        .checked_mul(u64::from(threshold_percent))
        .ok_or(CanBudgetError::ArithmeticOverflow)?;
    Ok(CanBusBudget {
        planned_bits_per_second,
        bitrate,
        threshold_percent,
        fits: planned_scaled <= allowed_scaled,
    })
}

/// Sums declared frame flows and applies the configured bus-utilization gate.
pub fn calculate_bus_budget(
    flows: &[CanBudgetFlow],
    bitrate: u64,
    threshold_percent: u8,
) -> Result<CanBusBudget, CanBudgetError> {
    let mut planned = 0_u64;
    for flow in flows {
        let frame_bits = classic_frame_worst_case_bits(flow.dlc)?;
        let flow_bits = frame_bits
            .checked_mul(flow.frames_per_second)
            .ok_or(CanBudgetError::ArithmeticOverflow)?;
        planned = planned
            .checked_add(flow_bits)
            .ok_or(CanBudgetError::ArithmeticOverflow)?;
    }
    evaluate_bus_budget(planned, bitrate, threshold_percent)
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FrameConversionError {
    Extended,
    Remote,
    Error,
    Malformed,
}

fn project_to_socketcan(frame: ClassicCanFrame) -> Result<CanFrame, FrameConversionError> {
    let id =
        socketcan::StandardId::new(frame.id().as_raw()).ok_or(FrameConversionError::Malformed)?;
    CanFrame::new(id, frame.data()).ok_or(FrameConversionError::Malformed)
}

fn socketcan_to_project(frame: CanFrame) -> Result<ClassicCanFrame, FrameConversionError> {
    match frame {
        CanFrame::Remote(_) => Err(FrameConversionError::Remote),
        CanFrame::Error(_) => Err(FrameConversionError::Error),
        CanFrame::Data(data) => {
            let SocketCanId::Standard(id) = data.id() else {
                return Err(FrameConversionError::Extended);
            };
            let id =
                StandardCanId::new(id.as_raw()).map_err(|_| FrameConversionError::Malformed)?;
            ClassicCanFrame::new(id, data.data()).map_err(|_| FrameConversionError::Malformed)
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum BackendRx {
    Data {
        frame: ClassicCanFrame,
        timestamps: CanDiagnosticTimestamps,
    },
    Error {
        bits: u32,
        data: [u8; 8],
    },
    Unsupported,
    Malformed,
}

trait Backend {
    fn try_receive(&mut self) -> io::Result<Option<BackendRx>>;
    fn try_transmit(&mut self, frame: ClassicCanFrame) -> io::Result<()>;
}

trait MonotonicClock {
    fn now(&self) -> MonoTime;
}

#[derive(Debug)]
struct ProcessClock {
    origin: Instant,
}

impl ProcessClock {
    fn new() -> Self {
        Self {
            origin: Instant::now(),
        }
    }
}

impl MonotonicClock for ProcessClock {
    fn now(&self) -> MonoTime {
        MonoTime::from_nanos(duration_nanos(self.origin.elapsed()))
    }
}

fn duration_nanos(duration: std::time::Duration) -> u64 {
    u64::try_from(duration.as_nanos()).unwrap_or(u64::MAX)
}

fn timestamps_to_diagnostics(timestamps: CanTimestamps) -> CanDiagnosticTimestamps {
    let mut diagnostics = CanDiagnosticTimestamps::new();
    if let Some(socket) = timestamps.socket {
        let nanos = socket
            .duration_since(std::time::UNIX_EPOCH)
            .map(duration_nanos)
            .unwrap_or(0);
        let _ = diagnostics.push(DiagnosticTimestamp {
            origin: DiagnosticTimestampOrigin::SocketRealtime,
            nanos,
        });
    }
    if let Some(software) = timestamps.sw {
        let nanos = software
            .duration_since(std::time::UNIX_EPOCH)
            .map(duration_nanos)
            .unwrap_or(0);
        let _ = diagnostics.push(DiagnosticTimestamp {
            origin: DiagnosticTimestampOrigin::KernelSoftwareRealtime,
            nanos,
        });
    }
    if let Some(hardware) = timestamps.hw {
        let _ = diagnostics.push(DiagnosticTimestamp {
            origin: DiagnosticTimestampOrigin::HardwareClock,
            nanos: duration_nanos(hardware),
        });
    }
    diagnostics
}

#[derive(Debug)]
struct LinuxBackend {
    socket: CanSocket,
}

impl LinuxBackend {
    fn open(interface: &str) -> Result<Self, SocketCanError> {
        if interface.is_empty() {
            return Err(SocketCanError::InvalidSetup(
                "interface name must not be empty",
            ));
        }
        let socket = CanSocket::open(interface).map_err(|source| SocketCanError::Io {
            operation: "open",
            source,
        })?;
        socket
            .as_raw_socket()
            .set_cloexec(true)
            .map_err(|source| SocketCanError::Io {
                operation: "set close-on-exec",
                source,
            })?;
        socket
            .set_nonblocking(true)
            .map_err(|source| SocketCanError::Io {
                operation: "set nonblocking",
                source,
            })?;
        socket
            .set_loopback(true)
            .map_err(|source| SocketCanError::Io {
                operation: "enable loopback visibility for peer sockets",
                source,
            })?;
        socket
            .set_recv_own_msgs(false)
            .map_err(|source| SocketCanError::Io {
                operation: "disable receive-own-messages",
                source,
            })?;
        socket
            .set_recv_timestamp(true)
            .map_err(|source| SocketCanError::Io {
                operation: "enable socket receive timestamps",
                source,
            })?;

        let mut timestamp_flags = socketcan::SOF_TIMESTAMPING_RX_SOFTWARE
            | socketcan::SOF_TIMESTAMPING_SOFTWARE
            | socketcan::SOF_TIMESTAMPING_OPT_CMSG;
        if socket.has_hw_timestamps() {
            timestamp_flags |=
                socketcan::SOF_TIMESTAMPING_RX_HARDWARE | socketcan::SOF_TIMESTAMPING_RAW_HARDWARE;
        }
        socket
            .set_timestamping(timestamp_flags)
            .map_err(|source| SocketCanError::Io {
                operation: "enable diagnostic receive timestamps",
                source,
            })?;
        socket
            .set_error_filter(CONTROL_ERROR_MASK)
            .map_err(|source| SocketCanError::Io {
                operation: "install CAN error filter",
                source,
            })?;
        let discovery = discovery_filter_spec();
        socket
            .set_filters(&[CanFilter::new(discovery.id, discovery.mask)])
            .map_err(|source| SocketCanError::Io {
                operation: "install discovery filter",
                source,
            })?;
        Ok(Self { socket })
    }

    fn install_feedback_filters(&self, ids: &[StandardCanId]) -> Result<(), SocketCanError> {
        if ids.is_empty() {
            return Err(SocketCanError::InvalidSetup(
                "at least one feedback identifier is required",
            ));
        }
        for (index, id) in ids.iter().enumerate() {
            if ids[..index].contains(id) {
                return Err(SocketCanError::InvalidSetup(
                    "duplicate feedback identifier",
                ));
            }
        }
        let filters: Vec<CanFilter> = ids
            .iter()
            .map(|id| {
                let spec = exact_feedback_filter_spec(*id);
                CanFilter::new(spec.id, spec.mask)
            })
            .collect();
        self.socket
            .set_filters(&filters)
            .map_err(|source| SocketCanError::Io {
                operation: "install exact feedback filters",
                source,
            })
    }
}

impl Backend for LinuxBackend {
    fn try_receive(&mut self) -> io::Result<Option<BackendRx>> {
        let (frame, timestamps) = match self.socket.read_frame_with_timestamps() {
            Ok(received) => received,
            Err(error) if error.kind() == io::ErrorKind::WouldBlock => return Ok(None),
            Err(error) => return Err(error),
        };
        let timestamps = timestamps_to_diagnostics(timestamps);
        let event = match frame {
            CanFrame::Error(error) => BackendRx::Error {
                bits: error.error_bits(),
                data: error.data().try_into().unwrap_or([0; 8]),
            },
            other => match socketcan_to_project(other) {
                Ok(frame) => BackendRx::Data { frame, timestamps },
                Err(FrameConversionError::Extended | FrameConversionError::Remote) => {
                    BackendRx::Unsupported
                }
                Err(FrameConversionError::Error | FrameConversionError::Malformed) => {
                    BackendRx::Malformed
                }
            },
        };
        Ok(Some(event))
    }

    fn try_transmit(&mut self, frame: ClassicCanFrame) -> io::Result<()> {
        let frame = project_to_socketcan(frame).map_err(|_| {
            io::Error::new(io::ErrorKind::InvalidInput, "invalid classic CAN frame")
        })?;
        self.socket.write_frame(&frame)
    }
}

struct TransportCore<B, C> {
    backend: B,
    clock: C,
    receive_limit: usize,
    health: SocketCanHealth,
}

impl<B: Backend, C: MonotonicClock> TransportCore<B, C> {
    fn new(backend: B, clock: C, receive_limit: usize) -> Result<Self, SocketCanError> {
        if receive_limit == 0 {
            return Err(SocketCanError::InvalidSetup(
                "receive drain limit must be nonzero",
            ));
        }
        let health = SocketCanHealth {
            link_state: LinkState::Up,
            ..SocketCanHealth::default()
        };
        Ok(Self {
            backend,
            clock,
            receive_limit,
            health,
        })
    }

    fn receive(&mut self, output: &mut [CanBusEvent]) -> Result<usize, SocketCanError> {
        let ceiling = output.len().min(self.receive_limit);
        let mut attempts = 0_usize;
        let mut produced = 0_usize;
        while attempts < ceiling {
            let event = match self.backend.try_receive() {
                Ok(Some(event)) => event,
                Ok(None) => break,
                Err(source) => {
                    self.health.rx_io_errors = self.health.rx_io_errors.saturating_add(1);
                    if self.health.link_state == LinkState::Up {
                        self.health.link_state = LinkState::Degraded;
                    }
                    return Err(SocketCanError::Io {
                        operation: "receive frame",
                        source,
                    });
                }
            };
            attempts += 1;
            let observed_monotonic = self.clock.now();
            match event {
                BackendRx::Data { frame, timestamps } => {
                    output[produced] = CanBusEvent::Data(ReceivedCanFrame {
                        frame,
                        observed_monotonic,
                        diagnostic_timestamps: timestamps,
                    });
                    produced += 1;
                    self.health.rx_frames = self.health.rx_frames.saturating_add(1);
                }
                BackendRx::Error { bits, data } => {
                    output[produced] = CanBusEvent::Error(CanErrorEvent {
                        error_bits: bits,
                        raw_data: data,
                        observed_monotonic,
                    });
                    produced += 1;
                    self.health.record_error_bits(bits);
                }
                BackendRx::Unsupported => {
                    self.health.rx_unsupported_frames =
                        self.health.rx_unsupported_frames.saturating_add(1);
                }
                BackendRx::Malformed => {
                    self.health.rx_malformed_frames =
                        self.health.rx_malformed_frames.saturating_add(1);
                }
            }
        }
        if ceiling != 0 && attempts == ceiling {
            self.health.rx_drain_limit_hits = self.health.rx_drain_limit_hits.saturating_add(1);
        }
        Ok(produced)
    }

    fn transmit(&mut self, frames: &[ClassicCanFrame]) -> Result<usize, SocketCanError> {
        if frames.is_empty() {
            return Ok(0);
        }
        if self.health.link_state == LinkState::Faulted {
            return Err(SocketCanError::BusOff);
        }
        for (accepted, frame) in frames.iter().enumerate() {
            if let Err(source) = self.backend.try_transmit(*frame) {
                let backpressure = source.kind() == io::ErrorKind::WouldBlock
                    || source.raw_os_error() == Some(LINUX_ENOBUFS);
                if backpressure {
                    self.health.tx_backpressure = self.health.tx_backpressure.saturating_add(1);
                } else {
                    self.health.tx_errors = self.health.tx_errors.saturating_add(1);
                }
                return Err(SocketCanError::PartialTransmit {
                    accepted,
                    backpressure,
                    source,
                });
            }
            self.health.tx_frames = self.health.tx_frames.saturating_add(1);
        }
        Ok(frames.len())
    }
}

/// Production SocketCAN transport with private Linux/socketcan implementation types.
pub struct SocketCanTransport {
    core: TransportCore<LinuxBackend, ProcessClock>,
    filter_phase: FilterPhase,
}

impl std::fmt::Debug for SocketCanTransport {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("SocketCanTransport")
            .field("filter_phase", &self.filter_phase)
            .field("receive_limit", &self.core.receive_limit)
            .field("health", &self.core.health)
            .finish_non_exhaustive()
    }
}

impl SocketCanTransport {
    /// Opens an already configured named interface in disarmed discovery mode.
    ///
    /// The caller must keep all attached devices disabled. This method changes
    /// socket-local options only and never changes network-interface state.
    pub fn open_discovery(interface: &str, receive_limit: usize) -> Result<Self, SocketCanError> {
        let backend = LinuxBackend::open(interface)?;
        Ok(Self {
            core: TransportCore::new(backend, ProcessClock::new(), receive_limit)?,
            filter_phase: FilterPhase::Discovery,
        })
    }

    /// Installs validated exact feedback filters and performs a bounded residue drain.
    ///
    /// This initialization-only method allocates inside the external socketcan
    /// filter API and must not be called after handoff to a realtime owner.
    pub fn finalize_feedback_filters(
        &mut self,
        ids: &[StandardCanId],
        startup_drain_limit: usize,
    ) -> Result<usize, SocketCanError> {
        if self.filter_phase != FilterPhase::Discovery {
            return Err(SocketCanError::InvalidSetup(
                "feedback filters are already finalized",
            ));
        }
        self.core.backend.install_feedback_filters(ids)?;
        let mut drained = 0_usize;
        while drained < startup_drain_limit {
            match self.core.backend.try_receive() {
                Ok(Some(_)) => drained += 1,
                Ok(None) => break,
                Err(source) => {
                    return Err(SocketCanError::Io {
                        operation: "drain startup residue",
                        source,
                    });
                }
            }
        }
        self.filter_phase = FilterPhase::Feedback;
        Ok(drained)
    }

    /// Returns the current initialization/filter phase.
    #[must_use]
    pub const fn filter_phase(&self) -> FilterPhase {
        self.filter_phase
    }

    /// Returns a timestamp in the same monotonic domain used on received events.
    #[must_use]
    pub fn monotonic_now(&self) -> MonoTime {
        self.core.clock.now()
    }

    /// Returns the detailed non-blocking health snapshot.
    #[must_use]
    pub const fn socketcan_health(&self) -> SocketCanHealth {
        self.core.health
    }
}

impl FrameTransport for SocketCanTransport {
    type RxFrame = CanBusEvent;
    type TxFrame = ClassicCanFrame;
    type Error = SocketCanError;

    fn receive(&mut self, output: &mut [Self::RxFrame]) -> Result<usize, Self::Error> {
        self.core.receive(output)
    }

    fn transmit(&mut self, frames: &[Self::TxFrame]) -> Result<usize, Self::Error> {
        self.core.transmit(frames)
    }

    fn health(&self) -> TransportHealth {
        self.core.health.generic()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use socketcan::{CanDataFrame, CanErrorFrame, CanRemoteFrame, ExtendedId};
    use std::{cell::Cell, collections::VecDeque};

    #[derive(Default)]
    struct FakeBackend {
        rx: VecDeque<io::Result<Option<BackendRx>>>,
        transmitted: Vec<ClassicCanFrame>,
        fail_tx_at: Option<(usize, io::ErrorKind)>,
    }

    impl Backend for FakeBackend {
        fn try_receive(&mut self) -> io::Result<Option<BackendRx>> {
            self.rx.pop_front().unwrap_or(Ok(None))
        }

        fn try_transmit(&mut self, frame: ClassicCanFrame) -> io::Result<()> {
            if let Some((index, kind)) = self.fail_tx_at
                && self.transmitted.len() == index
            {
                return Err(io::Error::from(kind));
            }
            self.transmitted.push(frame);
            Ok(())
        }
    }

    struct FakeClock {
        next: Cell<u64>,
    }

    impl MonotonicClock for FakeClock {
        fn now(&self) -> MonoTime {
            let value = self.next.get();
            self.next.set(value + 1);
            MonoTime::from_nanos(value)
        }
    }

    fn clock() -> FakeClock {
        FakeClock {
            next: Cell::new(10),
        }
    }

    fn frame(id: u16, byte: u8) -> ClassicCanFrame {
        ClassicCanFrame::new(StandardCanId::new(id).unwrap(), &[byte]).unwrap()
    }

    fn placeholder() -> CanBusEvent {
        CanBusEvent::Data(ReceivedCanFrame {
            frame: frame(1, 0),
            observed_monotonic: MonoTime::from_nanos(0),
            diagnostic_timestamps: CanDiagnosticTimestamps::new(),
        })
    }

    #[test]
    fn classic_frame_conversion_round_trips_and_rejects_non_data_sff() {
        let project = frame(0x321, 0x5a);
        assert_eq!(
            socketcan_to_project(project_to_socketcan(project).unwrap()),
            Ok(project)
        );

        let extended =
            CanFrame::Data(CanDataFrame::new(ExtendedId::new(0x321).unwrap(), &[1]).unwrap());
        assert_eq!(
            socketcan_to_project(extended),
            Err(FrameConversionError::Extended)
        );
        let remote = CanFrame::Remote(
            CanRemoteFrame::new_remote(socketcan::StandardId::new(1).unwrap(), 1).unwrap(),
        );
        assert_eq!(
            socketcan_to_project(remote),
            Err(FrameConversionError::Remote)
        );
        let error = CanFrame::Error(CanErrorFrame::new_error(CAN_ERROR_ACK, &[0; 8]).unwrap());
        assert_eq!(
            socketcan_to_project(error),
            Err(FrameConversionError::Error)
        );
    }

    #[test]
    fn filters_reject_extended_and_rtr_aliases() {
        assert_eq!(
            discovery_filter_spec(),
            CanFilterSpec {
                id: 0,
                mask: socketcan::frame::CAN_EFF_FLAG | socketcan::frame::CAN_RTR_FLAG,
            }
        );
        let spec = exact_feedback_filter_spec(StandardCanId::new(0x321).unwrap());
        assert_eq!(spec.id, 0x321);
        assert_eq!(
            spec.mask,
            socketcan::frame::CAN_SFF_MASK
                | socketcan::frame::CAN_EFF_FLAG
                | socketcan::frame::CAN_RTR_FLAG
        );
    }

    #[test]
    fn bounded_receive_preserves_monotonic_time_and_hits_ceiling() {
        let mut backend = FakeBackend::default();
        for byte in 1..=3 {
            backend.rx.push_back(Ok(Some(BackendRx::Data {
                frame: frame(0x10, byte),
                timestamps: CanDiagnosticTimestamps::new(),
            })));
        }
        let mut transport = TransportCore::new(backend, clock(), 2).unwrap();
        let mut output = [placeholder(), placeholder(), placeholder()];
        assert_eq!(transport.receive(&mut output).unwrap(), 2);
        assert_eq!(transport.health.rx_frames, 2);
        assert_eq!(transport.health.rx_drain_limit_hits, 1);
        match output[0] {
            CanBusEvent::Data(received) => {
                assert_eq!(received.frame.data(), &[1]);
                assert_eq!(received.observed_monotonic, MonoTime::from_nanos(10));
            }
            CanBusEvent::Error(_) => panic!("expected data frame"),
        }
        assert_eq!(transport.backend.rx.len(), 1);
    }

    #[test]
    fn unsupported_frames_consume_bounded_receive_attempts() {
        let mut backend = FakeBackend::default();
        backend.rx.push_back(Ok(Some(BackendRx::Unsupported)));
        backend.rx.push_back(Ok(Some(BackendRx::Malformed)));
        backend.rx.push_back(Ok(Some(BackendRx::Data {
            frame: frame(1, 1),
            timestamps: CanDiagnosticTimestamps::new(),
        })));
        let mut transport = TransportCore::new(backend, clock(), 2).unwrap();
        let mut output = [placeholder(), placeholder()];
        assert_eq!(transport.receive(&mut output).unwrap(), 0);
        assert_eq!(transport.health.rx_unsupported_frames, 1);
        assert_eq!(transport.health.rx_malformed_frames, 1);
        assert_eq!(transport.backend.rx.len(), 1);
    }

    #[test]
    fn one_shot_transmit_reports_accepted_prefix_and_backpressure() {
        let backend = FakeBackend {
            fail_tx_at: Some((1, io::ErrorKind::WouldBlock)),
            ..FakeBackend::default()
        };
        let mut transport = TransportCore::new(backend, clock(), 8).unwrap();
        let frames = [frame(1, 1), frame(2, 2), frame(3, 3)];
        match transport.transmit(&frames).unwrap_err() {
            SocketCanError::PartialTransmit {
                accepted,
                backpressure,
                ..
            } => {
                assert_eq!(accepted, 1);
                assert!(backpressure);
            }
            other => panic!("unexpected error: {other}"),
        }
        assert_eq!(transport.backend.transmitted, &[frames[0]]);
        assert_eq!(transport.health.tx_frames, 1);
        assert_eq!(transport.health.tx_backpressure, 1);
    }

    #[test]
    fn combined_bus_off_event_faults_link_and_inhibits_tx() {
        let bits = CAN_ERROR_BUS_OFF | CAN_ERROR_ACK;
        let mut backend = FakeBackend::default();
        backend
            .rx
            .push_back(Ok(Some(BackendRx::Error { bits, data: [9; 8] })));
        let mut transport = TransportCore::new(backend, clock(), 8).unwrap();
        let mut output = [placeholder()];
        assert_eq!(transport.receive(&mut output).unwrap(), 1);
        assert_eq!(transport.health.link_state, LinkState::Faulted);
        assert_eq!(transport.health.can_bus_off_events, 1);
        assert_eq!(transport.health.can_ack_events, 1);
        assert!(matches!(
            transport.transmit(&[frame(1, 1)]),
            Err(SocketCanError::BusOff)
        ));
        match output[0] {
            CanBusEvent::Error(event) => {
                assert_eq!(event.error_bits, bits);
                assert_eq!(event.raw_data, [9; 8]);
            }
            CanBusEvent::Data(_) => panic!("expected error event"),
        }
    }

    #[test]
    fn all_subscribed_error_bits_are_counted_independently() {
        let mut health = SocketCanHealth {
            link_state: LinkState::Up,
            ..SocketCanHealth::default()
        };
        health.record_error_bits(CONTROL_ERROR_MASK);
        assert_eq!(health.can_tx_timeout_events, 1);
        assert_eq!(health.can_controller_events, 1);
        assert_eq!(health.can_transceiver_events, 1);
        assert_eq!(health.can_ack_events, 1);
        assert_eq!(health.can_bus_off_events, 1);
        assert_eq!(health.can_restarted_events, 1);
        assert_eq!(health.link_state, LinkState::Faulted);
        assert_eq!(health.kernel_rx_dropped, MetricValue::Unavailable);
        assert_eq!(health.generic().rx_dropped, MetricValue::Unavailable);
    }

    #[test]
    fn timestamp_domains_are_tagged_without_affecting_freshness() {
        let timestamps = CanTimestamps {
            socket: Some(std::time::UNIX_EPOCH + std::time::Duration::from_nanos(11)),
            sw: Some(std::time::UNIX_EPOCH + std::time::Duration::from_nanos(12)),
            hw: Some(std::time::Duration::from_nanos(13)),
        };
        let diagnostics = timestamps_to_diagnostics(timestamps);
        assert_eq!(
            diagnostics.entries(),
            &[
                Some(DiagnosticTimestamp {
                    origin: DiagnosticTimestampOrigin::SocketRealtime,
                    nanos: 11,
                }),
                Some(DiagnosticTimestamp {
                    origin: DiagnosticTimestampOrigin::KernelSoftwareRealtime,
                    nanos: 12,
                }),
                Some(DiagnosticTimestamp {
                    origin: DiagnosticTimestampOrigin::HardwareClock,
                    nanos: 13,
                }),
            ]
        );
    }

    #[test]
    fn worst_case_frame_bits_cover_every_classic_dlc() {
        let expected = [55, 65, 75, 85, 95, 105, 115, 125, 135];
        for (dlc, bits) in expected.into_iter().enumerate() {
            assert_eq!(classic_frame_worst_case_bits(dlc as u8), Ok(bits));
        }
        assert!(matches!(
            classic_frame_worst_case_bits(9),
            Err(CanBudgetError::InvalidDlc { dlc: 9 })
        ));
    }

    #[test]
    fn budget_is_checked_and_uses_exact_seventy_percent_gate() {
        assert!(
            evaluate_bus_budget(700, 1_000, DEFAULT_BUS_BUDGET_PERCENT)
                .unwrap()
                .fits
        );
        assert!(
            !evaluate_bus_budget(701, 1_000, DEFAULT_BUS_BUDGET_PERCENT)
                .unwrap()
                .fits
        );
        assert_eq!(
            calculate_bus_budget(
                &[CanBudgetFlow {
                    dlc: 8,
                    frames_per_second: 2_000,
                }],
                1_000_000,
                DEFAULT_BUS_BUDGET_PERCENT,
            )
            .unwrap()
            .planned_bits_per_second,
            270_000
        );
        assert_eq!(
            calculate_bus_budget(
                &[CanBudgetFlow {
                    dlc: 8,
                    frames_per_second: u64::MAX,
                }],
                1,
                DEFAULT_BUS_BUDGET_PERCENT,
            ),
            Err(CanBudgetError::ArithmeticOverflow)
        );
    }

    #[test]
    #[ignore = "requires an existing nonphysical vcan interface named by RS_CONTROL_VCAN_INTERFACE"]
    fn opens_existing_vcan_without_configuring_or_transmitting() {
        let interface = std::env::var("RS_CONTROL_VCAN_INTERFACE")
            .expect("set RS_CONTROL_VCAN_INTERFACE to an existing vcan interface");
        let transport = SocketCanTransport::open_discovery(&interface, 8).unwrap();
        assert_eq!(transport.filter_phase(), FilterPhase::Discovery);
    }
}
