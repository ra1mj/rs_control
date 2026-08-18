//! Host-only, wiring-only motion runtime composition.
//!
//! The control worker is called with an injected monotonic tick. It owns no
//! scheduler, clock, transport, filesystem, logging, or motor capability.

use arrayvec::ArrayVec;
use control_core::{LeaseCacheError, LeaseEffect, LeaseEvent, MotionControlWorker};
use message_model::{CommandEnvelope, ResourceId, SourceId};
use middleware_api::{
    ChannelCounters, EventPublishError, EventPublisher, EventSubscriber, LatestPublish,
    LatestPublisher, LatestSubscriber,
};
use middleware_local::{
    LocalEventPublisher, LocalEventSubscriber, LocalLatestPublisher, LocalLatestSubscriber,
    latest_channel, ordered_channel,
};
use telemetry_recorder::{IngressOutcome, TelemetryIngress, TelemetryRecord};
use thiserror::Error;

pub use control_core::{
    ControlState as SyntheticState, MotionDecision as FakeSinkDecision, MotionDiagnostics,
    RuntimeAvailability, SafeStopReason,
};

/// Bounded ordered command queue capacity for host composition tests.
pub const COMMAND_QUEUE_CAPACITY: usize = 8;
/// Fixed number of fake-sink decisions retained for test inspection.
pub const FAKE_SINK_CAPACITY: usize = 64;

/// Explicit host roles. A target scheduler is outside this host-only app.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum HostRole {
    /// Synthetic, non-device state publisher.
    SyntheticIo,
    /// Injected-tick, bounded lease/control worker.
    Control,
    /// Non-realtime diagnostic and telemetry drain owner.
    DiagnosticDrain,
    /// Motor-disabled test double for output decisions.
    FakeSink,
}

/// Fixed-capacity, motor-disabled output test double.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct FakeSink {
    decisions: ArrayVec<FakeSinkDecision, FAKE_SINK_CAPACITY>,
}

impl FakeSink {
    fn push(&mut self, decision: FakeSinkDecision) {
        if self.decisions.try_push(decision).is_err() {
            let _ = self.decisions.remove(0);
            self.decisions
                .try_push(decision)
                .expect("removing one bounded fake decision restores capacity");
        }
    }

    /// Returns retained decisions in application order.
    #[must_use]
    pub fn decisions(&self) -> &[FakeSinkDecision] {
        self.decisions.as_slice()
    }

    /// Returns the most recent decision.
    #[must_use]
    pub fn last(&self) -> Option<FakeSinkDecision> {
        self.decisions.last().copied()
    }
}

/// Per-tick delivery outcome represented without formatted logs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ChannelOutcome {
    /// No fresh input was read during this tick.
    Unchanged,
    /// A latest-value generation was read or published.
    Latest,
    /// A latest publisher replaced an unread generation.
    Overwritten,
    /// An ordered input was read or queued.
    Ordered,
    /// A bounded ordered queue was full.
    Full,
    /// The opposite endpoint was gone.
    Disconnected,
}

/// Copyable host wiring evidence. Rendering belongs to the drain role.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RuntimeDiagnostics {
    /// Application-owned safety and typed health evidence.
    pub control: MotionDiagnostics,
    /// Last observed state-channel outcome.
    pub state_outcome: ChannelOutcome,
    /// Last observed command-channel outcome.
    pub command_outcome: ChannelOutcome,
    /// Previous diagnostic-channel delivery outcome.
    pub diagnostic_outcome: ChannelOutcome,
    /// Latest-state channel counters.
    pub state_counters: ChannelCounters,
    /// Ordered-command channel counters.
    pub command_counters: ChannelCounters,
}

/// Errors exposed at the composition boundary before role startup.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum MotionRuntimeError {
    /// Ordered command storage requires a nonzero capacity.
    #[error("motion command queue capacity must be nonzero")]
    CommandQueue,
}

/// Wiring-only host composition of synthetic I/O, control, diagnostics, and fake output.
pub struct HostMotionRuntime {
    state_publisher: LocalLatestPublisher<SyntheticState>,
    state_subscriber: LocalLatestSubscriber<SyntheticState>,
    command_publisher: LocalEventPublisher<CommandEnvelope>,
    command_subscriber: LocalEventSubscriber<CommandEnvelope>,
    diagnostic_publisher: LocalLatestPublisher<RuntimeDiagnostics>,
    diagnostic_subscriber: Option<LocalLatestSubscriber<RuntimeDiagnostics>>,
    control: MotionControlWorker,
    fake_sink: FakeSink,
    latest_state: Option<SyntheticState>,
    active_command: Option<CommandEnvelope>,
    availability: RuntimeAvailability,
    state_outcome: ChannelOutcome,
    command_outcome: ChannelOutcome,
    diagnostic_outcome: ChannelOutcome,
    telemetry: TelemetryIngress,
}

impl HostMotionRuntime {
    /// Constructs all fixed-capacity local ports before any control tick.
    pub fn new(
        resource_id: ResourceId,
        component_id: SourceId,
    ) -> Result<Self, MotionRuntimeError> {
        let (state_publisher, state_subscriber) = latest_channel();
        let (command_publisher, command_subscriber) = ordered_channel(COMMAND_QUEUE_CAPACITY)
            .map_err(|_| MotionRuntimeError::CommandQueue)?;
        let (diagnostic_publisher, diagnostic_subscriber) = latest_channel();
        Ok(Self {
            state_publisher,
            state_subscriber,
            command_publisher,
            command_subscriber,
            diagnostic_publisher,
            diagnostic_subscriber: Some(diagnostic_subscriber),
            control: MotionControlWorker::new(resource_id, component_id),
            fake_sink: FakeSink::default(),
            latest_state: None,
            active_command: None,
            availability: RuntimeAvailability::default(),
            state_outcome: ChannelOutcome::Unchanged,
            command_outcome: ChannelOutcome::Unchanged,
            diagnostic_outcome: ChannelOutcome::Unchanged,
            telemetry: TelemetryIngress::default(),
        })
    }

    /// Returns the explicit roles which a host harness may place on separate threads.
    #[must_use]
    pub const fn roles() -> [HostRole; 4] {
        [
            HostRole::SyntheticIo,
            HostRole::Control,
            HostRole::DiagnosticDrain,
            HostRole::FakeSink,
        ]
    }

    /// Applies a lease event exclusively through the control-core authority cache.
    pub fn apply_lease_event(
        &mut self,
        event: LeaseEvent,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<1>, LeaseCacheError> {
        self.control.apply_lease_event(event, now_monotonic_ns)
    }

    /// Publishes a synthetic latest state without waiting for the control role.
    pub fn publish_synthetic_state(&mut self, state: SyntheticState) -> ChannelOutcome {
        let outcome = match self.state_publisher.try_publish(state) {
            LatestPublish::Published {
                overwritten: true, ..
            } => ChannelOutcome::Overwritten,
            LatestPublish::Published { .. } => ChannelOutcome::Latest,
            LatestPublish::Disconnected => {
                self.availability.bridge = false;
                ChannelOutcome::Disconnected
            }
        };
        self.state_outcome = outcome;
        outcome
    }

    /// Attempts to queue a command without waiting for the control role.
    pub fn submit_command(&mut self, command: CommandEnvelope) -> ChannelOutcome {
        let outcome = match self.command_publisher.try_publish(command) {
            Ok(_) => ChannelOutcome::Ordered,
            Err(EventPublishError::Full(_)) => ChannelOutcome::Full,
            Err(EventPublishError::Disconnected(_)) => {
                self.availability.bridge = false;
                ChannelOutcome::Disconnected
            }
        };
        self.command_outcome = outcome;
        outcome
    }

    /// Runs one nonblocking control tick with an injected local monotonic time.
    pub fn step_at(&mut self, now_monotonic_ns: u64) -> FakeSinkDecision {
        if let Some(sample) = self.state_subscriber.try_read_latest() {
            self.latest_state = Some(sample.value);
            self.state_outcome = ChannelOutcome::Latest;
        }
        let mut new_command = false;
        if let Some(sample) = self.command_subscriber.try_receive() {
            self.active_command = Some(sample.value);
            self.command_outcome = ChannelOutcome::Ordered;
            new_command = true;
        }
        let decision = self.control.step_at(
            self.latest_state,
            self.active_command,
            new_command,
            self.availability,
            now_monotonic_ns,
        );
        if new_command && !matches!(decision, FakeSinkDecision::Apply { .. }) {
            self.active_command = None;
        }
        self.fake_sink.push(decision);
        let diagnostics = RuntimeDiagnostics {
            control: self.control.diagnostics(),
            state_outcome: self.state_outcome,
            command_outcome: self.command_outcome,
            diagnostic_outcome: self.diagnostic_outcome,
            state_counters: self.state_publisher.counters(),
            command_counters: self.command_publisher.counters(),
        };
        self.diagnostic_outcome = match self.diagnostic_publisher.try_publish(diagnostics) {
            LatestPublish::Published {
                overwritten: true, ..
            } => ChannelOutcome::Overwritten,
            LatestPublish::Published { .. } => ChannelOutcome::Latest,
            LatestPublish::Disconnected => {
                self.availability.diagnostics = false;
                ChannelOutcome::Disconnected
            }
        };
        decision
    }

    /// Reads one diagnostic snapshot on the non-realtime drain side.
    pub fn drain_diagnostics(&mut self) -> Option<RuntimeDiagnostics> {
        self.diagnostic_subscriber
            .as_mut()?
            .try_read_latest()
            .map(|sample| sample.value)
    }

    /// Drops the non-realtime diagnostic endpoint; the next publication reports loss.
    pub fn disconnect_diagnostic_drain(&mut self) {
        self.diagnostic_subscriber = None;
    }

    /// Queues telemetry evidence without coupling it to authority or fake output.
    pub fn queue_telemetry(&mut self, record: TelemetryRecord) -> IngressOutcome {
        self.telemetry.try_push(record)
    }

    /// Gives the non-realtime recorder owner bounded queued evidence.
    #[must_use]
    pub fn telemetry_ingress(&mut self) -> &mut TelemetryIngress {
        &mut self.telemetry
    }

    /// Reports a recorder boundary failure for the next bounded control tick.
    pub fn set_recorder_available(&mut self, available: bool) {
        self.availability.recorder = available;
    }

    /// Reports bridge loss for the next bounded control tick.
    pub fn set_bridge_available(&mut self, available: bool) {
        self.availability.bridge = available;
    }

    /// Reports a control-worker loss for the next bounded control tick.
    pub fn set_control_available(&mut self, available: bool) {
        self.availability.control = available;
    }

    /// Emits SafeStop before diagnostic or recorder teardown.
    pub fn shutdown_at(&mut self, now_monotonic_ns: u64) -> FakeSinkDecision {
        self.availability.bridge = false;
        let decision = self
            .control
            .force_safe_stop(SafeStopReason::Shutdown, now_monotonic_ns);
        self.fake_sink.push(decision);
        decision
    }

    /// Exposes motor-disabled test decisions only.
    #[must_use]
    pub fn fake_sink(&self) -> &FakeSink {
        &self.fake_sink
    }
}

#[cfg(test)]
mod tests {
    use std::{
        sync::{Arc, Barrier},
        thread,
    };

    use allocation_counter::measure;
    use control_core::LeaseEvent;
    use message_model::{
        AsciiId, ClockState, ClockStatus, CommandEnvelopeFields, DataQuality, Header, HeaderFields,
        Id128, ResourceLease, ResourceLeaseFields,
    };

    use super::*;

    fn identifier(value: &str) -> SourceId {
        AsciiId::parse(value).unwrap()
    }

    fn token(value: u8) -> Id128 {
        Id128::new([value; 16]).unwrap()
    }

    fn lease(expires_monotonic_ns: u64) -> ResourceLease {
        ResourceLease::new(ResourceLeaseFields {
            resource_id: identifier("chassis"),
            owner_id: identifier("navigation"),
            lease_id: token(3),
            session_epoch: 9,
            expires_monotonic_ns,
        })
        .unwrap()
    }

    fn command(sequence: u64, deadline_monotonic_ns: u64) -> CommandEnvelope {
        command_with_lease(sequence, deadline_monotonic_ns, token(3))
    }

    fn command_with_lease(
        sequence: u64,
        deadline_monotonic_ns: u64,
        lease_id: Id128,
    ) -> CommandEnvelope {
        CommandEnvelope::new(CommandEnvelopeFields {
            header: Header::new(HeaderFields {
                source_id: identifier("navigation"),
                boot_id: token(4),
                sequence,
                source_monotonic_ns: 1,
                publish_unix_ns: None,
                acquisition_timestamp: None,
                valid_for_ns: 100,
                frame_id: None,
                clock_status: ClockStatus {
                    state: ClockState::Local,
                    uncertainty_ns: 0,
                },
                quality: DataQuality::Valid,
            })
            .unwrap(),
            resource_id: identifier("chassis"),
            owner_id: identifier("navigation"),
            lease_id,
            session_epoch: 9,
            command_id: token(5),
            deadline_monotonic_ns,
        })
        .unwrap()
    }

    fn ready_runtime() -> HostMotionRuntime {
        let mut runtime =
            HostMotionRuntime::new(identifier("chassis"), identifier("motion-rt")).unwrap();
        runtime
            .apply_lease_event(LeaseEvent::StartSession { session_epoch: 9 }, 10)
            .unwrap();
        runtime
            .apply_lease_event(LeaseEvent::Grant(lease(1_000)), 10)
            .unwrap();
        runtime
    }

    #[test]
    fn host_roles_are_explicit_and_can_be_separated() {
        let roles = HostMotionRuntime::roles();
        assert_eq!(roles.len(), 4);
        let barrier = Arc::new(Barrier::new(4));
        let mut threads = Vec::new();
        for role in roles {
            let barrier = Arc::clone(&barrier);
            threads.push(thread::spawn(move || {
                barrier.wait();
                (role, thread::current().id())
            }));
        }
        let ids: Vec<_> = threads
            .into_iter()
            .map(|thread| thread.join().unwrap().1)
            .collect();
        for (index, current) in ids.iter().enumerate() {
            for other in &ids[..index] {
                assert_ne!(current, other);
            }
        }
    }

    #[test]
    fn current_exact_lease_applies_but_stale_expired_and_loss_paths_stop() {
        let mut runtime = ready_runtime();
        assert_eq!(
            runtime.publish_synthetic_state(SyntheticState {
                sequence: 1,
                valid_until_monotonic_ns: 100,
                available: true,
            }),
            ChannelOutcome::Latest
        );
        assert_eq!(
            runtime.submit_command(command(1, 90)),
            ChannelOutcome::Ordered
        );
        assert!(matches!(
            runtime.step_at(20),
            FakeSinkDecision::Apply { .. }
        ));

        assert_eq!(
            runtime.submit_command(command(1, 90)),
            ChannelOutcome::Ordered
        );
        assert!(matches!(
            runtime.step_at(21),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::CommandSequence,
                ..
            }
        ));
        assert_eq!(
            runtime.submit_command(command(2, 22)),
            ChannelOutcome::Ordered
        );
        assert!(matches!(
            runtime.step_at(22),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::DeadlineExpired,
                ..
            }
        ));
        runtime.set_bridge_available(false);
        assert!(matches!(
            runtime.step_at(23),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::BridgeLost,
                ..
            }
        ));

        let mut wrong_authority = ready_runtime();
        wrong_authority.publish_synthetic_state(SyntheticState {
            sequence: 1,
            valid_until_monotonic_ns: 100,
            available: true,
        });
        wrong_authority.submit_command(command_with_lease(1, 90, token(6)));
        assert!(matches!(
            wrong_authority.step_at(20),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::AuthorityRejected,
                ..
            }
        ));
    }

    #[test]
    fn stalled_diagnostics_do_not_block_control_but_lost_consumer_stops_next_tick() {
        let mut runtime = ready_runtime();
        runtime.publish_synthetic_state(SyntheticState {
            sequence: 1,
            valid_until_monotonic_ns: 100,
            available: true,
        });
        runtime.submit_command(command(1, 90));
        assert!(matches!(
            runtime.step_at(20),
            FakeSinkDecision::Apply { .. }
        ));
        for tick in 21..40 {
            assert!(matches!(
                runtime.step_at(tick),
                FakeSinkDecision::Apply { .. }
            ));
        }
        assert!(runtime.drain_diagnostics().is_some());
        runtime.disconnect_diagnostic_drain();
        assert!(matches!(
            runtime.step_at(40),
            FakeSinkDecision::Apply { .. }
        ));
        assert!(matches!(
            runtime.step_at(41),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::DiagnosticsUnavailable,
                ..
            }
        ));
    }

    #[test]
    fn recorder_and_control_worker_loss_fail_closed_without_authority_refresh() {
        let mut recorder_lost = ready_runtime();
        recorder_lost.publish_synthetic_state(SyntheticState {
            sequence: 1,
            valid_until_monotonic_ns: 100,
            available: true,
        });
        recorder_lost.submit_command(command(1, 90));
        assert!(matches!(
            recorder_lost.step_at(20),
            FakeSinkDecision::Apply { .. }
        ));
        recorder_lost.set_recorder_available(false);
        assert!(matches!(
            recorder_lost.step_at(21),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::RecorderUnavailable,
                ..
            }
        ));

        let mut worker_lost = ready_runtime();
        worker_lost.set_control_available(false);
        assert!(matches!(
            worker_lost.step_at(22),
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::ControlWorkerLost,
                ..
            }
        ));
    }

    #[test]
    fn warm_control_and_diagnostic_publication_do_not_allocate() {
        let mut runtime = ready_runtime();
        runtime.publish_synthetic_state(SyntheticState {
            sequence: 1,
            valid_until_monotonic_ns: 100,
            available: true,
        });
        runtime.submit_command(command(1, 90));
        assert!(matches!(
            runtime.step_at(20),
            FakeSinkDecision::Apply { .. }
        ));
        let allocations = measure(|| {
            for tick in 21..30 {
                assert!(matches!(
                    runtime.step_at(tick),
                    FakeSinkDecision::Apply { .. }
                ));
            }
        });
        assert_eq!(allocations.count_total, 0);
    }

    #[test]
    fn shutdown_stops_before_downstream_roles_are_torn_down() {
        let mut runtime = ready_runtime();
        let decision = runtime.shutdown_at(50);
        assert_eq!(
            decision,
            FakeSinkDecision::SafeStop {
                reason: SafeStopReason::Shutdown,
                tick_monotonic_ns: 50,
            }
        );
        assert_eq!(runtime.fake_sink().last(), Some(decision));
    }
}
