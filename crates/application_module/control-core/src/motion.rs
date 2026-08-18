//! Bounded motion authority and safe-stop decisions for a caller-owned control tick.

use message_model::{
    CommandEnvelope, DataQuality, Fault, FaultRecoverability, FaultSeverity, Id128, LifecycleState,
    ResourceId, SemanticError, SourceId,
};

use crate::{LeaseCacheError, LeaseEffect, LeaseEnforcementCache, LeaseEvent};

/// Copyable state consumed by one motion control tick.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ControlState {
    /// Source-local state sequence retained for diagnostics.
    pub sequence: u64,
    /// Exclusive local-monotonic state freshness deadline.
    pub valid_until_monotonic_ns: u64,
    /// Whether the source is currently available.
    pub available: bool,
}

/// Availability observed at bounded runtime boundaries before a control tick.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RuntimeAvailability {
    /// State/command bridge is present.
    pub bridge: bool,
    /// A diagnostic consumer endpoint remains attached.
    pub diagnostics: bool,
    /// The non-realtime recorder service is accepting evidence.
    pub recorder: bool,
    /// The control worker itself remains available.
    pub control: bool,
}

impl Default for RuntimeAvailability {
    fn default() -> Self {
        Self {
            bridge: true,
            diagnostics: true,
            recorder: true,
            control: true,
        }
    }
}

/// Bounded local safe-stop reason.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SafeStopReason {
    /// No usable control state was available.
    StateUnavailable,
    /// State freshness reached its exclusive deadline.
    StateExpired,
    /// No current command is available.
    CommandUnavailable,
    /// A new command repeated or regressed its source-boot sequence.
    CommandSequence,
    /// A command's semantic/lease authority was rejected.
    AuthorityRejected,
    /// An otherwise-current command or lease expired.
    DeadlineExpired,
    /// The bridge endpoint was lost.
    BridgeLost,
    /// The diagnostic endpoint was lost.
    DiagnosticsUnavailable,
    /// The recorder health boundary was lost.
    RecorderUnavailable,
    /// The caller reported the control worker lost.
    ControlWorkerLost,
    /// Ordered shutdown has begun.
    Shutdown,
}

impl SafeStopReason {
    const fn fault_code(self) -> u32 {
        match self {
            Self::StateUnavailable => 0x4d01,
            Self::StateExpired => 0x4d02,
            Self::CommandUnavailable => 0x4d03,
            Self::CommandSequence => 0x4d04,
            Self::AuthorityRejected => 0x4d05,
            Self::DeadlineExpired => 0x4d06,
            Self::BridgeLost => 0x4d07,
            Self::DiagnosticsUnavailable => 0x4d08,
            Self::RecorderUnavailable => 0x4d09,
            Self::ControlWorkerLost => 0x4d0a,
            Self::Shutdown => 0x4d0b,
        }
    }
}

/// A semantic motion decision with no device or transport capability.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MotionDecision {
    /// Exact current authority passed all control checks.
    Apply {
        /// Command identity retained for downstream correlation.
        command_id: Id128,
        /// Caller-injected local control tick.
        tick_monotonic_ns: u64,
    },
    /// The control contract failed closed.
    SafeStop {
        /// Typed inhibition cause.
        reason: SafeStopReason,
        /// Caller-injected local control tick.
        tick_monotonic_ns: u64,
    },
}

/// Copyable control-owned safety and health evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct MotionDiagnostics {
    /// Number of caller-injected ticks.
    pub ticks: u64,
    /// Number of exact-authority Apply decisions.
    pub applied: u64,
    /// Number of fail-closed decisions.
    pub safe_stops: u64,
    /// Availability captured at the latest decision point.
    pub availability: RuntimeAvailability,
    /// Existing semantic lifecycle owner, never inferred from log text.
    pub lifecycle: LifecycleState,
    /// Existing semantic fault owner, when the latest decision inhibited motion.
    pub fault: Option<Fault>,
    /// Typed local inhibition cause.
    pub safe_stop_reason: Option<SafeStopReason>,
}

impl MotionDiagnostics {
    const fn empty() -> Self {
        Self {
            ticks: 0,
            applied: 0,
            safe_stops: 0,
            availability: RuntimeAvailability {
                bridge: true,
                diagnostics: true,
                recorder: true,
                control: true,
            },
            lifecycle: LifecycleState::Starting,
            fault: None,
            safe_stop_reason: None,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct CommandOrder {
    boot_id: Id128,
    sequence: u64,
}

/// Single-owner fixed-capacity motion safety worker.
pub struct MotionControlWorker {
    cache: LeaseEnforcementCache<1>,
    resource_id: ResourceId,
    component_id: SourceId,
    last_command_order: Option<CommandOrder>,
    diagnostics: MotionDiagnostics,
}

impl MotionControlWorker {
    /// Creates a worker with no authority until a lease event is applied.
    #[must_use]
    pub fn new(resource_id: ResourceId, component_id: SourceId) -> Self {
        Self {
            cache: LeaseEnforcementCache::new(),
            resource_id,
            component_id,
            last_command_order: None,
            diagnostics: MotionDiagnostics::empty(),
        }
    }

    /// Applies an explicit authority event on the owning control worker.
    pub fn apply_lease_event(
        &mut self,
        event: LeaseEvent,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<1>, LeaseCacheError> {
        self.cache.apply(event, now_monotonic_ns)
    }

    /// Executes one injected-tick decision without I/O, clocks, allocation, or waits.
    pub fn step_at(
        &mut self,
        state: Option<ControlState>,
        command: Option<CommandEnvelope>,
        new_command: bool,
        availability: RuntimeAvailability,
        now_monotonic_ns: u64,
    ) -> MotionDecision {
        self.diagnostics.ticks = self.diagnostics.ticks.saturating_add(1);
        self.diagnostics.availability = availability;
        self.diagnostics.fault = None;
        self.diagnostics.safe_stop_reason = None;
        let _ = self.cache.apply(LeaseEvent::Tick, now_monotonic_ns);

        if !availability.control {
            return self.safe_stop(SafeStopReason::ControlWorkerLost, None, now_monotonic_ns);
        }
        if !availability.bridge {
            return self.safe_stop(SafeStopReason::BridgeLost, command, now_monotonic_ns);
        }
        if !availability.diagnostics {
            return self.safe_stop(
                SafeStopReason::DiagnosticsUnavailable,
                command,
                now_monotonic_ns,
            );
        }
        if !availability.recorder {
            return self.safe_stop(
                SafeStopReason::RecorderUnavailable,
                command,
                now_monotonic_ns,
            );
        }
        self.evaluate_state_and_command(state, command, new_command, now_monotonic_ns)
    }

    /// Emits a local safe stop before a caller tears down downstream roles.
    pub fn force_safe_stop(
        &mut self,
        reason: SafeStopReason,
        now_monotonic_ns: u64,
    ) -> MotionDecision {
        self.safe_stop(reason, None, now_monotonic_ns)
    }

    /// Returns fixed control-owned evidence after the latest decision.
    #[must_use]
    pub const fn diagnostics(&self) -> MotionDiagnostics {
        self.diagnostics
    }

    fn evaluate_state_and_command(
        &mut self,
        state: Option<ControlState>,
        command: Option<CommandEnvelope>,
        new_command: bool,
        now_monotonic_ns: u64,
    ) -> MotionDecision {
        let Some(state) = state else {
            return self.safe_stop(SafeStopReason::StateUnavailable, command, now_monotonic_ns);
        };
        if !state.available {
            return self.safe_stop(SafeStopReason::StateUnavailable, command, now_monotonic_ns);
        }
        if state.valid_until_monotonic_ns <= now_monotonic_ns {
            return self.safe_stop(SafeStopReason::StateExpired, command, now_monotonic_ns);
        }
        let Some(command) = command else {
            return self.safe_stop(SafeStopReason::CommandUnavailable, None, now_monotonic_ns);
        };
        if command.resource_id() != self.resource_id
            || command.header().source_id() != command.owner_id()
            || command.header().quality() != DataQuality::Valid
            || (new_command && !self.accept_command_order(command))
        {
            return self.safe_stop(
                SafeStopReason::CommandSequence,
                Some(command),
                now_monotonic_ns,
            );
        }
        match self.cache.authorize(&command, now_monotonic_ns) {
            Ok(_) => {
                if new_command {
                    self.last_command_order = Some(CommandOrder {
                        boot_id: command.header().boot_id(),
                        sequence: command.header().sequence(),
                    });
                }
                self.diagnostics.applied = self.diagnostics.applied.saturating_add(1);
                self.diagnostics.lifecycle = LifecycleState::Active;
                MotionDecision::Apply {
                    command_id: command.command_id(),
                    tick_monotonic_ns: now_monotonic_ns,
                }
            }
            Err(error) => {
                let reason = if matches!(
                    error,
                    LeaseCacheError::Semantic(SemanticError::Expired { .. })
                ) {
                    SafeStopReason::DeadlineExpired
                } else {
                    SafeStopReason::AuthorityRejected
                };
                self.safe_stop(reason, Some(command), now_monotonic_ns)
            }
        }
    }

    fn accept_command_order(&self, command: CommandEnvelope) -> bool {
        let header = command.header();
        if header.sequence() == 0 {
            return false;
        }
        self.last_command_order.is_none_or(|last| {
            last.boot_id != header.boot_id() || header.sequence() > last.sequence
        })
    }

    fn safe_stop(
        &mut self,
        reason: SafeStopReason,
        command: Option<CommandEnvelope>,
        now_monotonic_ns: u64,
    ) -> MotionDecision {
        self.diagnostics.safe_stops = self.diagnostics.safe_stops.saturating_add(1);
        self.diagnostics.lifecycle = LifecycleState::Faulted;
        self.diagnostics.safe_stop_reason = Some(reason);
        self.diagnostics.fault = Some(Fault {
            code: reason.fault_code(),
            severity: FaultSeverity::Critical,
            latched: true,
            recoverability: FaultRecoverability::Operator,
            first_seen_monotonic_ns: now_monotonic_ns,
            last_seen_monotonic_ns: now_monotonic_ns,
            count: 1,
            fault_source: self.component_id,
            related_command_id: command.map(CommandEnvelope::command_id),
            recommended_action: reason.fault_code(),
        });
        MotionDecision::SafeStop {
            reason,
            tick_monotonic_ns: now_monotonic_ns,
        }
    }
}
