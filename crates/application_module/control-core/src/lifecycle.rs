//! Deterministic robot lifecycle and restart-safety reducer.

use std::fmt::Write as _;

use arrayvec::ArrayVec;
use message_model::Id128;

/// Maximum ordered effects emitted by one lifecycle event.
pub const MAX_LIFECYCLE_EFFECTS: usize = 4;

/// Stable set of production lifecycle leaf states.
pub const ALL_LIFECYCLE_STATES: [LifecycleState; 8] = [
    LifecycleState::Boot,
    LifecycleState::Standby,
    LifecycleState::Armed,
    LifecycleState::Running,
    LifecycleState::Degraded,
    LifecycleState::Fault,
    LifecycleState::EStop,
    LifecycleState::Shutdown,
];

/// Stable set of semantic event kinds covered by diagnostics and tests.
pub const ALL_LIFECYCLE_EVENT_KINDS: [LifecycleEventKind; 18] = [
    LifecycleEventKind::ReconstructionProgress,
    LifecycleEventKind::ReconstructionReady,
    LifecycleEventKind::ReconstructionMissing,
    LifecycleEventKind::ReconstructionConflicting,
    LifecycleEventKind::ReconstructionTimedOut,
    LifecycleEventKind::ReconstructionFaulted,
    LifecycleEventKind::Arm,
    LifecycleEventKind::Start,
    LifecycleEventKind::Stop,
    LifecycleEventKind::Disarm,
    LifecycleEventKind::CapabilityLost,
    LifecycleEventKind::CapabilityRestored,
    LifecycleEventKind::FaultDetected,
    LifecycleEventKind::ClearFault,
    LifecycleEventKind::EmergencyStopAsserted,
    LifecycleEventKind::EmergencyStopReleased,
    LifecycleEventKind::ResetEmergencyStop,
    LifecycleEventKind::ShutdownRequested,
];

/// Robot-level lifecycle state.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum LifecycleState {
    /// Reconstructing current component health while motion is inhibited.
    Boot,
    /// Reconstruction succeeded; no motion authority exists.
    Standby,
    /// A fresh current-boot arm request was accepted.
    Armed,
    /// A mission may execute under separate resource leases.
    Running,
    /// A running capability was lost and mission work was cancelled.
    Degraded,
    /// A typed fault or unsafe reconstruction prevents arming.
    Fault,
    /// Emergency stop is latched.
    EStop,
    /// Shutdown completed; no further transition is accepted.
    Shutdown,
}

impl LifecycleState {
    /// Stable diagnostic spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Boot => "Boot",
            Self::Standby => "Standby",
            Self::Armed => "Armed",
            Self::Running => "Running",
            Self::Degraded => "Degraded",
            Self::Fault => "Fault",
            Self::EStop => "EStop",
            Self::Shutdown => "Shutdown",
        }
    }

    const fn requires_motion_inhibit(self) -> bool {
        matches!(
            self,
            Self::Boot
                | Self::Standby
                | Self::Degraded
                | Self::Fault
                | Self::EStop
                | Self::Shutdown
        )
    }
}

/// Terminal unsafe outcome of live-health reconstruction.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReconstructionFailure {
    /// One or more required components were absent.
    Missing,
    /// Live observations disagreed about required component health.
    Conflicting,
    /// The bounded reconstruction deadline elapsed.
    TimedOut,
    /// A required component reported faulted health.
    Faulted,
}

impl ReconstructionFailure {
    const fn event_kind(self) -> LifecycleEventKind {
        match self {
            Self::Missing => LifecycleEventKind::ReconstructionMissing,
            Self::Conflicting => LifecycleEventKind::ReconstructionConflicting,
            Self::TimedOut => LifecycleEventKind::ReconstructionTimedOut,
            Self::Faulted => LifecycleEventKind::ReconstructionFaulted,
        }
    }
}

/// Serialized semantic input to the lifecycle reducer.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleEvent {
    /// A newer current-boot live-health observation was incorporated.
    ReconstructionProgress {
        /// FSM process identity used to issue the reconstruction request.
        boot_id: Id128,
        /// Strictly increasing live-reconstruction revision.
        revision: u64,
    },
    /// Current-boot live health is complete and safe for standby.
    ReconstructionReady {
        /// FSM process identity used to issue the reconstruction request.
        boot_id: Id128,
        /// Strictly increasing live-reconstruction revision.
        revision: u64,
    },
    /// Live reconstruction completed with an unsafe result.
    ReconstructionFailed {
        /// FSM process identity used to issue the reconstruction request.
        boot_id: Id128,
        /// Strictly increasing live-reconstruction revision.
        revision: u64,
        /// Why reconstruction cannot authorize standby.
        reason: ReconstructionFailure,
    },
    /// Explicit operator arm request for this process start.
    Arm {
        /// Current FSM process identity.
        boot_id: Id128,
        /// Strictly increasing operator-command sequence.
        command_sequence: u64,
    },
    /// Explicit mission start tied to the current arm.
    Start {
        /// Current FSM process identity.
        boot_id: Id128,
        /// Exact sequence of the accepted Arm request.
        arm_sequence: u64,
        /// Strictly increasing operator-command sequence.
        command_sequence: u64,
    },
    /// Stop mission execution but retain current arm authority.
    Stop,
    /// Return to safe standby and discard current arm authority.
    Disarm,
    /// A capability required by the running mission was lost.
    CapabilityLost,
    /// The degraded capability recovered; mission remains cancelled.
    CapabilityRestored,
    /// A typed lifecycle fault was observed.
    FaultDetected,
    /// Explicitly clear a resolved fault for this process start.
    ClearFault {
        /// Current FSM process identity.
        boot_id: Id128,
        /// Exact reducer-owned fault generation being cleared.
        fault_generation: u64,
        /// Strictly increasing operator-command sequence.
        command_sequence: u64,
    },
    /// Physical or software emergency-stop assertion.
    EmergencyStopAsserted,
    /// Observation that the current emergency-stop assertion was released.
    EmergencyStopReleased {
        /// Current FSM process identity.
        boot_id: Id128,
        /// Exact reducer-owned E-stop assertion generation being released.
        assertion_generation: u64,
    },
    /// Explicit software reset after release observation.
    ResetEmergencyStop {
        /// Current FSM process identity.
        boot_id: Id128,
        /// Exact reducer-owned E-stop assertion generation being reset.
        assertion_generation: u64,
        /// Strictly increasing operator-command sequence.
        command_sequence: u64,
    },
    /// Complete process shutdown was requested.
    ShutdownRequested,
}

impl LifecycleEvent {
    /// Returns the stable payload-independent event classification.
    #[must_use]
    pub const fn kind(self) -> LifecycleEventKind {
        match self {
            Self::ReconstructionProgress { .. } => LifecycleEventKind::ReconstructionProgress,
            Self::ReconstructionReady { .. } => LifecycleEventKind::ReconstructionReady,
            Self::ReconstructionFailed { reason, .. } => reason.event_kind(),
            Self::Arm { .. } => LifecycleEventKind::Arm,
            Self::Start { .. } => LifecycleEventKind::Start,
            Self::Stop => LifecycleEventKind::Stop,
            Self::Disarm => LifecycleEventKind::Disarm,
            Self::CapabilityLost => LifecycleEventKind::CapabilityLost,
            Self::CapabilityRestored => LifecycleEventKind::CapabilityRestored,
            Self::FaultDetected => LifecycleEventKind::FaultDetected,
            Self::ClearFault { .. } => LifecycleEventKind::ClearFault,
            Self::EmergencyStopAsserted => LifecycleEventKind::EmergencyStopAsserted,
            Self::EmergencyStopReleased { .. } => LifecycleEventKind::EmergencyStopReleased,
            Self::ResetEmergencyStop { .. } => LifecycleEventKind::ResetEmergencyStop,
            Self::ShutdownRequested => LifecycleEventKind::ShutdownRequested,
        }
    }
}

/// Payload-independent lifecycle event classification.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum LifecycleEventKind {
    /// New live-health evidence was incorporated while remaining inhibited.
    ReconstructionProgress,
    /// Successful reconstruction.
    ReconstructionReady,
    /// Required component missing.
    ReconstructionMissing,
    /// Conflicting component health.
    ReconstructionConflicting,
    /// Reconstruction deadline elapsed.
    ReconstructionTimedOut,
    /// Required component faulted.
    ReconstructionFaulted,
    /// Arm request.
    Arm,
    /// Mission start request.
    Start,
    /// Mission stop request.
    Stop,
    /// Disarm request.
    Disarm,
    /// Running capability lost.
    CapabilityLost,
    /// Degraded capability restored.
    CapabilityRestored,
    /// Fault observation.
    FaultDetected,
    /// Fault clear request.
    ClearFault,
    /// E-stop assertion.
    EmergencyStopAsserted,
    /// E-stop release observation.
    EmergencyStopReleased,
    /// E-stop software reset.
    ResetEmergencyStop,
    /// Shutdown request.
    ShutdownRequested,
}

impl LifecycleEventKind {
    /// Stable diagnostic spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ReconstructionProgress => "ReconstructionProgress",
            Self::ReconstructionReady => "ReconstructionReady",
            Self::ReconstructionMissing => "ReconstructionMissing",
            Self::ReconstructionConflicting => "ReconstructionConflicting",
            Self::ReconstructionTimedOut => "ReconstructionTimedOut",
            Self::ReconstructionFaulted => "ReconstructionFaulted",
            Self::Arm => "Arm",
            Self::Start => "Start",
            Self::Stop => "Stop",
            Self::Disarm => "Disarm",
            Self::CapabilityLost => "CapabilityLost",
            Self::CapabilityRestored => "CapabilityRestored",
            Self::FaultDetected => "FaultDetected",
            Self::ClearFault => "ClearFault",
            Self::EmergencyStopAsserted => "EmergencyStopAsserted",
            Self::EmergencyStopReleased => "EmergencyStopReleased",
            Self::ResetEmergencyStop => "ResetEmergencyStop",
            Self::ShutdownRequested => "ShutdownRequested",
        }
    }
}

/// Reducer-owned E-stop latch condition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EmergencyStopLatch {
    /// No E-stop is latched.
    Clear,
    /// E-stop remains asserted or release has not been observed.
    Asserted,
    /// Release was observed; an explicit software reset is still required.
    ReleaseObserved,
}

/// Explicit reason an event did not change lifecycle authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleRejection {
    /// This event has no legal meaning in the current state.
    InvalidInState,
    /// The event belongs to a different process start.
    StaleBoot,
    /// A sequence was zero, duplicated, or older than the accepted sequence.
    StaleSequence {
        /// Highest accepted sequence.
        current: u64,
        /// Rejected sequence.
        proposed: u64,
    },
    /// Mission start was attempted without current arm authority.
    ArmRequired,
    /// Mission start did not reference the current arm sequence.
    ArmSequenceMismatch {
        /// Current accepted Arm sequence.
        expected: u64,
        /// Sequence supplied by Start.
        proposed: u64,
    },
    /// Software E-stop reset arrived before release observation.
    EmergencyStopReleaseRequired,
    /// E-stop release or reset refers to an older/newer assertion.
    EmergencyStopAssertionMismatch {
        /// Current reducer-owned assertion generation.
        expected: u64,
        /// Generation supplied by the event.
        proposed: u64,
    },
    /// Fault clear refers to an older/newer fault occurrence.
    FaultGenerationMismatch {
        /// Current reducer-owned fault generation.
        expected: u64,
        /// Generation supplied by the event.
        proposed: u64,
    },
    /// Completed shutdown is immutable.
    ShutdownComplete,
}

/// Ordered lifecycle side effect.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleEffect {
    /// Prevent the motion owner from accepting output.
    MotionInhibited,
    /// A newer live-health reconstruction observation was accepted.
    ReconstructionProgressAccepted,
    /// Live reconstruction was accepted for this process start.
    ReconstructionAccepted,
    /// Unsafe live reconstruction entered Fault.
    ReconstructionFailed {
        /// Unsafe reconstruction outcome.
        reason: ReconstructionFailure,
        /// Reducer-owned fault generation created by the outcome.
        fault_generation: u64,
    },
    /// A fresh arm request was accepted.
    ArmAccepted {
        /// Accepted operator-command sequence.
        command_sequence: u64,
    },
    /// A mission start request was accepted.
    MissionStarted,
    /// Cancel all mission work synchronously.
    CancelMission,
    /// Release all mission resources after cancellation.
    ReleaseResources,
    /// Capability loss cancelled the running mission.
    CapabilityDegraded,
    /// A fault was latched.
    FaultLatched {
        /// Reducer-owned fault generation.
        fault_generation: u64,
    },
    /// A resolved fault was explicitly cleared.
    FaultCleared,
    /// Emergency stop was latched.
    EmergencyStopLatched {
        /// Reducer-owned assertion generation.
        assertion_generation: u64,
    },
    /// Emergency-stop release was observed; software reset remains required.
    EmergencyStopReleaseObserved,
    /// Emergency stop was explicitly reset.
    EmergencyStopReset,
    /// Shutdown completed.
    ShutdownCompleted,
    /// The event was explicitly rejected.
    Rejected(LifecycleRejection),
}

/// Fixed-capacity ordered effects from one reducer step.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LifecycleEffects {
    items: ArrayVec<LifecycleEffect, MAX_LIFECYCLE_EFFECTS>,
}

impl Default for LifecycleEffects {
    fn default() -> Self {
        Self::new()
    }
}

impl LifecycleEffects {
    const fn new() -> Self {
        Self {
            items: ArrayVec::new_const(),
        }
    }

    fn push(&mut self, effect: LifecycleEffect) {
        self.items
            .try_push(effect)
            .expect("lifecycle effect capacity is statically proven");
    }

    /// Returns effects in their required execution order.
    #[must_use]
    pub fn as_slice(&self) -> &[LifecycleEffect] {
        self.items.as_slice()
    }
}

/// Observable result of construction or one event.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LifecycleProjection {
    state: LifecycleState,
    emergency_stop_latch: EmergencyStopLatch,
    effects: LifecycleEffects,
}

impl LifecycleProjection {
    /// Returns the leaf state after the operation.
    #[must_use]
    pub const fn state(&self) -> LifecycleState {
        self.state
    }

    /// Returns the E-stop latch context after the operation.
    #[must_use]
    pub const fn emergency_stop_latch(&self) -> EmergencyStopLatch {
        self.emergency_stop_latch
    }

    /// Returns the synchronous effects in execution order.
    #[must_use]
    pub const fn effects(&self) -> &LifecycleEffects {
        &self.effects
    }
}

/// Single-owner production lifecycle reducer.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LifecycleMachine {
    boot_id: Id128,
    state: LifecycleState,
    emergency_stop_latch: EmergencyStopLatch,
    reconstruction_revision: u64,
    highest_operator_sequence: u64,
    current_arm_sequence: Option<u64>,
    fault_generation: u64,
    emergency_stop_assertion_generation: u64,
    emergency_stop_reset_target: LifecycleState,
}

impl LifecycleMachine {
    /// Creates a fresh process lifecycle in motion-inhibited `Boot`.
    ///
    /// No constructor accepts a persisted state or mission checkpoint.
    #[must_use]
    pub const fn new(boot_id: Id128) -> Self {
        Self {
            boot_id,
            state: LifecycleState::Boot,
            emergency_stop_latch: EmergencyStopLatch::Clear,
            reconstruction_revision: 0,
            highest_operator_sequence: 0,
            current_arm_sequence: None,
            fault_generation: 0,
            emergency_stop_assertion_generation: 0,
            emergency_stop_reset_target: LifecycleState::Boot,
        }
    }

    /// Returns construction-time safety evidence.
    #[must_use]
    pub fn startup_projection(&self) -> LifecycleProjection {
        let mut effects = LifecycleEffects::new();
        effects.push(LifecycleEffect::MotionInhibited);
        self.projection(effects)
    }

    /// Returns this process-start identity.
    #[must_use]
    pub const fn boot_id(&self) -> Id128 {
        self.boot_id
    }

    /// Returns the current leaf state.
    #[must_use]
    pub const fn state(&self) -> LifecycleState {
        self.state
    }

    /// Returns the current E-stop latch context.
    #[must_use]
    pub const fn emergency_stop_latch(&self) -> EmergencyStopLatch {
        self.emergency_stop_latch
    }

    /// Returns the accepted arm sequence, if arm authority still exists.
    #[must_use]
    pub const fn current_arm_sequence(&self) -> Option<u64> {
        self.current_arm_sequence
    }

    /// Returns the reducer-owned generation of the latest fault occurrence.
    #[must_use]
    pub const fn fault_generation(&self) -> u64 {
        self.fault_generation
    }

    /// Returns the reducer-owned generation of the latest E-stop assertion.
    #[must_use]
    pub const fn emergency_stop_assertion_generation(&self) -> u64 {
        self.emergency_stop_assertion_generation
    }

    /// Applies one already-serialized semantic event.
    pub fn dispatch(&mut self, event: LifecycleEvent) -> LifecycleProjection {
        match event {
            LifecycleEvent::ReconstructionProgress { boot_id, revision } => match self.state {
                LifecycleState::Boot => {
                    if let Err(rejection) = self.accept_reconstruction_revision(boot_id, revision) {
                        return self.reject(rejection);
                    }
                    self.same_state(Some(LifecycleEffect::ReconstructionProgressAccepted))
                }
                LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::ReconstructionReady { boot_id, revision } => match self.state {
                LifecycleState::Boot => self.accept_reconstruction(boot_id, revision, None),
                LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::ReconstructionFailed {
                boot_id,
                revision,
                reason,
            } => match self.state {
                LifecycleState::Boot => self.accept_reconstruction(boot_id, revision, Some(reason)),
                LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::Arm {
                boot_id,
                command_sequence,
            } => match self.state {
                LifecycleState::Standby => {
                    if let Err(rejection) = self.accept_operator_sequence(boot_id, command_sequence)
                    {
                        return self.reject(rejection);
                    }
                    self.current_arm_sequence = Some(command_sequence);
                    self.transition(
                        LifecycleState::Armed,
                        Some(LifecycleEffect::ArmAccepted { command_sequence }),
                    )
                }
                LifecycleState::Boot
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::Start {
                boot_id,
                arm_sequence,
                command_sequence,
            } => match self.state {
                LifecycleState::Armed => {
                    if boot_id != self.boot_id {
                        return self.reject(LifecycleRejection::StaleBoot);
                    }
                    let Some(expected) = self.current_arm_sequence else {
                        return self.reject(LifecycleRejection::ArmRequired);
                    };
                    if arm_sequence != expected {
                        return self.reject(LifecycleRejection::ArmSequenceMismatch {
                            expected,
                            proposed: arm_sequence,
                        });
                    }
                    if let Err(rejection) = self.accept_operator_sequence(boot_id, command_sequence)
                    {
                        return self.reject(rejection);
                    }
                    self.transition(
                        LifecycleState::Running,
                        Some(LifecycleEffect::MissionStarted),
                    )
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::Stop => match self.state {
                LifecycleState::Running | LifecycleState::Degraded => {
                    self.transition(LifecycleState::Armed, None)
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::Disarm => match self.state {
                LifecycleState::Armed | LifecycleState::Running | LifecycleState::Degraded => {
                    self.transition(LifecycleState::Standby, None)
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::CapabilityLost => match self.state {
                LifecycleState::Running => self.transition(
                    LifecycleState::Degraded,
                    Some(LifecycleEffect::CapabilityDegraded),
                ),
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::CapabilityRestored => match self.state {
                LifecycleState::Degraded => self.transition(LifecycleState::Armed, None),
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Fault
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::FaultDetected => match self.state {
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded => {
                    let effect = self.next_fault_effect();
                    self.transition(LifecycleState::Fault, Some(effect))
                }
                LifecycleState::Fault => {
                    let effect = self.next_fault_effect();
                    self.same_state(Some(effect))
                }
                LifecycleState::EStop => {
                    let effect = self.next_fault_effect();
                    self.emergency_stop_reset_target = LifecycleState::Fault;
                    self.same_state(Some(effect))
                }
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::ClearFault {
                boot_id,
                fault_generation,
                command_sequence,
            } => match self.state {
                LifecycleState::Fault => {
                    if fault_generation != self.fault_generation {
                        return self.reject(LifecycleRejection::FaultGenerationMismatch {
                            expected: self.fault_generation,
                            proposed: fault_generation,
                        });
                    }
                    if let Err(rejection) = self.accept_operator_sequence(boot_id, command_sequence)
                    {
                        return self.reject(rejection);
                    }
                    self.transition(LifecycleState::Standby, Some(LifecycleEffect::FaultCleared))
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::EStop => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::EmergencyStopAsserted => match self.state {
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => {
                    if self.state != LifecycleState::EStop {
                        self.emergency_stop_reset_target = match self.state {
                            LifecycleState::Boot => LifecycleState::Boot,
                            LifecycleState::Fault => LifecycleState::Fault,
                            LifecycleState::Standby
                            | LifecycleState::Armed
                            | LifecycleState::Running
                            | LifecycleState::Degraded => LifecycleState::Standby,
                            LifecycleState::EStop | LifecycleState::Shutdown => {
                                LifecycleState::Standby
                            }
                        };
                    }
                    self.emergency_stop_assertion_generation =
                        self.emergency_stop_assertion_generation.saturating_add(1);
                    self.emergency_stop_latch = EmergencyStopLatch::Asserted;
                    self.transition(
                        LifecycleState::EStop,
                        Some(LifecycleEffect::EmergencyStopLatched {
                            assertion_generation: self.emergency_stop_assertion_generation,
                        }),
                    )
                }
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::EmergencyStopReleased {
                boot_id,
                assertion_generation,
            } => match self.state {
                LifecycleState::EStop => {
                    if boot_id != self.boot_id {
                        return self.reject(LifecycleRejection::StaleBoot);
                    }
                    if assertion_generation != self.emergency_stop_assertion_generation {
                        return self.reject(LifecycleRejection::EmergencyStopAssertionMismatch {
                            expected: self.emergency_stop_assertion_generation,
                            proposed: assertion_generation,
                        });
                    }
                    if self.emergency_stop_latch == EmergencyStopLatch::ReleaseObserved {
                        return self.reject(LifecycleRejection::InvalidInState);
                    }
                    self.emergency_stop_latch = EmergencyStopLatch::ReleaseObserved;
                    self.same_state(Some(LifecycleEffect::EmergencyStopReleaseObserved))
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::ResetEmergencyStop {
                boot_id,
                assertion_generation,
                command_sequence,
            } => match self.state {
                LifecycleState::EStop => {
                    if boot_id != self.boot_id {
                        return self.reject(LifecycleRejection::StaleBoot);
                    }
                    if assertion_generation != self.emergency_stop_assertion_generation {
                        return self.reject(LifecycleRejection::EmergencyStopAssertionMismatch {
                            expected: self.emergency_stop_assertion_generation,
                            proposed: assertion_generation,
                        });
                    }
                    if self.emergency_stop_latch != EmergencyStopLatch::ReleaseObserved {
                        return self.reject(LifecycleRejection::EmergencyStopReleaseRequired);
                    }
                    if let Err(rejection) = self.accept_operator_sequence(boot_id, command_sequence)
                    {
                        return self.reject(rejection);
                    }
                    self.transition(
                        self.emergency_stop_reset_target,
                        Some(LifecycleEffect::EmergencyStopReset),
                    )
                }
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault => self.reject(LifecycleRejection::InvalidInState),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
            LifecycleEvent::ShutdownRequested => match self.state {
                LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Armed
                | LifecycleState::Running
                | LifecycleState::Degraded
                | LifecycleState::Fault
                | LifecycleState::EStop => self.transition(
                    LifecycleState::Shutdown,
                    Some(LifecycleEffect::ShutdownCompleted),
                ),
                LifecycleState::Shutdown => self.reject(LifecycleRejection::ShutdownComplete),
            },
        }
    }

    fn accept_reconstruction(
        &mut self,
        boot_id: Id128,
        revision: u64,
        failure: Option<ReconstructionFailure>,
    ) -> LifecycleProjection {
        if let Err(rejection) = self.accept_reconstruction_revision(boot_id, revision) {
            return self.reject(rejection);
        }
        match failure {
            None => self.transition(
                LifecycleState::Standby,
                Some(LifecycleEffect::ReconstructionAccepted),
            ),
            Some(reason) => {
                self.fault_generation = self.fault_generation.saturating_add(1);
                self.transition(
                    LifecycleState::Fault,
                    Some(LifecycleEffect::ReconstructionFailed {
                        reason,
                        fault_generation: self.fault_generation,
                    }),
                )
            }
        }
    }

    fn accept_reconstruction_revision(
        &mut self,
        boot_id: Id128,
        revision: u64,
    ) -> Result<(), LifecycleRejection> {
        if boot_id != self.boot_id {
            return Err(LifecycleRejection::StaleBoot);
        }
        if revision == 0 || revision <= self.reconstruction_revision {
            return Err(LifecycleRejection::StaleSequence {
                current: self.reconstruction_revision,
                proposed: revision,
            });
        }
        self.reconstruction_revision = revision;
        Ok(())
    }

    fn accept_operator_sequence(
        &mut self,
        boot_id: Id128,
        command_sequence: u64,
    ) -> Result<(), LifecycleRejection> {
        if boot_id != self.boot_id {
            return Err(LifecycleRejection::StaleBoot);
        }
        if command_sequence == 0 || command_sequence <= self.highest_operator_sequence {
            return Err(LifecycleRejection::StaleSequence {
                current: self.highest_operator_sequence,
                proposed: command_sequence,
            });
        }
        self.highest_operator_sequence = command_sequence;
        Ok(())
    }

    fn next_fault_effect(&mut self) -> LifecycleEffect {
        self.fault_generation = self.fault_generation.saturating_add(1);
        LifecycleEffect::FaultLatched {
            fault_generation: self.fault_generation,
        }
    }

    fn transition(
        &mut self,
        target: LifecycleState,
        primary: Option<LifecycleEffect>,
    ) -> LifecycleProjection {
        let source = self.state;
        let mut effects = LifecycleEffects::new();
        if source == LifecycleState::Running && target != LifecycleState::Running {
            effects.push(LifecycleEffect::CancelMission);
            effects.push(LifecycleEffect::ReleaseResources);
        }
        if let Some(effect) = primary {
            effects.push(effect);
        }

        self.state = target;
        if target != LifecycleState::EStop {
            self.emergency_stop_latch = EmergencyStopLatch::Clear;
        }
        if matches!(
            target,
            LifecycleState::Boot
                | LifecycleState::Standby
                | LifecycleState::Fault
                | LifecycleState::EStop
                | LifecycleState::Shutdown
        ) {
            self.current_arm_sequence = None;
        }
        if target.requires_motion_inhibit() {
            effects.push(LifecycleEffect::MotionInhibited);
        }
        self.projection(effects)
    }

    fn same_state(&self, primary: Option<LifecycleEffect>) -> LifecycleProjection {
        let mut effects = LifecycleEffects::new();
        if let Some(effect) = primary {
            effects.push(effect);
        }
        if self.state.requires_motion_inhibit() {
            effects.push(LifecycleEffect::MotionInhibited);
        }
        self.projection(effects)
    }

    fn reject(&self, rejection: LifecycleRejection) -> LifecycleProjection {
        self.same_state(Some(LifecycleEffect::Rejected(rejection)))
    }

    fn projection(&self, effects: LifecycleEffects) -> LifecycleProjection {
        LifecycleProjection {
            state: self.state,
            emergency_stop_latch: self.emergency_stop_latch,
            effects,
        }
    }
}

/// Generates a complete, stable state/event transition report through the
/// production reducer.
#[must_use]
pub fn lifecycle_transition_report() -> String {
    let mut report = String::new();
    for state in ALL_LIFECYCLE_STATES {
        for kind in ALL_LIFECYCLE_EVENT_KINDS {
            let mut machine = diagnostic_machine(state);
            let source_latch = machine.emergency_stop_latch();
            let event = canonical_event(kind, &machine);
            let projection = machine.dispatch(event);
            writeln!(
                report,
                "{}({:?}) + {} -> {}({:?}) {:?}",
                state.as_str(),
                source_latch,
                kind.as_str(),
                projection.state().as_str(),
                projection.emergency_stop_latch(),
                projection.effects().as_slice()
            )
            .expect("writing to String cannot fail");
        }
    }
    report
}

/// Generates a deterministic DOT graph from production reducer projections.
#[must_use]
pub fn lifecycle_dot_graph() -> String {
    let mut dot = String::from("digraph lifecycle {\n");
    for state in ALL_LIFECYCLE_STATES {
        writeln!(dot, "  {};", state.as_str()).expect("writing to String cannot fail");
    }
    for state in ALL_LIFECYCLE_STATES {
        for kind in ALL_LIFECYCLE_EVENT_KINDS {
            let mut machine = diagnostic_machine(state);
            let projection = machine.dispatch(canonical_event(kind, &machine));
            if projection.state() != state {
                writeln!(
                    dot,
                    "  {} -> {} [label=\"{}\"];",
                    state.as_str(),
                    projection.state().as_str(),
                    kind.as_str()
                )
                .expect("writing to String cannot fail");
            }
        }
    }
    dot.push_str("}\n");
    dot
}

fn canonical_boot_id() -> Id128 {
    Id128::new([0xA5; 16]).expect("canonical boot identity is nonzero")
}

fn diagnostic_machine(target: LifecycleState) -> LifecycleMachine {
    let boot_id = canonical_boot_id();
    let mut machine = LifecycleMachine::new(boot_id);
    match target {
        LifecycleState::Boot => {}
        LifecycleState::Standby
        | LifecycleState::Armed
        | LifecycleState::Running
        | LifecycleState::Degraded => {
            let _ = machine.dispatch(LifecycleEvent::ReconstructionReady {
                boot_id,
                revision: 1,
            });
        }
        LifecycleState::Fault => {
            let _ = machine.dispatch(LifecycleEvent::ReconstructionFailed {
                boot_id,
                revision: 1,
                reason: ReconstructionFailure::Faulted,
            });
        }
        LifecycleState::EStop => {
            let _ = machine.dispatch(LifecycleEvent::ReconstructionReady {
                boot_id,
                revision: 1,
            });
            let _ = machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
        }
        LifecycleState::Shutdown => {
            let _ = machine.dispatch(LifecycleEvent::ShutdownRequested);
        }
    }
    match target {
        LifecycleState::Armed | LifecycleState::Running | LifecycleState::Degraded => {
            let _ = machine.dispatch(LifecycleEvent::Arm {
                boot_id,
                command_sequence: 1,
            });
        }
        LifecycleState::Boot
        | LifecycleState::Standby
        | LifecycleState::Fault
        | LifecycleState::EStop
        | LifecycleState::Shutdown => {}
    }
    match target {
        LifecycleState::Running | LifecycleState::Degraded => {
            let _ = machine.dispatch(LifecycleEvent::Start {
                boot_id,
                arm_sequence: 1,
                command_sequence: 2,
            });
        }
        LifecycleState::Boot
        | LifecycleState::Standby
        | LifecycleState::Armed
        | LifecycleState::Fault
        | LifecycleState::EStop
        | LifecycleState::Shutdown => {}
    }
    if target == LifecycleState::Degraded {
        let _ = machine.dispatch(LifecycleEvent::CapabilityLost);
    }
    if target == LifecycleState::EStop {
        let _ = machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: machine.emergency_stop_assertion_generation(),
        });
    }
    debug_assert_eq!(machine.state(), target);
    machine
}

fn canonical_event(kind: LifecycleEventKind, machine: &LifecycleMachine) -> LifecycleEvent {
    let boot_id = machine.boot_id();
    let reconstruction_revision = machine.reconstruction_revision.saturating_add(1);
    let command_sequence = machine.highest_operator_sequence.saturating_add(1);
    match kind {
        LifecycleEventKind::ReconstructionProgress => LifecycleEvent::ReconstructionProgress {
            boot_id,
            revision: reconstruction_revision,
        },
        LifecycleEventKind::ReconstructionReady => LifecycleEvent::ReconstructionReady {
            boot_id,
            revision: reconstruction_revision,
        },
        LifecycleEventKind::ReconstructionMissing => LifecycleEvent::ReconstructionFailed {
            boot_id,
            revision: reconstruction_revision,
            reason: ReconstructionFailure::Missing,
        },
        LifecycleEventKind::ReconstructionConflicting => LifecycleEvent::ReconstructionFailed {
            boot_id,
            revision: reconstruction_revision,
            reason: ReconstructionFailure::Conflicting,
        },
        LifecycleEventKind::ReconstructionTimedOut => LifecycleEvent::ReconstructionFailed {
            boot_id,
            revision: reconstruction_revision,
            reason: ReconstructionFailure::TimedOut,
        },
        LifecycleEventKind::ReconstructionFaulted => LifecycleEvent::ReconstructionFailed {
            boot_id,
            revision: reconstruction_revision,
            reason: ReconstructionFailure::Faulted,
        },
        LifecycleEventKind::Arm => LifecycleEvent::Arm {
            boot_id,
            command_sequence,
        },
        LifecycleEventKind::Start => LifecycleEvent::Start {
            boot_id,
            arm_sequence: machine.current_arm_sequence().unwrap_or(1),
            command_sequence,
        },
        LifecycleEventKind::Stop => LifecycleEvent::Stop,
        LifecycleEventKind::Disarm => LifecycleEvent::Disarm,
        LifecycleEventKind::CapabilityLost => LifecycleEvent::CapabilityLost,
        LifecycleEventKind::CapabilityRestored => LifecycleEvent::CapabilityRestored,
        LifecycleEventKind::FaultDetected => LifecycleEvent::FaultDetected,
        LifecycleEventKind::ClearFault => LifecycleEvent::ClearFault {
            boot_id,
            fault_generation: machine.fault_generation(),
            command_sequence,
        },
        LifecycleEventKind::EmergencyStopAsserted => LifecycleEvent::EmergencyStopAsserted,
        LifecycleEventKind::EmergencyStopReleased => LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: machine.emergency_stop_assertion_generation(),
        },
        LifecycleEventKind::ResetEmergencyStop => LifecycleEvent::ResetEmergencyStop {
            boot_id,
            assertion_generation: machine.emergency_stop_assertion_generation(),
            command_sequence,
        },
        LifecycleEventKind::ShutdownRequested => LifecycleEvent::ShutdownRequested,
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;

    use super::*;

    fn boot(value: u8) -> Id128 {
        Id128::new([value; 16]).unwrap()
    }

    fn ready(machine: &mut LifecycleMachine, revision: u64) -> LifecycleProjection {
        machine.dispatch(LifecycleEvent::ReconstructionReady {
            boot_id: machine.boot_id(),
            revision,
        })
    }

    fn arm(machine: &mut LifecycleMachine, sequence: u64) -> LifecycleProjection {
        machine.dispatch(LifecycleEvent::Arm {
            boot_id: machine.boot_id(),
            command_sequence: sequence,
        })
    }

    fn start(
        machine: &mut LifecycleMachine,
        arm_sequence: u64,
        command_sequence: u64,
    ) -> LifecycleProjection {
        machine.dispatch(LifecycleEvent::Start {
            boot_id: machine.boot_id(),
            arm_sequence,
            command_sequence,
        })
    }

    #[test]
    fn transition_report_covers_every_state_event_pair() {
        let report = lifecycle_transition_report();
        assert_eq!(
            report.lines().count(),
            ALL_LIFECYCLE_STATES.len() * ALL_LIFECYCLE_EVENT_KINDS.len()
        );
        for state in ALL_LIFECYCLE_STATES {
            for kind in ALL_LIFECYCLE_EVENT_KINDS {
                assert!(
                    report.contains(&format!(
                        "{}({:?}) + {}",
                        state.as_str(),
                        if state == LifecycleState::EStop {
                            EmergencyStopLatch::ReleaseObserved
                        } else {
                            EmergencyStopLatch::Clear
                        },
                        kind.as_str()
                    )),
                    "missing {} + {}",
                    state.as_str(),
                    kind.as_str()
                );
            }
        }
    }

    #[test]
    fn startup_and_all_reconstruction_failures_remain_inhibited() {
        let boot_id = boot(1);
        let machine = LifecycleMachine::new(boot_id);
        assert_eq!(machine.state(), LifecycleState::Boot);
        assert_eq!(
            machine.startup_projection().effects().as_slice(),
            &[LifecycleEffect::MotionInhibited]
        );

        let mut ready_machine = LifecycleMachine::new(boot_id);
        let projection = ready(&mut ready_machine, 1);
        assert_eq!(projection.state(), LifecycleState::Standby);
        assert_eq!(
            projection.effects().as_slice(),
            &[
                LifecycleEffect::ReconstructionAccepted,
                LifecycleEffect::MotionInhibited
            ]
        );

        for reason in [
            ReconstructionFailure::Missing,
            ReconstructionFailure::Conflicting,
            ReconstructionFailure::TimedOut,
            ReconstructionFailure::Faulted,
        ] {
            let mut machine = LifecycleMachine::new(boot_id);
            let projection = machine.dispatch(LifecycleEvent::ReconstructionFailed {
                boot_id,
                revision: 1,
                reason,
            });
            assert_eq!(projection.state(), LifecycleState::Fault);
            assert_eq!(
                projection.effects().as_slice(),
                &[
                    LifecycleEffect::ReconstructionFailed {
                        reason,
                        fault_generation: 1
                    },
                    LifecycleEffect::MotionInhibited
                ]
            );
        }
    }

    #[test]
    fn restart_rejects_prior_boot_authority_and_requires_fresh_arm() {
        let old_boot = boot(1);
        let new_boot = boot(2);
        let mut old = LifecycleMachine::new(old_boot);
        ready(&mut old, 1);
        arm(&mut old, 1);
        start(&mut old, 1, 2);
        assert_eq!(old.state(), LifecycleState::Running);

        let mut restarted = LifecycleMachine::new(new_boot);
        assert_eq!(restarted.state(), LifecycleState::Boot);
        let stale_reconstruction = restarted.dispatch(LifecycleEvent::ReconstructionReady {
            boot_id: old_boot,
            revision: u64::MAX,
        });
        assert_eq!(stale_reconstruction.state(), LifecycleState::Boot);
        assert_eq!(
            stale_reconstruction.effects().as_slice(),
            &[
                LifecycleEffect::Rejected(LifecycleRejection::StaleBoot),
                LifecycleEffect::MotionInhibited
            ]
        );

        ready(&mut restarted, 1);
        let stale_arm = restarted.dispatch(LifecycleEvent::Arm {
            boot_id: old_boot,
            command_sequence: u64::MAX,
        });
        assert_eq!(stale_arm.state(), LifecycleState::Standby);
        assert_eq!(restarted.current_arm_sequence(), None);
        arm(&mut restarted, 1);
        let stale_start = restarted.dispatch(LifecycleEvent::Start {
            boot_id: old_boot,
            arm_sequence: 1,
            command_sequence: u64::MAX,
        });
        assert_eq!(stale_start.state(), LifecycleState::Armed);
        let fresh_start = start(&mut restarted, 1, 2);
        assert_eq!(fresh_start.state(), LifecycleState::Running);
    }

    #[test]
    fn duplicate_reordered_loss_and_timeout_events_fail_closed() {
        let boot_id = boot(1);
        let mut no_completion = LifecycleMachine::new(boot_id);
        let lost_ready = no_completion.dispatch(LifecycleEvent::Arm {
            boot_id,
            command_sequence: 1,
        });
        assert_eq!(lost_ready.state(), LifecycleState::Boot);
        assert_eq!(
            lost_ready.effects().as_slice().last(),
            Some(&LifecycleEffect::MotionInhibited)
        );

        let mut reconstruction = LifecycleMachine::new(boot_id);
        let zero = reconstruction.dispatch(LifecycleEvent::ReconstructionProgress {
            boot_id,
            revision: 0,
        });
        assert!(matches!(
            zero.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleSequence { .. })
        ));
        let progress = reconstruction.dispatch(LifecycleEvent::ReconstructionProgress {
            boot_id,
            revision: 2,
        });
        assert_eq!(
            progress.effects().as_slice(),
            &[
                LifecycleEffect::ReconstructionProgressAccepted,
                LifecycleEffect::MotionInhibited
            ]
        );
        let duplicate_progress = reconstruction.dispatch(LifecycleEvent::ReconstructionProgress {
            boot_id,
            revision: 2,
        });
        assert!(matches!(
            duplicate_progress.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleSequence {
                current: 2,
                proposed: 2
            })
        ));
        let reordered_progress = reconstruction.dispatch(LifecycleEvent::ReconstructionProgress {
            boot_id,
            revision: 1,
        });
        assert!(matches!(
            reordered_progress.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleSequence {
                current: 2,
                proposed: 1
            })
        ));
        let timeout = reconstruction.dispatch(LifecycleEvent::ReconstructionFailed {
            boot_id,
            revision: 3,
            reason: ReconstructionFailure::TimedOut,
        });
        assert_eq!(timeout.state(), LifecycleState::Fault);

        let mut commands = LifecycleMachine::new(boot_id);
        ready(&mut commands, 1);
        arm(&mut commands, 5);
        commands.dispatch(LifecycleEvent::Disarm);
        let duplicate = commands.dispatch(LifecycleEvent::Arm {
            boot_id,
            command_sequence: 5,
        });
        assert!(matches!(
            duplicate.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleSequence {
                current: 5,
                proposed: 5
            })
        ));
        let reordered = commands.dispatch(LifecycleEvent::Arm {
            boot_id,
            command_sequence: 4,
        });
        assert!(matches!(
            reordered.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleSequence {
                current: 5,
                proposed: 4
            })
        ));
    }

    #[test]
    fn emergency_stop_is_global_latched_and_reset_is_boot_correlated() {
        for state in ALL_LIFECYCLE_STATES {
            let mut machine = diagnostic_machine(state);
            let projection = machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
            let expected = if state == LifecycleState::Shutdown {
                LifecycleState::Shutdown
            } else {
                LifecycleState::EStop
            };
            assert_eq!(projection.state(), expected);
            if state != LifecycleState::Shutdown {
                assert_eq!(
                    projection.emergency_stop_latch(),
                    EmergencyStopLatch::Asserted
                );
            }
        }

        let boot_id = boot(1);
        let mut machine = LifecycleMachine::new(boot_id);
        ready(&mut machine, 1);
        machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
        let too_early = machine.dispatch(LifecycleEvent::ResetEmergencyStop {
            boot_id,
            assertion_generation: 1,
            command_sequence: 1,
        });
        assert_eq!(too_early.state(), LifecycleState::EStop);
        assert_eq!(
            too_early.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::EmergencyStopReleaseRequired)
        );
        let stale_release = machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id: boot(2),
            assertion_generation: 1,
        });
        assert_eq!(
            stale_release.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleBoot)
        );
        let released = machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: 1,
        });
        assert_eq!(
            released.emergency_stop_latch(),
            EmergencyStopLatch::ReleaseObserved
        );
        let stale = machine.dispatch(LifecycleEvent::ResetEmergencyStop {
            boot_id: boot(2),
            assertion_generation: 1,
            command_sequence: u64::MAX,
        });
        assert_eq!(stale.state(), LifecycleState::EStop);
        machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
        let old_release = machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: 1,
        });
        assert_eq!(
            old_release.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::EmergencyStopAssertionMismatch {
                expected: 2,
                proposed: 1
            })
        );
        machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: 2,
        });
        let reset = machine.dispatch(LifecycleEvent::ResetEmergencyStop {
            boot_id,
            assertion_generation: 2,
            command_sequence: 1,
        });
        assert_eq!(reset.state(), LifecycleState::Standby);
        assert_eq!(
            reset.effects().as_slice(),
            &[
                LifecycleEffect::EmergencyStopReset,
                LifecycleEffect::MotionInhibited
            ]
        );
    }

    #[test]
    fn emergency_stop_reset_preserves_boot_and_fault_prerequisites() {
        let boot_id = boot(1);

        let mut boot_machine = LifecycleMachine::new(boot_id);
        boot_machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
        boot_machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: 1,
        });
        let boot_reset = boot_machine.dispatch(LifecycleEvent::ResetEmergencyStop {
            boot_id,
            assertion_generation: 1,
            command_sequence: 1,
        });
        assert_eq!(boot_reset.state(), LifecycleState::Boot);
        assert_eq!(
            boot_reset.effects().as_slice().last(),
            Some(&LifecycleEffect::MotionInhibited)
        );

        let mut fault_machine = LifecycleMachine::new(boot_id);
        fault_machine.dispatch(LifecycleEvent::FaultDetected);
        assert_eq!(fault_machine.fault_generation(), 1);
        fault_machine.dispatch(LifecycleEvent::EmergencyStopAsserted);
        fault_machine.dispatch(LifecycleEvent::EmergencyStopReleased {
            boot_id,
            assertion_generation: 1,
        });
        let fault_reset = fault_machine.dispatch(LifecycleEvent::ResetEmergencyStop {
            boot_id,
            assertion_generation: 1,
            command_sequence: 1,
        });
        assert_eq!(fault_reset.state(), LifecycleState::Fault);

        let stale_boot_clear = fault_machine.dispatch(LifecycleEvent::ClearFault {
            boot_id: boot(2),
            fault_generation: 1,
            command_sequence: u64::MAX,
        });
        assert_eq!(
            stale_boot_clear.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::StaleBoot)
        );
        let stale_clear = fault_machine.dispatch(LifecycleEvent::ClearFault {
            boot_id,
            fault_generation: 0,
            command_sequence: 2,
        });
        assert_eq!(
            stale_clear.effects().as_slice()[0],
            LifecycleEffect::Rejected(LifecycleRejection::FaultGenerationMismatch {
                expected: 1,
                proposed: 0
            })
        );
        fault_machine.dispatch(LifecycleEvent::FaultDetected);
        assert_eq!(fault_machine.fault_generation(), 2);
        let stale_generation = fault_machine.dispatch(LifecycleEvent::ClearFault {
            boot_id,
            fault_generation: 1,
            command_sequence: 2,
        });
        assert_eq!(stale_generation.state(), LifecycleState::Fault);
        let clear = fault_machine.dispatch(LifecycleEvent::ClearFault {
            boot_id,
            fault_generation: 2,
            command_sequence: 2,
        });
        assert_eq!(clear.state(), LifecycleState::Standby);
    }

    #[test]
    fn every_running_exit_cancels_before_release() {
        for event in [
            LifecycleEvent::Stop,
            LifecycleEvent::Disarm,
            LifecycleEvent::CapabilityLost,
            LifecycleEvent::FaultDetected,
            LifecycleEvent::EmergencyStopAsserted,
            LifecycleEvent::ShutdownRequested,
        ] {
            let mut machine = diagnostic_machine(LifecycleState::Running);
            let projection = machine.dispatch(event);
            assert_ne!(projection.state(), LifecycleState::Running);
            assert_eq!(
                &projection.effects().as_slice()[..2],
                &[
                    LifecycleEffect::CancelMission,
                    LifecycleEffect::ReleaseResources
                ]
            );
            if projection.state().requires_motion_inhibit() {
                assert_eq!(
                    projection.effects().as_slice().last(),
                    Some(&LifecycleEffect::MotionInhibited)
                );
            }
        }
    }

    #[test]
    fn capability_recovery_never_resumes_a_cancelled_mission() {
        let mut machine = diagnostic_machine(LifecycleState::Running);
        machine.dispatch(LifecycleEvent::CapabilityLost);
        assert_eq!(machine.state(), LifecycleState::Degraded);
        let recovered = machine.dispatch(LifecycleEvent::CapabilityRestored);
        assert_eq!(recovered.state(), LifecycleState::Armed);
        assert!(
            !recovered
                .effects()
                .as_slice()
                .contains(&LifecycleEffect::MissionStarted)
        );
    }

    #[test]
    fn replay_report_and_dot_are_deterministic() {
        let boot_id = boot(1);
        let events = [
            LifecycleEvent::ReconstructionReady {
                boot_id,
                revision: 1,
            },
            LifecycleEvent::Arm {
                boot_id,
                command_sequence: 1,
            },
            LifecycleEvent::Start {
                boot_id,
                arm_sequence: 1,
                command_sequence: 2,
            },
            LifecycleEvent::CapabilityLost,
            LifecycleEvent::CapabilityRestored,
            LifecycleEvent::Start {
                boot_id,
                arm_sequence: 1,
                command_sequence: 3,
            },
            LifecycleEvent::EmergencyStopAsserted,
            LifecycleEvent::EmergencyStopReleased {
                boot_id,
                assertion_generation: 1,
            },
            LifecycleEvent::ResetEmergencyStop {
                boot_id,
                assertion_generation: 1,
                command_sequence: 4,
            },
        ];
        let replay = || {
            let mut machine = LifecycleMachine::new(boot_id);
            let mut projections = ArrayVec::<LifecycleProjection, 9>::new();
            for event in events {
                projections.push(machine.dispatch(event));
            }
            projections
        };
        assert_eq!(replay(), replay());
        assert_eq!(lifecycle_transition_report(), lifecycle_transition_report());
        let dot = lifecycle_dot_graph();
        assert_eq!(dot, lifecycle_dot_graph());
        assert!(dot.contains("Running -> EStop [label=\"EmergencyStopAsserted\"];"));
        assert!(dot.contains("EStop -> Standby [label=\"ResetEmergencyStop\"];"));
        assert!(dot.contains("Boot -> Fault [label=\"ReconstructionTimedOut\"];"));
    }

    #[test]
    fn construction_and_every_dispatch_class_allocate_nothing() {
        let boot_id = boot(1);
        let allocations = measure(|| {
            for state in ALL_LIFECYCLE_STATES {
                for kind in ALL_LIFECYCLE_EVENT_KINDS {
                    let mut machine = diagnostic_machine(state);
                    let event = canonical_event(kind, &machine);
                    let projection = machine.dispatch(event);
                    std::hint::black_box(projection);
                }
            }
            let machine = LifecycleMachine::new(boot_id);
            std::hint::black_box(machine.startup_projection());
        });
        assert_eq!(allocations.count_total, 0);
    }

    #[test]
    fn restarting_from_every_state_always_returns_to_inhibited_boot() {
        let new_boot = boot(9);
        for state in ALL_LIFECYCLE_STATES {
            let old = diagnostic_machine(state);
            let restarted = LifecycleMachine::new(new_boot);
            assert_eq!(restarted.state(), LifecycleState::Boot);
            assert_ne!(restarted.boot_id(), old.boot_id());
            assert_eq!(
                restarted.startup_projection().effects().as_slice(),
                &[LifecycleEffect::MotionInhibited]
            );
        }
    }
}
