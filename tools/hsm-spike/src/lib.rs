//! Development-only lifecycle HSM comparison harness.

mod direct;
mod statig_candidate;

pub use direct::DirectMachine;
pub use statig_candidate::StatigMachine;

use std::fmt::Write as _;

use arrayvec::ArrayVec;

/// Maximum effects produced by one prototype transition.
pub const MAX_EFFECTS: usize = 3;

/// Lifecycle leaf states shared by both candidates.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum LifecycleState {
    /// Reconstruct live health while motion is inhibited.
    Boot,
    /// Safe idle state.
    Standby,
    /// Explicitly armed but not running.
    Armed,
    /// Mission execution is permitted.
    Running,
    /// Capability loss has stopped current mission work.
    Degraded,
    /// A typed fault prevents arming.
    Fault,
    /// Latched emergency stop.
    EStop,
    /// Completed shutdown state.
    Shutdown,
}

impl LifecycleState {
    /// Stable report spelling.
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

    const fn checksum_value(self) -> u64 {
        match self {
            Self::Boot => 1,
            Self::Standby => 2,
            Self::Armed => 3,
            Self::Running => 4,
            Self::Degraded => 5,
            Self::Fault => 6,
            Self::EStop => 7,
            Self::Shutdown => 8,
        }
    }
}

/// Serialized lifecycle events shared by both candidates.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleEvent {
    /// Live health reconstruction completed successfully.
    ReconstructionReady,
    /// Fresh explicit arm request.
    Arm,
    /// Begin running after arm.
    Start,
    /// Stop mission execution while retaining arm.
    Stop,
    /// Return to safe standby.
    Disarm,
    /// A running capability became unavailable.
    CapabilityLost,
    /// A degraded capability recovered.
    CapabilityRestored,
    /// A fault was observed.
    FaultDetected,
    /// Explicit fault recovery completed.
    ClearFault,
    /// Physical or software emergency stop observation.
    EmergencyStop,
    /// Explicit emergency-stop reset sequence completed.
    ResetEmergencyStop,
    /// Complete shutdown was requested.
    ShutdownRequested,
}

impl LifecycleEvent {
    /// Stable report spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::ReconstructionReady => "ReconstructionReady",
            Self::Arm => "Arm",
            Self::Start => "Start",
            Self::Stop => "Stop",
            Self::Disarm => "Disarm",
            Self::CapabilityLost => "CapabilityLost",
            Self::CapabilityRestored => "CapabilityRestored",
            Self::FaultDetected => "FaultDetected",
            Self::ClearFault => "ClearFault",
            Self::EmergencyStop => "EmergencyStop",
            Self::ResetEmergencyStop => "ResetEmergencyStop",
            Self::ShutdownRequested => "ShutdownRequested",
        }
    }
}

/// Ordered safety effects emitted synchronously by a transition.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LifecycleEffect {
    /// Motion output remains inhibited.
    MotionInhibited,
    /// Cancel all active mission work.
    CancelMission,
    /// Release all resources after cancellation.
    ReleaseResources,
}

impl LifecycleEffect {
    const fn checksum_value(self) -> u64 {
        match self {
            Self::MotionInhibited => 11,
            Self::CancelMission => 12,
            Self::ReleaseResources => 13,
        }
    }
}

/// Fixed-capacity ordered transition effects.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct EffectBatch {
    items: ArrayVec<LifecycleEffect, MAX_EFFECTS>,
}

impl EffectBatch {
    pub(crate) fn push(&mut self, effect: LifecycleEffect) {
        self.items
            .try_push(effect)
            .expect("prototype transition effect bound is statically proven");
    }

    /// Returns transition effects in execution order.
    #[must_use]
    pub fn as_slice(&self) -> &[LifecycleEffect] {
        self.items.as_slice()
    }
}

/// Observable result used for candidate conformance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Projection {
    /// State after initialization or event dispatch.
    pub state: LifecycleState,
    /// Synchronous ordered effects.
    pub effects: EffectBatch,
}

/// Common interface implemented by both lifecycle candidates.
pub trait Candidate: Sized {
    /// Constructs and initializes an inhibited `Boot` machine.
    fn new() -> Self;

    /// Returns the initial entry projection.
    fn startup_projection(&self) -> Projection;

    /// Returns the current leaf state.
    fn state(&self) -> LifecycleState;

    /// Dispatches one event synchronously.
    fn dispatch(&mut self, event: LifecycleEvent) -> Projection;
}

/// Stable set of all prototype states.
pub const ALL_STATES: [LifecycleState; 8] = [
    LifecycleState::Boot,
    LifecycleState::Standby,
    LifecycleState::Armed,
    LifecycleState::Running,
    LifecycleState::Degraded,
    LifecycleState::Fault,
    LifecycleState::EStop,
    LifecycleState::Shutdown,
];

/// Stable set of all prototype events.
pub const ALL_EVENTS: [LifecycleEvent; 12] = [
    LifecycleEvent::ReconstructionReady,
    LifecycleEvent::Arm,
    LifecycleEvent::Start,
    LifecycleEvent::Stop,
    LifecycleEvent::Disarm,
    LifecycleEvent::CapabilityLost,
    LifecycleEvent::CapabilityRestored,
    LifecycleEvent::FaultDetected,
    LifecycleEvent::ClearFault,
    LifecycleEvent::EmergencyStop,
    LifecycleEvent::ResetEmergencyStop,
    LifecycleEvent::ShutdownRequested,
];

/// Representative deterministic trace used by tests and size binaries.
pub const WORKLOAD: [LifecycleEvent; 19] = [
    LifecycleEvent::Arm,
    LifecycleEvent::ReconstructionReady,
    LifecycleEvent::ReconstructionReady,
    LifecycleEvent::Start,
    LifecycleEvent::Arm,
    LifecycleEvent::Start,
    LifecycleEvent::CapabilityLost,
    LifecycleEvent::CapabilityRestored,
    LifecycleEvent::Stop,
    LifecycleEvent::Start,
    LifecycleEvent::FaultDetected,
    LifecycleEvent::ClearFault,
    LifecycleEvent::Arm,
    LifecycleEvent::Start,
    LifecycleEvent::EmergencyStop,
    LifecycleEvent::EmergencyStop,
    LifecycleEvent::ResetEmergencyStop,
    LifecycleEvent::Arm,
    LifecycleEvent::ShutdownRequested,
];

/// Runs a candidate workload and returns a stable observable checksum.
#[must_use]
pub fn workload_checksum<C: Candidate>() -> u64 {
    let mut checksum = 0_u64;
    for _ in 0..128 {
        let mut machine = C::new();
        let startup = machine.startup_projection();
        checksum = mix_projection(checksum, &startup);
        for event in WORKLOAD {
            let projection = machine.dispatch(event);
            checksum = mix_projection(checksum, &projection);
        }
    }
    checksum
}

/// Generates a complete stable state/event projection report.
#[must_use]
pub fn transition_report<C: Candidate>() -> String {
    let mut report = String::new();
    for state in ALL_STATES {
        for event in ALL_EVENTS {
            let mut machine = machine_at::<C>(state);
            let projection = machine.dispatch(event);
            writeln!(
                report,
                "{} + {} -> {} {:?}",
                state.as_str(),
                event.as_str(),
                projection.state.as_str(),
                projection.effects.as_slice()
            )
            .expect("writing to String cannot fail");
        }
    }
    report
}

/// Generates a deterministic DOT graph from observed changed-state projections.
#[must_use]
pub fn transition_dot<C: Candidate>(graph_name: &str) -> String {
    let mut dot = String::new();
    writeln!(dot, "digraph {graph_name} {{").expect("writing to String cannot fail");
    for state in ALL_STATES {
        writeln!(dot, "  {};", state.as_str()).expect("writing to String cannot fail");
    }
    for state in ALL_STATES {
        for event in ALL_EVENTS {
            let mut machine = machine_at::<C>(state);
            let projection = machine.dispatch(event);
            if projection.state != state {
                writeln!(
                    dot,
                    "  {} -> {} [label=\"{}\"];",
                    state.as_str(),
                    projection.state.as_str(),
                    event.as_str()
                )
                .expect("writing to String cannot fail");
            }
        }
    }
    dot.push_str("}\n");
    dot
}

fn machine_at<C: Candidate>(target: LifecycleState) -> C {
    let mut machine = C::new();
    for event in path_to(target) {
        machine.dispatch(*event);
    }
    assert_eq!(machine.state(), target);
    machine
}

fn path_to(state: LifecycleState) -> &'static [LifecycleEvent] {
    use LifecycleEvent::{
        Arm, CapabilityLost, EmergencyStop, FaultDetected, ReconstructionReady, ShutdownRequested,
        Start,
    };

    match state {
        LifecycleState::Boot => &[],
        LifecycleState::Standby => &[ReconstructionReady],
        LifecycleState::Armed => &[ReconstructionReady, Arm],
        LifecycleState::Running => &[ReconstructionReady, Arm, Start],
        LifecycleState::Degraded => &[ReconstructionReady, Arm, Start, CapabilityLost],
        LifecycleState::Fault => &[FaultDetected],
        LifecycleState::EStop => &[EmergencyStop],
        LifecycleState::Shutdown => &[ShutdownRequested],
    }
}

fn mix_projection(mut checksum: u64, projection: &Projection) -> u64 {
    checksum = checksum
        .wrapping_mul(1_099_511_628_211)
        .wrapping_add(projection.state.checksum_value());
    for effect in projection.effects.as_slice() {
        checksum = checksum
            .wrapping_mul(1_099_511_628_211)
            .wrapping_add(effect.checksum_value());
    }
    checksum
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;

    use super::*;

    fn assert_conformance_trace(events: &[LifecycleEvent]) {
        let mut direct = DirectMachine::new();
        let mut statig = StatigMachine::new();
        assert_eq!(direct.startup_projection(), statig.startup_projection());
        for event in events {
            assert_eq!(direct.dispatch(*event), statig.dispatch(*event));
        }
    }

    #[test]
    fn candidates_match_valid_invalid_duplicate_and_preemptive_traces() {
        assert_conformance_trace(&WORKLOAD);
        assert_conformance_trace(&[
            LifecycleEvent::FaultDetected,
            LifecycleEvent::FaultDetected,
            LifecycleEvent::ClearFault,
            LifecycleEvent::EmergencyStop,
            LifecycleEvent::ResetEmergencyStop,
            LifecycleEvent::ShutdownRequested,
            LifecycleEvent::Arm,
        ]);
        assert_eq!(
            transition_report::<DirectMachine>(),
            transition_report::<StatigMachine>()
        );
    }

    #[test]
    fn running_exit_orders_cancellation_before_release_and_inhibit() {
        for event in [
            LifecycleEvent::Stop,
            LifecycleEvent::Disarm,
            LifecycleEvent::CapabilityLost,
            LifecycleEvent::FaultDetected,
            LifecycleEvent::EmergencyStop,
            LifecycleEvent::ShutdownRequested,
        ] {
            let mut direct = machine_at::<DirectMachine>(LifecycleState::Running);
            let mut statig = machine_at::<StatigMachine>(LifecycleState::Running);
            let direct_projection = direct.dispatch(event);
            let statig_projection = statig.dispatch(event);
            assert_eq!(direct_projection, statig_projection);
            assert_eq!(
                &direct_projection.effects.as_slice()[..2],
                &[
                    LifecycleEffect::CancelMission,
                    LifecycleEffect::ReleaseResources
                ]
            );
        }
    }

    #[test]
    fn emergency_stop_preempts_every_state_except_shutdown() {
        for state in ALL_STATES {
            let mut direct = machine_at::<DirectMachine>(state);
            let projection = direct.dispatch(LifecycleEvent::EmergencyStop);
            let expected = if state == LifecycleState::Shutdown {
                LifecycleState::Shutdown
            } else {
                LifecycleState::EStop
            };
            assert_eq!(projection.state, expected);
        }
    }

    #[test]
    fn restart_never_restores_active_state() {
        let mut direct = machine_at::<DirectMachine>(LifecycleState::Running);
        let mut statig = machine_at::<StatigMachine>(LifecycleState::Running);
        direct.dispatch(LifecycleEvent::CapabilityLost);
        statig.dispatch(LifecycleEvent::CapabilityLost);

        let direct = DirectMachine::new();
        let statig = StatigMachine::new();
        assert_eq!(direct.state(), LifecycleState::Boot);
        assert_eq!(statig.state(), LifecycleState::Boot);
        assert_eq!(
            direct.startup_projection().effects.as_slice(),
            &[LifecycleEffect::MotionInhibited]
        );
        assert_eq!(direct.startup_projection(), statig.startup_projection());
    }

    #[test]
    fn reports_and_dot_are_byte_stable() {
        let direct_report = transition_report::<DirectMachine>();
        let statig_report = transition_report::<StatigMachine>();
        assert_eq!(direct_report, transition_report::<DirectMachine>(),);
        assert_eq!(direct_report, statig_report);
        assert_eq!(
            direct_report.lines().count(),
            ALL_STATES.len() * ALL_EVENTS.len()
        );

        let direct_dot = transition_dot::<DirectMachine>("lifecycle");
        let statig_dot = transition_dot::<StatigMachine>("lifecycle");
        assert_eq!(direct_dot, transition_dot::<DirectMachine>("lifecycle"),);
        assert_eq!(direct_dot, statig_dot);
        assert!(direct_dot.contains("Running -> EStop [label=\"EmergencyStop\"];"));
        assert!(direct_dot.contains("Running -> Shutdown [label=\"ShutdownRequested\"];"));
    }

    #[test]
    fn both_dispatch_paths_allocate_nothing() {
        let mut warm_direct = DirectMachine::new();
        let mut warm_statig = StatigMachine::new();
        for event in WORKLOAD {
            warm_direct.dispatch(event);
            warm_statig.dispatch(event);
        }

        let allocations = measure(|| {
            for _ in 0..128 {
                let mut direct = DirectMachine::new();
                let mut statig = StatigMachine::new();
                for event in WORKLOAD {
                    let direct_projection = direct.dispatch(event);
                    let statig_projection = statig.dispatch(event);
                    assert_eq!(direct_projection, statig_projection);
                }
            }
        });
        assert_eq!(allocations.count_total, 0);
    }
}
