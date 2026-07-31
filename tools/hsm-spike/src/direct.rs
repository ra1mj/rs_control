use crate::{Candidate, EffectBatch, LifecycleEffect, LifecycleEvent, LifecycleState, Projection};

/// Exhaustive direct-enum lifecycle reference.
#[derive(Clone, Debug)]
pub struct DirectMachine {
    state: LifecycleState,
    startup: EffectBatch,
}

impl Candidate for DirectMachine {
    fn new() -> Self {
        let mut startup = EffectBatch::default();
        startup.push(LifecycleEffect::MotionInhibited);
        Self {
            state: LifecycleState::Boot,
            startup,
        }
    }

    fn startup_projection(&self) -> Projection {
        Projection {
            state: LifecycleState::Boot,
            effects: self.startup.clone(),
        }
    }

    fn state(&self) -> LifecycleState {
        self.state
    }

    fn dispatch(&mut self, event: LifecycleEvent) -> Projection {
        let target = transition(self.state, event);
        let effects = transition_effects(self.state, target);
        self.state = target;
        Projection {
            state: target,
            effects,
        }
    }
}

fn transition(state: LifecycleState, event: LifecycleEvent) -> LifecycleState {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    use LifecycleState::{Armed, Boot, Degraded, EStop, Fault, Running, Shutdown, Standby};

    match (state, event) {
        (Shutdown, ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost)
        | (Shutdown, CapabilityRestored | FaultDetected | ClearFault | EmergencyStop)
        | (Shutdown, ResetEmergencyStop | ShutdownRequested) => Shutdown,

        (EStop, ResetEmergencyStop) | (Fault, ClearFault) => Standby,
        (EStop, EmergencyStop | FaultDetected) | (Fault, FaultDetected) => state,

        (Boot | Standby | Armed | Running | Degraded | Fault, EmergencyStop) => EStop,
        (Boot | Standby | Armed | Running | Degraded | Fault | EStop, ShutdownRequested) => {
            Shutdown
        }
        (Boot | Standby | Armed | Running | Degraded, FaultDetected) => Fault,

        (Boot, ReconstructionReady) => Standby,
        (Standby, Arm) => Armed,
        (Armed, Start) | (Degraded, CapabilityRestored) => Running,
        (Armed, Disarm) | (Running, Disarm) | (Degraded, Disarm) => Standby,
        (Running, Stop) | (Degraded, Stop) => Armed,
        (Running, CapabilityLost) => Degraded,

        (Boot, Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored)
        | (Boot, ClearFault | ResetEmergencyStop)
        | (Standby, ReconstructionReady | Start | Stop | Disarm | CapabilityLost)
        | (Standby, CapabilityRestored | ClearFault | ResetEmergencyStop)
        | (Armed, ReconstructionReady | Arm | Stop | CapabilityLost | CapabilityRestored)
        | (Armed, ClearFault | ResetEmergencyStop)
        | (Running, ReconstructionReady | Arm | Start | CapabilityRestored)
        | (Running, ClearFault | ResetEmergencyStop)
        | (Degraded, ReconstructionReady | Arm | Start | CapabilityLost)
        | (Degraded, ClearFault | ResetEmergencyStop)
        | (Fault, ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost)
        | (Fault, CapabilityRestored | ResetEmergencyStop)
        | (EStop, ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost)
        | (EStop, CapabilityRestored | ClearFault) => state,
    }
}

fn transition_effects(source: LifecycleState, target: LifecycleState) -> EffectBatch {
    let mut effects = EffectBatch::default();
    if source == target {
        return effects;
    }
    if source == LifecycleState::Running {
        effects.push(LifecycleEffect::CancelMission);
        effects.push(LifecycleEffect::ReleaseResources);
    }
    if matches!(
        target,
        LifecycleState::Boot
            | LifecycleState::Standby
            | LifecycleState::Degraded
            | LifecycleState::Fault
            | LifecycleState::EStop
            | LifecycleState::Shutdown
    ) {
        effects.push(LifecycleEffect::MotionInhibited);
    }
    effects
}
