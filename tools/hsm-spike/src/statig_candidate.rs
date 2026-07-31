use statig::Outcome::{self, Handled, Super, Transition};
use statig::blocking::{
    IntoStateMachine, IntoStateMachineExt as _, State as StatigStateTrait,
    Superstate as StatigSuperstateTrait,
};

use crate::{Candidate, EffectBatch, LifecycleEffect, LifecycleEvent, LifecycleState, Projection};

#[derive(Default)]
struct EffectContext {
    effects: EffectBatch,
    #[cfg(test)]
    before_transition: Option<(StatigState, StatigState)>,
    #[cfg(test)]
    after_transition: Option<(StatigState, StatigState)>,
}

#[derive(Default)]
struct StatigLifecycle;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum StatigState {
    Boot,
    Standby,
    Armed,
    Running,
    Degraded,
    Fault,
    EStop,
    Shutdown,
}

#[derive(Clone, Copy, Debug)]
enum StatigSuperstate {
    Live,
}

impl IntoStateMachine for StatigLifecycle {
    type Event<'event> = LifecycleEvent;
    type Context<'context> = EffectContext;
    type State = StatigState;
    type Superstate<'sub> = StatigSuperstate;

    fn initial() -> Self::State {
        StatigState::Boot
    }

    fn before_transition(
        &mut self,
        source: &Self::State,
        target: &Self::State,
        context: &mut Self::Context<'_>,
    ) {
        #[cfg(test)]
        {
            context.before_transition = Some((*source, *target));
        }
        #[cfg(not(test))]
        let _ = (source, target, context);
    }

    fn after_transition(
        &mut self,
        source: &Self::State,
        target: &Self::State,
        context: &mut Self::Context<'_>,
    ) {
        #[cfg(test)]
        {
            context.after_transition = Some((*source, *target));
        }
        #[cfg(not(test))]
        let _ = (source, target, context);
    }
}

impl StatigStateTrait<StatigLifecycle> for StatigState {
    fn call_handler(
        &mut self,
        _shared: &mut StatigLifecycle,
        event: &LifecycleEvent,
        _context: &mut EffectContext,
    ) -> Outcome<Self> {
        match self {
            Self::Boot => boot(event),
            Self::Standby => standby(event),
            Self::Armed => armed(event),
            Self::Running => running(event),
            Self::Degraded => degraded(event),
            Self::Fault => fault(event),
            Self::EStop => e_stop(event),
            Self::Shutdown => shutdown(event),
        }
    }

    fn call_entry_action(&mut self, _shared: &mut StatigLifecycle, context: &mut EffectContext) {
        match self {
            Self::Boot
            | Self::Standby
            | Self::Degraded
            | Self::Fault
            | Self::EStop
            | Self::Shutdown => context.effects.push(LifecycleEffect::MotionInhibited),
            Self::Armed | Self::Running => {}
        }
    }

    fn call_exit_action(&mut self, _shared: &mut StatigLifecycle, context: &mut EffectContext) {
        match self {
            Self::Running => {
                context.effects.push(LifecycleEffect::CancelMission);
                context.effects.push(LifecycleEffect::ReleaseResources);
            }
            Self::Boot
            | Self::Standby
            | Self::Armed
            | Self::Degraded
            | Self::Fault
            | Self::EStop
            | Self::Shutdown => {}
        }
    }

    fn superstate(&mut self) -> Option<StatigSuperstate> {
        match self {
            Self::Boot
            | Self::Standby
            | Self::Armed
            | Self::Running
            | Self::Degraded
            | Self::Fault
            | Self::EStop => Some(StatigSuperstate::Live),
            Self::Shutdown => None,
        }
    }
}

impl StatigSuperstateTrait<StatigLifecycle> for StatigSuperstate {
    fn call_handler(
        &mut self,
        _shared: &mut StatigLifecycle,
        event: &LifecycleEvent,
        _context: &mut EffectContext,
    ) -> Outcome<StatigState> {
        match self {
            Self::Live => live(event),
        }
    }
}

fn boot(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        ReconstructionReady => Transition(StatigState::Standby),
        FaultDetected | EmergencyStop | ShutdownRequested => Super,
        Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored | ClearFault
        | ResetEmergencyStop => Handled,
    }
}

fn standby(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        Arm => Transition(StatigState::Armed),
        FaultDetected | EmergencyStop | ShutdownRequested => Super,
        ReconstructionReady | Start | Stop | Disarm | CapabilityLost | CapabilityRestored
        | ClearFault | ResetEmergencyStop => Handled,
    }
}

fn armed(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        Start => Transition(StatigState::Running),
        Disarm => Transition(StatigState::Standby),
        FaultDetected | EmergencyStop | ShutdownRequested => Super,
        ReconstructionReady | Arm | Stop | CapabilityLost | CapabilityRestored | ClearFault
        | ResetEmergencyStop => Handled,
    }
}

fn running(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        Stop => Transition(StatigState::Armed),
        Disarm => Transition(StatigState::Standby),
        CapabilityLost => Transition(StatigState::Degraded),
        FaultDetected | EmergencyStop | ShutdownRequested => Super,
        ReconstructionReady | Arm | Start | CapabilityRestored | ClearFault
        | ResetEmergencyStop => Handled,
    }
}

fn degraded(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        CapabilityRestored => Transition(StatigState::Running),
        Stop => Transition(StatigState::Armed),
        Disarm => Transition(StatigState::Standby),
        FaultDetected | EmergencyStop | ShutdownRequested => Super,
        ReconstructionReady | Arm | Start | CapabilityLost | ClearFault | ResetEmergencyStop => {
            Handled
        }
    }
}

fn fault(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        ClearFault => Transition(StatigState::Standby),
        EmergencyStop | ShutdownRequested => Super,
        ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored
        | FaultDetected | ResetEmergencyStop => Handled,
    }
}

fn e_stop(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        ResetEmergencyStop => Transition(StatigState::Standby),
        ShutdownRequested => Super,
        ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored
        | FaultDetected | ClearFault | EmergencyStop => Handled,
    }
}

fn shutdown(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored
        | FaultDetected | ClearFault | EmergencyStop | ResetEmergencyStop | ShutdownRequested => {
            Handled
        }
    }
}

fn live(event: &LifecycleEvent) -> Outcome<StatigState> {
    use LifecycleEvent::{
        Arm, CapabilityLost, CapabilityRestored, ClearFault, Disarm, EmergencyStop, FaultDetected,
        ReconstructionReady, ResetEmergencyStop, ShutdownRequested, Start, Stop,
    };
    match event {
        FaultDetected => Transition(StatigState::Fault),
        EmergencyStop => Transition(StatigState::EStop),
        ShutdownRequested => Transition(StatigState::Shutdown),
        ReconstructionReady | Arm | Start | Stop | Disarm | CapabilityLost | CapabilityRestored
        | ClearFault | ResetEmergencyStop => Handled,
    }
}

/// Synchronous no-macro statig lifecycle candidate.
pub struct StatigMachine {
    machine: statig::blocking::StateMachine<StatigLifecycle>,
    startup: EffectBatch,
}

impl Candidate for StatigMachine {
    fn new() -> Self {
        let mut machine = StatigLifecycle.state_machine();
        let mut context = EffectContext::default();
        machine.init_with_context(&mut context);
        Self {
            machine,
            startup: context.effects,
        }
    }

    fn startup_projection(&self) -> Projection {
        Projection {
            state: LifecycleState::Boot,
            effects: self.startup.clone(),
        }
    }

    fn state(&self) -> LifecycleState {
        match self.machine.state() {
            StatigState::Boot => LifecycleState::Boot,
            StatigState::Standby => LifecycleState::Standby,
            StatigState::Armed => LifecycleState::Armed,
            StatigState::Running => LifecycleState::Running,
            StatigState::Degraded => LifecycleState::Degraded,
            StatigState::Fault => LifecycleState::Fault,
            StatigState::EStop => LifecycleState::EStop,
            StatigState::Shutdown => LifecycleState::Shutdown,
        }
    }

    fn dispatch(&mut self, event: LifecycleEvent) -> Projection {
        let mut context = EffectContext::default();
        self.machine.handle_with_context(&event, &mut context);
        Projection {
            state: self.state(),
            effects: context.effects,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_0_4_1_before_transition_hook_reverses_documented_arguments() {
        let mut machine = StatigLifecycle.state_machine();
        let mut startup = EffectContext::default();
        machine.init_with_context(&mut startup);
        let mut context = EffectContext::default();
        machine.handle_with_context(&LifecycleEvent::ReconstructionReady, &mut context);

        assert_eq!(
            context.before_transition,
            Some((StatigState::Standby, StatigState::Boot))
        );
        assert_eq!(
            context.after_transition,
            Some((StatigState::Boot, StatigState::Standby))
        );
    }
}
