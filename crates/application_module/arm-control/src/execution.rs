//! Deterministic host simulation/logging and closed physical-executor contract.

use arm_algorithms::{CollisionChecked, CollisionUnchecked};
use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorBinding, ArmProfileId, CommandEnvelope, KinematicGroupId, MAX_JOINTS,
    RotaryMitSetpoint,
};

use crate::{AdmittedArmCommand, ArmControlError};

/// One host-only simulated actuator snapshot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulatedActuatorState {
    binding: ArmActuatorBinding,
    setpoint: RotaryMitSetpoint,
}

impl SimulatedActuatorState {
    /// Returns stable binding identity.
    #[must_use]
    pub const fn binding(self) -> ArmActuatorBinding {
        self.binding
    }

    /// Returns the deterministic accepted actuator target.
    #[must_use]
    pub const fn setpoint(self) -> RotaryMitSetpoint {
        self.setpoint
    }
}

/// Pure deterministic host executor with no clock, sleep or I/O.
#[derive(Clone, Debug, Default)]
pub struct SimExecutor {
    generation: u64,
    states: ArrayVec<SimulatedActuatorState, MAX_JOINTS>,
}

impl SimExecutor {
    /// Applies one collision-unchecked command only to host simulation state.
    pub fn execute(
        &mut self,
        command: &AdmittedArmCommand<CollisionUnchecked>,
    ) -> &[SimulatedActuatorState] {
        self.states.clear();
        for actuator in command.actuator_commands() {
            self.states.push(SimulatedActuatorState {
                binding: actuator.binding(),
                setpoint: actuator.setpoint(),
            });
        }
        self.generation = self.generation.saturating_add(1);
        &self.states
    }

    /// Returns the number of applied simulated commands.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }
}

/// Bounded typed record copied into caller-owned storage.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmExecutionRecord {
    envelope: CommandEnvelope,
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    setpoints: ArrayVec<SimulatedActuatorState, MAX_JOINTS>,
}

impl ArmExecutionRecord {
    /// Returns command correlation and local authority metadata.
    #[must_use]
    pub const fn envelope(&self) -> CommandEnvelope {
        self.envelope
    }

    /// Returns recorded outputs in stable actuator order.
    #[must_use]
    pub fn setpoints(&self) -> &[SimulatedActuatorState] {
        &self.setpoints
    }
}

/// Caller-owned bounded execution record storage.
#[derive(Clone, Debug)]
pub struct LogExecutor<const CAPACITY: usize> {
    records: ArrayVec<ArmExecutionRecord, CAPACITY>,
}

impl<const CAPACITY: usize> Default for LogExecutor<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> LogExecutor<CAPACITY> {
    /// Creates empty storage without I/O or allocation.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            records: ArrayVec::new_const(),
        }
    }

    /// Appends one collision-unchecked host record.
    pub fn execute(
        &mut self,
        command: &AdmittedArmCommand<CollisionUnchecked>,
    ) -> Result<(), ArmControlError> {
        let mut setpoints = ArrayVec::new();
        for actuator in command.actuator_commands() {
            setpoints.push(SimulatedActuatorState {
                binding: actuator.binding(),
                setpoint: actuator.setpoint(),
            });
        }
        self.records
            .try_push(ArmExecutionRecord {
                envelope: command.envelope(),
                profile_id: command.profile_id(),
                group_id: command.group_id(),
                generation: command.generation(),
                setpoints,
            })
            .map_err(|_| ArmControlError::Capacity { capacity: CAPACITY })
    }

    /// Returns immutable caller-owned records.
    #[must_use]
    pub fn records(&self) -> &[ArmExecutionRecord] {
        &self.records
    }
}

/// Future physical executors can accept only collision-checked admission.
///
/// This delivery exposes no constructor or elevation path for
/// `AdmittedArmCommand<CollisionChecked>`.
pub trait PhysicalArmExecutor {
    /// Backend-specific typed failure.
    type Error;

    /// Executes only a command carrying qualified collision evidence.
    fn execute(
        &mut self,
        command: &AdmittedArmCommand<CollisionChecked>,
    ) -> Result<(), Self::Error>;
}
