//! Fail-closed arm proposal admission.

use std::marker::PhantomData;

use arm_algorithms::CollisionUnchecked;
use arrayvec::ArrayVec;
use control_core::{LeaseEnforcementCache, LifecycleState};
use message_model::{
    ActionStatus, ArmActuatorBinding, ArmControlMode, ArmControlProposal, ArmControlValue,
    ArmJointObservation, ClockState, CommandEnvelope, DataQuality, KinematicGroupId, MAX_JOINTS,
    RotaryMitSetpoint,
};

use crate::{ArmControlConfig, ArmControlError};

/// One command intent carrying explicit collision evidence as typestate.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmControlIntent<Evidence> {
    envelope: CommandEnvelope,
    proposal: ArmControlProposal,
    _evidence: PhantomData<Evidence>,
}

impl ArmControlIntent<CollisionUnchecked> {
    /// Creates a host-software intent that has not been collision checked.
    #[must_use]
    pub const fn collision_unchecked(
        envelope: CommandEnvelope,
        proposal: ArmControlProposal,
    ) -> Self {
        Self {
            envelope,
            proposal,
            _evidence: PhantomData,
        }
    }
}

impl<Evidence> ArmControlIntent<Evidence> {
    /// Returns authority/deadline metadata.
    #[must_use]
    pub const fn envelope(&self) -> CommandEnvelope {
        self.envelope
    }

    /// Returns the semantic proposal.
    #[must_use]
    pub const fn proposal(&self) -> &ArmControlProposal {
        &self.proposal
    }
}

/// One immutable binding and validated MIT-compatible output.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdmittedActuatorCommand {
    binding: ArmActuatorBinding,
    setpoint: RotaryMitSetpoint,
}

impl AdmittedActuatorCommand {
    /// Returns stable binding identity.
    #[must_use]
    pub const fn binding(self) -> ArmActuatorBinding {
        self.binding
    }

    /// Returns bounded actuator-space output.
    #[must_use]
    pub const fn setpoint(self) -> RotaryMitSetpoint {
        self.setpoint
    }
}

/// Fully admitted semantic output, still carrying collision typestate.
#[derive(Clone, Debug, PartialEq)]
pub struct AdmittedArmCommand<Evidence> {
    envelope: CommandEnvelope,
    profile_id: message_model::ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    actuator_commands: ArrayVec<AdmittedActuatorCommand, MAX_JOINTS>,
    _evidence: PhantomData<Evidence>,
}

impl<Evidence> AdmittedArmCommand<Evidence> {
    /// Returns local authority/deadline metadata.
    #[must_use]
    pub const fn envelope(&self) -> CommandEnvelope {
        self.envelope
    }

    /// Returns immutable morphology identity.
    #[must_use]
    pub const fn profile_id(&self) -> message_model::ArmProfileId {
        self.profile_id
    }

    /// Returns exact group identity.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns exact proposal/model generation.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns outputs in compiled actuator order.
    #[must_use]
    pub fn actuator_commands(&self) -> &[AdmittedActuatorCommand] {
        &self.actuator_commands
    }
}

/// Stateful exclusive-sequence and slew admission for one compiled group.
#[derive(Clone, Debug)]
pub struct ArmControlAdmission {
    config: ArmControlConfig,
    last_observation_sequence: Option<u64>,
    last_admission_monotonic_ns: Option<u64>,
    last_setpoints: ArrayVec<RotaryMitSetpoint, MAX_JOINTS>,
}

impl ArmControlAdmission {
    /// Starts with no accepted observation or prior output.
    #[must_use]
    pub const fn new(config: ArmControlConfig) -> Self {
        Self {
            config,
            last_observation_sequence: None,
            last_admission_monotonic_ns: None,
            last_setpoints: ArrayVec::new_const(),
        }
    }

    /// Returns immutable configuration.
    #[must_use]
    pub const fn config(&self) -> &ArmControlConfig {
        &self.config
    }

    /// Validates an unchecked command for host simulation/logging.
    ///
    /// The returned value remains `CollisionUnchecked` and therefore cannot
    /// satisfy [`crate::PhysicalArmExecutor`].
    #[allow(clippy::too_many_arguments)]
    pub fn admit_collision_unchecked<const LEASE_CAPACITY: usize>(
        &mut self,
        intent: &ArmControlIntent<CollisionUnchecked>,
        observation: &ArmJointObservation,
        lease_cache: &LeaseEnforcementCache<LEASE_CAPACITY>,
        lifecycle: LifecycleState,
        action_status: ActionStatus,
        now_monotonic_ns: u64,
    ) -> Result<AdmittedArmCommand<CollisionUnchecked>, ArmControlError> {
        if lifecycle != LifecycleState::Running {
            return Err(ArmControlError::ExecutionClosed {
                field: "robot_lifecycle",
            });
        }
        if action_status != ActionStatus::Executing {
            return Err(ArmControlError::ExecutionClosed {
                field: "action_lifecycle",
            });
        }
        lease_cache.authorize(&intent.envelope, now_monotonic_ns)?;
        self.validate_identity(intent.proposal(), observation)?;
        self.validate_observation(observation, now_monotonic_ns)?;

        let setpoints = self.project(intent.proposal(), observation)?;
        self.validate_limits(&setpoints)?;
        self.validate_slew(&setpoints, now_monotonic_ns)?;

        self.last_observation_sequence = Some(observation.header().sequence());
        self.last_admission_monotonic_ns = Some(now_monotonic_ns);
        self.last_setpoints.clear();
        self.last_setpoints
            .try_extend_from_slice(&setpoints)
            .map_err(|_| ArmControlError::InvalidConfiguration {
                reason: "admitted output exceeds MAX_JOINTS",
            })?;

        let mut actuator_commands = ArrayVec::new();
        for (binding, setpoint) in self
            .config
            .actuator_order()
            .bindings()
            .iter()
            .copied()
            .zip(setpoints)
        {
            actuator_commands.push(AdmittedActuatorCommand { binding, setpoint });
        }
        Ok(AdmittedArmCommand {
            envelope: intent.envelope,
            profile_id: self.config.profile_id(),
            group_id: self.config.group_id(),
            generation: self.config.generation(),
            actuator_commands,
            _evidence: PhantomData,
        })
    }

    fn validate_identity(
        &self,
        proposal: &ArmControlProposal,
        observation: &ArmJointObservation,
    ) -> Result<(), ArmControlError> {
        let context = proposal.context();
        for (field, matches) in [
            (
                "proposal.profile_id",
                context.profile_id() == self.config.profile_id(),
            ),
            (
                "proposal.group_id",
                context.group_id() == self.config.group_id(),
            ),
            (
                "proposal.generation",
                context.generation() == self.config.generation(),
            ),
            (
                "observation.profile_id",
                observation.profile_id() == self.config.profile_id(),
            ),
            (
                "observation.group_id",
                observation.group_id() == self.config.group_id(),
            ),
        ] {
            if !matches {
                return Err(ArmControlError::IdentityMismatch { field });
            }
        }
        if proposal.dof() as usize != self.config.actuator_order().len()
            || observation.samples().len() != self.config.joint_order().len()
        {
            return Err(ArmControlError::InvalidProposal {
                reason: "proposal or observation DOF differs from compiled group",
            });
        }
        for (sample, expected) in observation
            .samples()
            .iter()
            .zip(self.config.joint_order().joints())
        {
            if sample.descriptor() != *expected {
                return Err(ArmControlError::IdentityMismatch {
                    field: "observation.joint_order",
                });
            }
        }
        Ok(())
    }

    fn validate_observation(
        &self,
        observation: &ArmJointObservation,
        now_monotonic_ns: u64,
    ) -> Result<(), ArmControlError> {
        let header = observation.header();
        if header.source_id() != self.config.observation_source() {
            return Err(ArmControlError::IdentityMismatch {
                field: "observation.source_id",
            });
        }
        if header.boot_id() != self.config.observation_boot() {
            return Err(ArmControlError::IdentityMismatch {
                field: "observation.boot_id",
            });
        }
        if header.clock_status().state != ClockState::Local
            || header.quality() != DataQuality::Valid
            || header.source_monotonic_ns() > now_monotonic_ns
        {
            return Err(ArmControlError::InvalidObservation);
        }
        if header
            .source_expiration_ns()
            .is_none_or(|expiration| now_monotonic_ns >= expiration)
        {
            return Err(ArmControlError::StaleObservation { now_monotonic_ns });
        }
        if let Some(previous) = self.last_observation_sequence
            && header.sequence() <= previous
        {
            return Err(ArmControlError::StaleSequence {
                previous,
                actual: header.sequence(),
            });
        }
        Ok(())
    }

    fn project(
        &self,
        proposal: &ArmControlProposal,
        observation: &ArmJointObservation,
    ) -> Result<ArrayVec<RotaryMitSetpoint, MAX_JOINTS>, ArmControlError> {
        let mut output = ArrayVec::new();
        for (index, (((value, sample), transmission), limits)) in proposal
            .values()
            .iter()
            .zip(observation.samples())
            .zip(self.config.transmissions())
            .zip(self.config.limits())
            .enumerate()
        {
            let measured_position = transmission.actuator_position_rad(sample.position())?;
            let setpoint = match (proposal.mode(), value) {
                (ArmControlMode::AbsolutePosition, ArmControlValue::Position(position)) => {
                    RotaryMitSetpoint::new(
                        transmission.actuator_position_rad(*position)?,
                        0.0,
                        limits.fixed_kp(),
                        limits.fixed_kd(),
                        0.0,
                    )?
                }
                (ArmControlMode::PositionResidual, ArmControlValue::Position(residual)) => {
                    if residual.kind() != sample.position().kind() {
                        return Err(ArmControlError::InvalidProposal {
                            reason: "position residual unit differs from observation",
                        });
                    }
                    let joint_target = match residual.kind() {
                        message_model::ArmCoordinateKind::Angular => {
                            message_model::ArmPosition::angular_rad(
                                sample.position().value_si() + residual.value_si(),
                            )?
                        }
                        message_model::ArmCoordinateKind::Linear => {
                            message_model::ArmPosition::linear_m(
                                sample.position().value_si() + residual.value_si(),
                            )?
                        }
                    };
                    RotaryMitSetpoint::new(
                        transmission.actuator_position_rad(joint_target)?,
                        0.0,
                        limits.fixed_kp(),
                        limits.fixed_kd(),
                        0.0,
                    )?
                }
                (ArmControlMode::Velocity, ArmControlValue::Velocity(velocity)) => {
                    RotaryMitSetpoint::new(
                        measured_position,
                        transmission.actuator_velocity_radps(*velocity)?,
                        0.0,
                        limits.fixed_kd(),
                        0.0,
                    )?
                }
                (ArmControlMode::DirectRotaryTorque, ArmControlValue::RotaryTorque(torque)) => {
                    RotaryMitSetpoint::new(measured_position, 0.0, 0.0, 0.0, torque.get())?
                }
                (ArmControlMode::Mit, ArmControlValue::Mit(setpoint)) => {
                    if setpoint.kp() != limits.fixed_kp() || setpoint.kd() != limits.fixed_kd() {
                        return Err(ArmControlError::IdentityMismatch {
                            field: "proposal.fixed_mit_gains",
                        });
                    }
                    *setpoint
                }
                _ => {
                    return Err(ArmControlError::InvalidProposal {
                        reason: "proposal mode and payload value differ",
                    });
                }
            };
            if index >= MAX_JOINTS {
                return Err(ArmControlError::InvalidProposal {
                    reason: "proposal exceeds MAX_JOINTS",
                });
            }
            output.push(setpoint);
        }
        Ok(output)
    }

    fn validate_limits(&self, setpoints: &[RotaryMitSetpoint]) -> Result<(), ArmControlError> {
        for ((setpoint, limits), binding) in setpoints
            .iter()
            .zip(self.config.limits())
            .zip(self.config.actuator_order().bindings())
        {
            for (field, value, maximum) in [
                (
                    "position_rad",
                    setpoint.position_rad(),
                    limits.maximum_position_rad(),
                ),
                (
                    "velocity_radps",
                    setpoint.velocity_radps(),
                    limits.maximum_velocity_radps(),
                ),
                (
                    "feed_forward_torque_nm",
                    setpoint.feed_forward_torque_nm(),
                    limits.maximum_torque_nm(),
                ),
            ] {
                if value.abs() > maximum {
                    return Err(ArmControlError::LimitExceeded {
                        actuator: binding.actuator_id(),
                        field,
                        value,
                        maximum,
                    });
                }
            }
        }
        Ok(())
    }

    fn validate_slew(
        &self,
        setpoints: &[RotaryMitSetpoint],
        now_monotonic_ns: u64,
    ) -> Result<(), ArmControlError> {
        let Some(previous_time) = self.last_admission_monotonic_ns else {
            return Ok(());
        };
        if now_monotonic_ns <= previous_time {
            return Err(ArmControlError::ExecutionClosed {
                field: "non_increasing_admission_time",
            });
        }
        let elapsed_s = (now_monotonic_ns - previous_time) as f64 * 1.0e-9;
        for (((setpoint, previous), limits), binding) in setpoints
            .iter()
            .zip(&self.last_setpoints)
            .zip(self.config.limits())
            .zip(self.config.actuator_order().bindings())
        {
            for (field, delta, maximum) in [
                (
                    "position_rad",
                    setpoint.position_rad() - previous.position_rad(),
                    limits.maximum_position_slew_radps(),
                ),
                (
                    "velocity_radps",
                    setpoint.velocity_radps() - previous.velocity_radps(),
                    limits.maximum_velocity_slew_radps2(),
                ),
                (
                    "feed_forward_torque_nm",
                    setpoint.feed_forward_torque_nm() - previous.feed_forward_torque_nm(),
                    limits.maximum_torque_slew_nmps(),
                ),
            ] {
                let actual = delta.abs() / elapsed_s;
                if actual > maximum {
                    return Err(ArmControlError::SlewExceeded {
                        actuator: binding.actuator_id(),
                        field,
                        actual,
                        maximum,
                    });
                }
            }
        }
        Ok(())
    }
}
