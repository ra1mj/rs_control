//! Bounded semantic action decoding into project-owned proposals.

use arm_algorithms::CollisionUnchecked;
use arm_control::{ArmControlConfig, ArmControlIntent};
use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorVector, ArmControlProposal, ArmCoordinateKind, ArmJointKind, ArmJointObservation,
    ArmJointVector, ArmPosition, ArmProposalContext, ArmVelocity, CommandEnvelope, MAX_JOINTS,
    RotaryMitSetpoint, RotaryTorqueNm,
};

use crate::{ActionManifest, PolicyActionKind, PolicyError, PolicyGeneration, PolicyIdentity};

/// Valid policy proposal plus projection evidence.
#[derive(Clone, Debug)]
pub struct PolicyDecision {
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    request_sequence: u64,
    proposal: ArmControlProposal,
    saturated: bool,
}

impl PolicyDecision {
    /// Returns immutable artifact identity.
    #[must_use]
    pub const fn identity(&self) -> PolicyIdentity {
        self.identity
    }

    /// Returns process-local load generation.
    #[must_use]
    pub const fn generation(&self) -> PolicyGeneration {
        self.generation
    }

    /// Returns worker request identity.
    #[must_use]
    pub const fn request_sequence(&self) -> u64 {
        self.request_sequence
    }

    /// Returns a semantic proposal with no lease or hardware authority.
    #[must_use]
    pub const fn proposal(&self) -> &ArmControlProposal {
        &self.proposal
    }

    /// Returns whether command projection or slew limiting changed an output.
    #[must_use]
    pub const fn saturated(&self) -> bool {
        self.saturated
    }

    /// Attach caller-owned authority metadata while preserving explicit
    /// collision-unchecked typestate.
    #[must_use]
    pub fn into_collision_unchecked(
        self,
        envelope: CommandEnvelope,
    ) -> ArmControlIntent<CollisionUnchecked> {
        ArmControlIntent::collision_unchecked(envelope, self.proposal)
    }
}

/// Stateful per-generation output projection and slew limiter.
pub(crate) struct ActionDecoder {
    action: ActionManifest,
    control: ArmControlConfig,
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    previous: ArrayVec<f64, { crate::MAX_POLICY_ELEMENTS }>,
    previous_time_ns: Option<u64>,
}

impl ActionDecoder {
    pub(crate) fn new(
        action: ActionManifest,
        control: ArmControlConfig,
        identity: PolicyIdentity,
        generation: PolicyGeneration,
    ) -> Self {
        Self {
            action,
            control,
            identity,
            generation,
            previous: ArrayVec::new(),
            previous_time_ns: None,
        }
    }

    pub(crate) fn decode(
        &mut self,
        request_sequence: u64,
        raw: &[f32],
        observation: &ArmJointObservation,
        accepted_time_ns: u64,
    ) -> Result<PolicyDecision, PolicyError> {
        if raw.len() != self.action.scale.len() {
            self.reset();
            return Err(PolicyError::Inference(
                "output element count differs from manifest".into(),
            ));
        }
        let mut projected = ArrayVec::new();
        let mut saturated = false;
        let elapsed_s = self
            .previous_time_ns
            .and_then(|previous| accepted_time_ns.checked_sub(previous))
            .map(|elapsed| elapsed as f64 * 1.0e-9);
        if self.previous_time_ns.is_some() && elapsed_s.is_none_or(|elapsed| elapsed <= 0.0) {
            self.reset();
            return Err(PolicyError::Inference(
                "action acceptance time did not advance".into(),
            ));
        }
        for (index, raw) in raw.iter().copied().enumerate() {
            let decoded = f64::from(raw) * self.action.scale[index] + self.action.offset[index];
            if !decoded.is_finite()
                || decoded < self.action.hard_minimum[index]
                || decoded > self.action.hard_maximum[index]
            {
                self.reset();
                return Err(PolicyError::Inference(
                    "output is non-finite or outside its hard bound".into(),
                ));
            }
            let mut value = decoded.clamp(
                self.action.command_minimum[index],
                self.action.command_maximum[index],
            );
            saturated |= value != decoded;
            if let (Some(elapsed_s), Some(previous)) = (elapsed_s, self.previous.get(index)) {
                let maximum_delta = self.action.slew_per_second[index] * elapsed_s;
                let slewed = value.clamp(previous - maximum_delta, previous + maximum_delta);
                saturated |= slewed != value;
                value = slewed;
            }
            projected
                .try_push(value)
                .map_err(|_| PolicyError::Inference("action capacity exceeded".into()))?;
        }

        let proposal = self.build_proposal(&projected, observation)?;
        self.previous.clone_from(&projected);
        self.previous_time_ns = Some(accepted_time_ns);
        Ok(PolicyDecision {
            identity: self.identity,
            generation: self.generation,
            request_sequence,
            proposal,
            saturated,
        })
    }

    pub(crate) fn reset(&mut self) {
        self.previous.clear();
        self.previous_time_ns = None;
    }

    fn build_proposal(
        &self,
        output: &[f64],
        observation: &ArmJointObservation,
    ) -> Result<ArmControlProposal, PolicyError> {
        let context = ArmProposalContext::new(
            self.control.profile_id(),
            self.control.group_id(),
            self.control.generation(),
        )?;
        match self.action.kind {
            PolicyActionKind::AbsolutePosition | PolicyActionKind::PositionResidual => {
                let mut values = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
                for (&value, descriptor) in output.iter().zip(self.control.joint_order().joints()) {
                    let position = match descriptor.kind().coordinate_kind() {
                        ArmCoordinateKind::Angular => ArmPosition::angular_rad(value)?,
                        ArmCoordinateKind::Linear => ArmPosition::linear_m(value)?,
                    };
                    values
                        .try_push(position)
                        .map_err(|_| PolicyError::Inference("joint capacity exceeded".into()))?;
                }
                let vector = ArmJointVector::new(self.control.joint_order().clone(), values)?;
                if self.action.kind == PolicyActionKind::AbsolutePosition {
                    ArmControlProposal::absolute_position(context, vector).map_err(Into::into)
                } else {
                    // The residual is interpreted later against this exact
                    // fresh observation by arm-control admission.
                    if observation.samples().len() != output.len() {
                        return Err(PolicyError::Inference(
                            "residual observation dimension differs".into(),
                        ));
                    }
                    ArmControlProposal::position_residual(context, vector).map_err(Into::into)
                }
            }
            PolicyActionKind::JointVelocity => {
                let mut values = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
                for (&value, descriptor) in output.iter().zip(self.control.joint_order().joints()) {
                    let velocity = match descriptor.kind().coordinate_kind() {
                        ArmCoordinateKind::Angular => ArmVelocity::angular_radps(value)?,
                        ArmCoordinateKind::Linear => ArmVelocity::linear_mps(value)?,
                    };
                    values
                        .try_push(velocity)
                        .map_err(|_| PolicyError::Inference("joint capacity exceeded".into()))?;
                }
                let vector = ArmJointVector::new(self.control.joint_order().clone(), values)?;
                ArmControlProposal::velocity(context, vector).map_err(Into::into)
            }
            PolicyActionKind::DirectRotaryTorque => {
                let mut values = ArrayVec::<RotaryTorqueNm, MAX_JOINTS>::new();
                for &value in output {
                    values
                        .try_push(RotaryTorqueNm::new(value)?)
                        .map_err(|_| PolicyError::Inference("actuator capacity exceeded".into()))?;
                }
                let vector = ArmActuatorVector::new(self.control.actuator_order().clone(), values)?;
                Ok(ArmControlProposal::direct_rotary_torque(context, vector))
            }
            PolicyActionKind::Mit => {
                if self
                    .control
                    .joint_order()
                    .joints()
                    .iter()
                    .any(|descriptor| descriptor.kind() == ArmJointKind::Prismatic)
                {
                    return Err(PolicyError::Binding(
                        "MIT requires a complete rotary group".into(),
                    ));
                }
                let mut values = ArrayVec::<RotaryMitSetpoint, MAX_JOINTS>::new();
                for index in 0..self.control.actuator_order().len() {
                    values
                        .try_push(RotaryMitSetpoint::new(
                            output[index * 3],
                            output[index * 3 + 1],
                            self.action.fixed_kp[index],
                            self.action.fixed_kd[index],
                            output[index * 3 + 2],
                        )?)
                        .map_err(|_| PolicyError::Inference("actuator capacity exceeded".into()))?;
                }
                let vector = ArmActuatorVector::new(self.control.actuator_order().clone(), values)?;
                Ok(ArmControlProposal::mit(context, vector))
            }
        }
    }
}
