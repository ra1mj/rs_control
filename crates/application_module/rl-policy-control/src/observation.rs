//! Coherent morphology-bound observation encoding.

use arrayvec::ArrayVec;
use message_model::{
    ArmCoordinateKind, ArmEffortKind, ArmJointObservation, ClockState, DataQuality, Id128, SourceId,
};

use crate::{
    MAX_POLICY_ELEMENTS, ObservationElement, ObservationField, PolicyError, PolicyGeneration,
    PolicyIdentity, PolicyManifest,
};

/// Fixed-capacity normalized input plus the exact semantic snapshot used to
/// derive position-residual actions.
#[derive(Clone, Debug)]
pub struct EncodedPolicyObservation {
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    values: ArrayVec<f32, MAX_POLICY_ELEMENTS>,
    observation: ArmJointObservation,
}

impl EncodedPolicyObservation {
    /// Returns immutable policy identity.
    #[must_use]
    pub const fn identity(&self) -> PolicyIdentity {
        self.identity
    }

    /// Returns process-local policy generation.
    #[must_use]
    pub const fn generation(&self) -> PolicyGeneration {
        self.generation
    }

    /// Returns normalized input in exact manifest order.
    #[must_use]
    pub fn values(&self) -> &[f32] {
        &self.values
    }

    /// Returns the exact coherent semantic source snapshot.
    #[must_use]
    pub const fn observation(&self) -> &ArmJointObservation {
        &self.observation
    }
}

/// Stateful freshness/replay gate for one loaded policy.
pub struct PolicyObservationEncoder {
    manifest: PolicyManifest,
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    profile_id: message_model::ArmProfileId,
    group_id: message_model::KinematicGroupId,
    source_id: SourceId,
    boot_id: Id128,
    last_boot: Option<Id128>,
    last_sequence: Option<u64>,
}

impl PolicyObservationEncoder {
    pub(crate) fn new(
        manifest: PolicyManifest,
        identity: PolicyIdentity,
        generation: PolicyGeneration,
        profile_id: message_model::ArmProfileId,
        group_id: message_model::KinematicGroupId,
        source_id: SourceId,
        boot_id: Id128,
    ) -> Self {
        Self {
            manifest,
            identity,
            generation,
            profile_id,
            group_id,
            source_id,
            boot_id,
            last_boot: None,
            last_sequence: None,
        }
    }

    /// Validate one complete observation and deterministically normalize it.
    ///
    /// Missing fields, stale/future/replayed data and non-finite values fail;
    /// no numeric substitute is produced.
    pub fn encode(
        &mut self,
        observation: ArmJointObservation,
        now_monotonic_ns: u64,
    ) -> Result<EncodedPolicyObservation, PolicyError> {
        if observation.profile_id() != self.profile_id || observation.group_id() != self.group_id {
            return Err(PolicyError::Observation(
                "profile or group identity differs".into(),
            ));
        }
        if observation.samples().len() != self.manifest.joint_order.len()
            || observation
                .samples()
                .iter()
                .zip(&self.manifest.joint_order)
                .any(|(sample, expected)| sample.descriptor().joint_id().as_str() != expected)
        {
            return Err(PolicyError::Observation(
                "joint order differs from manifest".into(),
            ));
        }
        let header = observation.header();
        if header.source_id() != self.source_id
            || header.boot_id() != self.boot_id
            || header.clock_status().state != ClockState::Local
            || header.quality() != DataQuality::Valid
            || header.source_monotonic_ns() > now_monotonic_ns
            || header
                .source_expiration_ns()
                .is_none_or(|expires| now_monotonic_ns >= expires)
        {
            return Err(PolicyError::Observation(
                "observation is future, stale, degraded or outside local clock".into(),
            ));
        }
        if self.last_boot == Some(header.boot_id())
            && self
                .last_sequence
                .is_some_and(|previous| header.sequence() <= previous)
        {
            return Err(PolicyError::Observation(
                "observation sequence was replayed or reordered".into(),
            ));
        }

        let mut values = ArrayVec::new();
        for contract in &self.manifest.observations {
            let value = observation_value(&observation, contract)?;
            if !value.is_finite() || value < contract.minimum || value > contract.maximum {
                return Err(PolicyError::Observation(format!(
                    "{}.{:?} is non-finite or outside its hard range",
                    contract.joint, contract.field
                )));
            }
            let normalized = (value - contract.mean) * contract.inverse_std;
            let normalized = normalized as f32;
            if !normalized.is_finite() {
                return Err(PolicyError::Observation(
                    "normalized observation is not finite f32".into(),
                ));
            }
            values
                .try_push(normalized)
                .map_err(|_| PolicyError::Observation("observation capacity exceeded".into()))?;
        }
        self.last_boot = Some(header.boot_id());
        self.last_sequence = Some(header.sequence());
        Ok(EncodedPolicyObservation {
            identity: self.identity,
            generation: self.generation,
            values,
            observation,
        })
    }

    /// Invalidate replay history on a disarmed policy reload.
    pub fn reset(&mut self) {
        self.last_boot = None;
        self.last_sequence = None;
    }
}

fn observation_value(
    observation: &ArmJointObservation,
    contract: &ObservationElement,
) -> Result<f64, PolicyError> {
    let sample = observation
        .samples()
        .iter()
        .find(|sample| sample.descriptor().joint_id().as_str() == contract.joint)
        .ok_or_else(|| PolicyError::Observation("required joint is absent".into()))?;
    let (value, expected_unit) = match contract.field {
        ObservationField::Position => (
            sample.position().value_si(),
            match sample.position().kind() {
                ArmCoordinateKind::Angular => "rad",
                ArmCoordinateKind::Linear => "m",
            },
        ),
        ObservationField::Velocity => (
            sample.velocity().value_si(),
            match sample.velocity().kind() {
                ArmCoordinateKind::Angular => "rad_per_s",
                ArmCoordinateKind::Linear => "m_per_s",
            },
        ),
        ObservationField::Effort => (
            sample.effort().value_si(),
            match sample.effort().kind() {
                ArmEffortKind::Torque => "nm",
                ArmEffortKind::Force => "n",
            },
        ),
    };
    if contract.unit != expected_unit {
        return Err(PolicyError::Observation(
            "observation unit differs from semantic joint kind".into(),
        ));
    }
    Ok(value)
}
