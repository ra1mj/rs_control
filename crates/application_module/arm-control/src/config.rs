//! Immutable arm-control limits and fixed MIT gains.

use arm_model::{ArmModel, RotaryTransmission};
use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorBinding, ArmActuatorOrder, ArmJointOrder, ArmProfileId, Id128, KinematicGroupId,
    MAX_JOINTS, SourceId,
};

use crate::ArmControlError;

/// Per-actuator immutable magnitude, gain and slew bounds.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ActuatorControlLimits {
    binding: ArmActuatorBinding,
    maximum_position_rad: f64,
    maximum_velocity_radps: f64,
    maximum_torque_nm: f64,
    fixed_kp: f64,
    fixed_kd: f64,
    maximum_position_slew_radps: f64,
    maximum_velocity_slew_radps2: f64,
    maximum_torque_slew_nmps: f64,
}

impl ActuatorControlLimits {
    /// Validates one stable binding with positive finite bounds and fixed gains.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        binding: ArmActuatorBinding,
        maximum_position_rad: f64,
        maximum_velocity_radps: f64,
        maximum_torque_nm: f64,
        fixed_kp: f64,
        fixed_kd: f64,
        maximum_position_slew_radps: f64,
        maximum_velocity_slew_radps2: f64,
        maximum_torque_slew_nmps: f64,
    ) -> Result<Self, ArmControlError> {
        if [
            maximum_position_rad,
            maximum_velocity_radps,
            maximum_torque_nm,
            maximum_position_slew_radps,
            maximum_velocity_slew_radps2,
            maximum_torque_slew_nmps,
        ]
        .iter()
        .any(|value| !value.is_finite() || *value <= 0.0)
            || !fixed_kp.is_finite()
            || fixed_kp < 0.0
            || !fixed_kd.is_finite()
            || fixed_kd < 0.0
        {
            return Err(ArmControlError::InvalidConfiguration {
                reason: "actuator limits must be finite and positive and gains nonnegative",
            });
        }
        Ok(Self {
            binding,
            maximum_position_rad,
            maximum_velocity_radps,
            maximum_torque_nm,
            fixed_kp,
            fixed_kd,
            maximum_position_slew_radps,
            maximum_velocity_slew_radps2,
            maximum_torque_slew_nmps,
        })
    }

    /// Returns the stable joint-to-actuator binding.
    #[must_use]
    pub const fn binding(self) -> ArmActuatorBinding {
        self.binding
    }

    pub(crate) const fn maximum_position_rad(self) -> f64 {
        self.maximum_position_rad
    }

    pub(crate) const fn maximum_velocity_radps(self) -> f64 {
        self.maximum_velocity_radps
    }

    pub(crate) const fn maximum_torque_nm(self) -> f64 {
        self.maximum_torque_nm
    }

    /// Returns immutable proportional gain.
    #[must_use]
    pub const fn fixed_kp(self) -> f64 {
        self.fixed_kp
    }

    /// Returns immutable derivative gain.
    #[must_use]
    pub const fn fixed_kd(self) -> f64 {
        self.fixed_kd
    }

    pub(crate) const fn maximum_position_slew_radps(self) -> f64 {
        self.maximum_position_slew_radps
    }

    pub(crate) const fn maximum_velocity_slew_radps2(self) -> f64 {
        self.maximum_velocity_slew_radps2
    }

    pub(crate) const fn maximum_torque_slew_nmps(self) -> f64 {
        self.maximum_torque_slew_nmps
    }
}

/// Validated, morphology-bound admission configuration.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmControlConfig {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    observation_source: SourceId,
    observation_boot: Id128,
    joint_order: ArmJointOrder,
    actuator_order: ArmActuatorOrder,
    transmissions: ArrayVec<RotaryTransmission, MAX_JOINTS>,
    limits: ArrayVec<ActuatorControlLimits, MAX_JOINTS>,
}

impl ArmControlConfig {
    /// Compiles a complete actuator admission configuration for one group.
    pub fn new(
        model: &ArmModel,
        group_id: KinematicGroupId,
        generation: u64,
        observation_source: SourceId,
        observation_boot: Id128,
        limits: Vec<ActuatorControlLimits>,
    ) -> Result<Self, ArmControlError> {
        if generation == 0 {
            return Err(ArmControlError::InvalidConfiguration {
                reason: "generation must be nonzero",
            });
        }
        let group = model
            .group(group_id)
            .ok_or(ArmControlError::InvalidConfiguration {
                reason: "group is absent",
            })?;
        if limits.len() != group.joint_order().len() {
            return Err(ArmControlError::InvalidConfiguration {
                reason: "limit count differs from group DOF",
            });
        }
        let mut transmissions = ArrayVec::new();
        let mut ordered_limits = ArrayVec::new();
        let mut bindings = ArrayVec::new();
        for (descriptor, limit) in group.joint_order().joints().iter().zip(limits) {
            let transmission = *model.transmission(descriptor.joint_id()).ok_or(
                ArmControlError::InvalidConfiguration {
                    reason: "group has no complete rotary transmission mapping",
                },
            )?;
            if limit.binding != transmission.binding() {
                return Err(ArmControlError::IdentityMismatch {
                    field: "actuator_binding_order",
                });
            }
            transmissions.push(transmission);
            bindings.push(transmission.binding());
            ordered_limits.push(limit);
        }
        Ok(Self {
            profile_id: model.profile_id(),
            group_id,
            generation,
            observation_source,
            observation_boot,
            joint_order: group.joint_order().clone(),
            actuator_order: ArmActuatorOrder::new(bindings)?,
            transmissions,
            limits: ordered_limits,
        })
    }

    /// Returns immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns exact selected group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns exact proposal/model generation.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns compiled actuator order.
    #[must_use]
    pub const fn actuator_order(&self) -> &ArmActuatorOrder {
        &self.actuator_order
    }

    pub(crate) const fn joint_order(&self) -> &ArmJointOrder {
        &self.joint_order
    }

    pub(crate) const fn observation_source(&self) -> SourceId {
        self.observation_source
    }

    pub(crate) const fn observation_boot(&self) -> Id128 {
        self.observation_boot
    }

    pub(crate) fn transmissions(&self) -> &[RotaryTransmission] {
        &self.transmissions
    }

    pub(crate) fn limits(&self) -> &[ActuatorControlLimits] {
        &self.limits
    }
}
