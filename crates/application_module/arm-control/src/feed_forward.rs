//! Joint-space feed-forward proposal composition.

use arrayvec::ArrayVec;
use message_model::{
    ArmActuatorVector, ArmControlProposal, ArmEffort, ArmJointVector, ArmPosition,
    ArmProposalContext, ArmVelocity, MAX_JOINTS, RotaryMitSetpoint, RotaryTorqueNm,
};

use crate::{ArmControlConfig, ArmControlError};

/// Builds an ordinary direct-torque proposal from stable-order joint effort.
/// The result still requires the normal lease/lifecycle/freshness/limit/slew
/// admission path.
pub fn direct_effort_proposal(
    config: &ArmControlConfig,
    effort: &ArmJointVector<ArmEffort>,
) -> Result<ArmControlProposal, ArmControlError> {
    require_joint_order(config, effort.order())?;
    let context = proposal_context(config)?;
    let mut torques = ArrayVec::<RotaryTorqueNm, MAX_JOINTS>::new();
    for (transmission, value) in config.transmissions().iter().zip(effort.values()) {
        torques.push(RotaryTorqueNm::new(
            transmission.actuator_torque_nm(*value)?,
        )?);
    }
    let vector = ArmActuatorVector::new(config.actuator_order().clone(), torques)?;
    Ok(ArmControlProposal::direct_rotary_torque(context, vector))
}

/// Builds a fixed-gain MIT proposal from joint targets plus joint-space
/// feed-forward effort.
pub fn mit_feed_forward_proposal(
    config: &ArmControlConfig,
    positions: &ArmJointVector<ArmPosition>,
    velocities: &ArmJointVector<ArmVelocity>,
    effort: &ArmJointVector<ArmEffort>,
) -> Result<ArmControlProposal, ArmControlError> {
    require_joint_order(config, positions.order())?;
    require_joint_order(config, velocities.order())?;
    require_joint_order(config, effort.order())?;
    let context = proposal_context(config)?;
    let mut setpoints = ArrayVec::<RotaryMitSetpoint, MAX_JOINTS>::new();
    for ((((transmission, limits), position), velocity), effort) in config
        .transmissions()
        .iter()
        .zip(config.limits())
        .zip(positions.values())
        .zip(velocities.values())
        .zip(effort.values())
    {
        setpoints.push(RotaryMitSetpoint::new(
            transmission.actuator_position_rad(*position)?,
            transmission.actuator_velocity_radps(*velocity)?,
            limits.fixed_kp(),
            limits.fixed_kd(),
            transmission.actuator_torque_nm(*effort)?,
        )?);
    }
    let vector = ArmActuatorVector::new(config.actuator_order().clone(), setpoints)?;
    Ok(ArmControlProposal::mit(context, vector))
}

fn proposal_context(config: &ArmControlConfig) -> Result<ArmProposalContext, ArmControlError> {
    Ok(ArmProposalContext::new(
        config.profile_id(),
        config.group_id(),
        config.generation(),
    )?)
}

fn require_joint_order(
    config: &ArmControlConfig,
    actual: &message_model::ArmJointOrder,
) -> Result<(), ArmControlError> {
    if actual != config.joint_order() {
        return Err(ArmControlError::IdentityMismatch {
            field: "feed_forward.joint_order",
        });
    }
    Ok(())
}
