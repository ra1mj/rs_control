//! Exact joint-indexed trajectory deviation monitoring.

use message_model::{ArmJointId, ArmJointVector, ArmPosition, ArmVelocity};

use super::{ArmTrajectory, TrajectorySampleBuffer};
use crate::ArmAlgorithmError;

/// Deviation channel that crossed its configured bound.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TrajectoryDeviationKind {
    /// Position error in radians or metres.
    Position,
    /// Velocity error in radians/second or metres/second.
    Velocity,
}

/// Exact trajectory-deviation evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryDeviation {
    kind: TrajectoryDeviationKind,
    joint: ArmJointId,
    index: usize,
    absolute_error_si: f64,
    maximum_error_si: f64,
}

impl TrajectoryDeviation {
    /// Returns the violated channel.
    #[must_use]
    pub const fn kind(&self) -> TrajectoryDeviationKind {
        self.kind
    }

    /// Returns the stable joint.
    #[must_use]
    pub const fn joint(&self) -> ArmJointId {
        self.joint
    }

    /// Returns the stable order index.
    #[must_use]
    pub const fn index(&self) -> usize {
        self.index
    }

    /// Returns the absolute observed error.
    #[must_use]
    pub const fn absolute_error_si(&self) -> f64 {
        self.absolute_error_si
    }

    /// Returns the configured absolute bound.
    #[must_use]
    pub const fn maximum_error_si(&self) -> f64 {
        self.maximum_error_si
    }
}

/// Per-joint absolute trajectory-monitor limits.
#[derive(Clone, Debug, PartialEq)]
pub struct TrajectoryDeviationLimits {
    maximum_position_error_si: Vec<f64>,
    maximum_velocity_error_si: Vec<f64>,
}

impl TrajectoryDeviationLimits {
    /// Validates exact dimensions and finite positive bounds.
    pub fn new(
        dof: usize,
        maximum_position_error_si: Vec<f64>,
        maximum_velocity_error_si: Vec<f64>,
    ) -> Result<Self, ArmAlgorithmError> {
        for (field, actual) in [
            (
                "trajectory_monitor.position_limits",
                maximum_position_error_si.len(),
            ),
            (
                "trajectory_monitor.velocity_limits",
                maximum_velocity_error_si.len(),
            ),
        ] {
            if actual != dof {
                return Err(ArmAlgorithmError::DimensionMismatch {
                    field,
                    expected: dof,
                    actual,
                });
            }
        }
        if maximum_position_error_si
            .iter()
            .chain(&maximum_velocity_error_si)
            .any(|value| !value.is_finite() || *value <= 0.0)
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "trajectory_monitor.limits",
            });
        }
        Ok(Self {
            maximum_position_error_si,
            maximum_velocity_error_si,
        })
    }
}

impl ArmTrajectory {
    /// Compares a stable-order observation against the interpolated trajectory.
    pub fn check_deviation(
        &self,
        time_from_start_ns: u64,
        observed_positions: &ArmJointVector<ArmPosition>,
        observed_velocities: &ArmJointVector<ArmVelocity>,
        limits: &TrajectoryDeviationLimits,
        scratch: &mut TrajectorySampleBuffer,
    ) -> Result<Option<TrajectoryDeviation>, ArmAlgorithmError> {
        if observed_positions.order() != self.order() || observed_velocities.order() != self.order()
        {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id(),
            });
        }
        if limits.maximum_position_error_si.len() != self.order().len()
            || limits.maximum_velocity_error_si.len() != self.order().len()
        {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "trajectory_monitor.limits",
                expected: self.order().len(),
                actual: limits.maximum_position_error_si.len(),
            });
        }
        self.sample_into(time_from_start_ns, scratch)?;
        for (index, descriptor) in self.order().joints().iter().enumerate() {
            if observed_positions.values()[index].kind() != descriptor.kind().coordinate_kind()
                || observed_velocities.values()[index].kind() != descriptor.kind().coordinate_kind()
            {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            let position_error = (observed_positions.values()[index].value_si()
                - scratch.positions()[index].value_si())
            .abs();
            if position_error > limits.maximum_position_error_si[index] {
                return Ok(Some(TrajectoryDeviation {
                    kind: TrajectoryDeviationKind::Position,
                    joint: descriptor.joint_id(),
                    index,
                    absolute_error_si: position_error,
                    maximum_error_si: limits.maximum_position_error_si[index],
                }));
            }
            let velocity_error = (observed_velocities.values()[index].value_si()
                - scratch.velocities()[index].value_si())
            .abs();
            if velocity_error > limits.maximum_velocity_error_si[index] {
                return Ok(Some(TrajectoryDeviation {
                    kind: TrajectoryDeviationKind::Velocity,
                    joint: descriptor.joint_id(),
                    index,
                    absolute_error_si: velocity_error,
                    maximum_error_si: limits.maximum_velocity_error_si[index],
                }));
            }
        }
        Ok(None)
    }
}
