//! Power-consistent joint-to-rotary-actuator conversion.

use message_model::{
    ArmActuatorBinding, ArmCoordinateKind, ArmEffort, ArmEffortKind, ArmJointKind, ArmPosition,
    ArmVelocity,
};

use crate::TransmissionError;

/// Validated one-to-one rotary actuator transmission.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RotaryTransmission {
    binding: ArmActuatorBinding,
    joint_kind: ArmJointKind,
    scale_rad_per_joint_unit: f64,
    offset_rad: f64,
}

impl RotaryTransmission {
    pub(crate) const fn new(
        binding: ArmActuatorBinding,
        joint_kind: ArmJointKind,
        scale_rad_per_joint_unit: f64,
        offset_rad: f64,
    ) -> Self {
        Self {
            binding,
            joint_kind,
            scale_rad_per_joint_unit,
            offset_rad,
        }
    }

    /// Returns the stable logical binding.
    #[must_use]
    pub const fn binding(self) -> ArmActuatorBinding {
        self.binding
    }

    /// Returns radians of actuator motion per radian or metre of joint motion.
    #[must_use]
    pub const fn scale_rad_per_joint_unit(self) -> f64 {
        self.scale_rad_per_joint_unit
    }

    /// Returns actuator position offset in radians.
    #[must_use]
    pub const fn offset_rad(self) -> f64 {
        self.offset_rad
    }

    /// Projects a tagged joint position into actuator radians.
    pub fn actuator_position_rad(self, value: ArmPosition) -> Result<f64, TransmissionError> {
        if value.kind() != self.joint_kind.coordinate_kind() {
            return Err(TransmissionError::UnitMismatch);
        }
        finite(
            self.scale_rad_per_joint_unit
                .mul_add(value.value_si(), self.offset_rad),
        )
    }

    /// Projects tagged joint velocity into actuator radians/second.
    pub fn actuator_velocity_radps(self, value: ArmVelocity) -> Result<f64, TransmissionError> {
        if value.kind() != self.joint_kind.coordinate_kind() {
            return Err(TransmissionError::UnitMismatch);
        }
        finite(self.scale_rad_per_joint_unit * value.value_si())
    }

    /// Projects joint effort into actuator torque while preserving power.
    pub fn actuator_torque_nm(self, value: ArmEffort) -> Result<f64, TransmissionError> {
        let expected = match self.joint_kind.coordinate_kind() {
            ArmCoordinateKind::Angular => ArmEffortKind::Torque,
            ArmCoordinateKind::Linear => ArmEffortKind::Force,
        };
        if value.kind() != expected {
            return Err(TransmissionError::UnitMismatch);
        }
        finite(value.value_si() / self.scale_rad_per_joint_unit)
    }

    /// Converts actuator torque back to joint effort while preserving power.
    pub fn joint_effort(self, torque_nm: f64) -> Result<ArmEffort, TransmissionError> {
        let value = finite(finite(torque_nm)? * self.scale_rad_per_joint_unit)?;
        match self.joint_kind {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                ArmEffort::torque_nm(value).map_err(|_| TransmissionError::NonFinite)
            }
            ArmJointKind::Prismatic => {
                ArmEffort::force_n(value).map_err(|_| TransmissionError::NonFinite)
            }
        }
    }

    /// Converts actuator position back into a tagged joint coordinate.
    pub fn joint_position(self, position_rad: f64) -> Result<ArmPosition, TransmissionError> {
        let value =
            finite((finite(position_rad)? - self.offset_rad) / self.scale_rad_per_joint_unit)?;
        match self.joint_kind.coordinate_kind() {
            ArmCoordinateKind::Angular => {
                ArmPosition::angular_rad(value).map_err(|_| TransmissionError::NonFinite)
            }
            ArmCoordinateKind::Linear => {
                ArmPosition::linear_m(value).map_err(|_| TransmissionError::NonFinite)
            }
        }
    }

    /// Converts actuator velocity back into a tagged joint velocity.
    pub fn joint_velocity(self, velocity_radps: f64) -> Result<ArmVelocity, TransmissionError> {
        let value = finite(finite(velocity_radps)? / self.scale_rad_per_joint_unit)?;
        match self.joint_kind.coordinate_kind() {
            ArmCoordinateKind::Angular => {
                ArmVelocity::angular_radps(value).map_err(|_| TransmissionError::NonFinite)
            }
            ArmCoordinateKind::Linear => {
                ArmVelocity::linear_mps(value).map_err(|_| TransmissionError::NonFinite)
            }
        }
    }
}

fn finite(value: f64) -> Result<f64, TransmissionError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(TransmissionError::NonFinite)
    }
}

#[cfg(test)]
mod tests {
    use message_model::{
        ArmActuatorBinding, ArmEffort, ArmJointKind, ArmPosition, ArmVelocity, AsciiId,
    };

    use super::RotaryTransmission;

    fn binding() -> ArmActuatorBinding {
        ArmActuatorBinding::new(
            AsciiId::parse("slide").unwrap(),
            AsciiId::parse("slide_motor").unwrap(),
        )
    }

    #[test]
    fn prismatic_projection_is_power_consistent() {
        let transmission = RotaryTransmission::new(binding(), ArmJointKind::Prismatic, 20.0, 0.1);
        let actuator_position = transmission
            .actuator_position_rad(ArmPosition::linear_m(0.2).unwrap())
            .unwrap();
        assert!((actuator_position - 4.1).abs() < 1.0e-12);
        assert_eq!(
            transmission
                .actuator_velocity_radps(ArmVelocity::linear_mps(0.5).unwrap())
                .unwrap(),
            10.0
        );
        assert_eq!(
            transmission
                .actuator_torque_nm(ArmEffort::force_n(40.0).unwrap())
                .unwrap(),
            2.0
        );
        assert_eq!(transmission.joint_effort(2.0).unwrap().value_si(), 40.0);
        let joint_position = transmission.joint_position(4.1).unwrap().value_si();
        assert!((joint_position - 0.2).abs() < 1.0e-12);
    }

    #[test]
    fn projection_rejects_wrong_units_and_nonfinite_values() {
        let transmission = RotaryTransmission::new(binding(), ArmJointKind::Prismatic, 20.0, 0.0);
        assert!(
            transmission
                .actuator_position_rad(ArmPosition::angular_rad(0.0).unwrap())
                .is_err()
        );
        assert!(transmission.joint_velocity(f64::NAN).is_err());
    }
}
