//! Validated numeric values shared by protocol projections.

use crate::SemanticError;

/// Maximum accepted absolute error from unit quaternion norm.
pub const QUATERNION_NORM_TOLERANCE: f64 = 1.0e-6;
const COVARIANCE_SYMMETRY_TOLERANCE: f64 = 1.0e-9;

/// Finite three-dimensional vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3 {
    x: f64,
    y: f64,
    z: f64,
}

impl Vector3 {
    /// Constructs a vector after rejecting non-finite components.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self, SemanticError> {
        require_finite(x, "vector.x")?;
        require_finite(y, "vector.y")?;
        require_finite(z, "vector.z")?;
        Ok(Self { x, y, z })
    }

    /// Returns the x component.
    #[must_use]
    pub const fn x(self) -> f64 {
        self.x
    }

    /// Returns the y component.
    #[must_use]
    pub const fn y(self) -> f64 {
        self.y
    }

    /// Returns the z component.
    #[must_use]
    pub const fn z(self) -> f64 {
        self.z
    }
}

/// Finite unit quaternion in x, y, z, w ordering.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitQuaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

impl UnitQuaternion {
    /// Constructs a unit quaternion without silently normalizing input.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Result<Self, SemanticError> {
        require_finite(x, "quaternion.x")?;
        require_finite(y, "quaternion.y")?;
        require_finite(z, "quaternion.z")?;
        require_finite(w, "quaternion.w")?;
        let norm_squared = x.mul_add(x, y.mul_add(y, z.mul_add(z, w * w)));
        if (norm_squared - 1.0).abs() > QUATERNION_NORM_TOLERANCE {
            return Err(SemanticError::InvalidQuaternion);
        }
        Ok(Self { x, y, z, w })
    }

    /// Returns components in canonical x, y, z, w ordering.
    #[must_use]
    pub const fn components(self) -> [f64; 4] {
        [self.x, self.y, self.z, self.w]
    }
}

/// Validated row-major 3-by-3 covariance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Covariance3([f64; 9]);

impl Covariance3 {
    /// Validates finiteness, symmetry and nonnegative diagonal entries.
    pub fn new(values: [f64; 9]) -> Result<Self, SemanticError> {
        validate_covariance(&values, 3)?;
        Ok(Self(values))
    }

    /// Returns row-major values.
    #[must_use]
    pub const fn values(&self) -> &[f64; 9] {
        &self.0
    }

    /// Returns whether every element is exactly zero (the ROS unknown sentinel).
    #[must_use]
    pub fn is_all_zero(&self) -> bool {
        self.0.iter().all(|value| *value == 0.0)
    }
}

/// Validated row-major 6-by-6 covariance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Covariance6([f64; 36]);

impl Covariance6 {
    /// Validates finiteness, symmetry and nonnegative diagonal entries.
    pub fn new(values: [f64; 36]) -> Result<Self, SemanticError> {
        validate_covariance(&values, 6)?;
        Ok(Self(values))
    }

    /// Returns row-major values.
    #[must_use]
    pub const fn values(&self) -> &[f64; 36] {
        &self.0
    }
}

/// Finite 3D pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose3 {
    /// Position in metres.
    pub position_m: Vector3,
    /// Unit orientation.
    pub orientation: UnitQuaternion,
}

/// Finite 3D twist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Twist3 {
    /// Linear velocity in metres per second.
    pub linear_mps: Vector3,
    /// Angular velocity in radians per second.
    pub angular_radps: Vector3,
}

/// Finite planar twist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Twist2 {
    vx_mps: f64,
    vy_mps: f64,
    yaw_rate_radps: f64,
}

impl Twist2 {
    /// Constructs a planar twist after rejecting non-finite components.
    pub fn new(vx_mps: f64, vy_mps: f64, yaw_rate_radps: f64) -> Result<Self, SemanticError> {
        require_finite(vx_mps, "twist.vx_mps")?;
        require_finite(vy_mps, "twist.vy_mps")?;
        require_finite(yaw_rate_radps, "twist.yaw_rate_radps")?;
        Ok(Self {
            vx_mps,
            vy_mps,
            yaw_rate_radps,
        })
    }

    /// Returns components in x, y, yaw order.
    #[must_use]
    pub const fn components(self) -> [f64; 3] {
        [self.vx_mps, self.vy_mps, self.yaw_rate_radps]
    }
}

pub(crate) fn require_finite(value: f64, field: &'static str) -> Result<(), SemanticError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(SemanticError::NonFinite { field })
    }
}

fn validate_covariance(values: &[f64], width: usize) -> Result<(), SemanticError> {
    for value in values {
        require_finite(*value, "covariance")?;
    }
    for row in 0..width {
        if values[row * width + row] < 0.0 {
            return Err(SemanticError::NegativeCovarianceDiagonal { index: row });
        }
        for column in (row + 1)..width {
            let left = values[row * width + column];
            let right = values[column * width + row];
            if (left - right).abs() > COVARIANCE_SYMMETRY_TOLERANCE {
                return Err(SemanticError::AsymmetricCovariance { row, column });
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{Covariance3, UnitQuaternion, Vector3};

    #[test]
    fn numeric_values_reject_nonfinite_and_invalid_quaternion() {
        assert!(Vector3::new(f64::NAN, 0.0, 0.0).is_err());
        assert!(UnitQuaternion::new(0.0, 0.0, 0.0, 2.0).is_err());
        assert!(UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).is_ok());
    }

    #[test]
    fn covariance_rejects_asymmetry_and_negative_variance() {
        let mut asymmetric = [0.0; 9];
        asymmetric[1] = 1.0;
        assert!(Covariance3::new(asymmetric).is_err());

        let mut negative = [0.0; 9];
        negative[0] = -1.0;
        assert!(Covariance3::new(negative).is_err());
    }
}
