//! Simulator-only attached payload description.

use thiserror::Error;

/// Validation failure for simulator-only payload inertia.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum AttachedPayloadError {
    /// Mass/CoG values are not finite or mass is not positive.
    #[error("payload mass/CoG must be finite and mass positive")]
    InvalidMassOrCog,
    /// Inertia contains NaN or infinity.
    #[error("payload inertia must be finite")]
    NonFiniteInertia,
    /// Symmetric inertia tensor is not positive definite.
    #[error("payload inertia must be positive definite")]
    NonPositiveDefiniteInertia,
}

/// A fixed payload attached to the selected group's terminal link.
///
/// This type is intentionally a dynamics/qualification input.  It is not
/// part of the adaptive estimator state (static gravity only needs mass and
/// first moments).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttachedPayloadInertia {
    mass_kg: f64,
    cog_m: [f64; 3],
    inertia_kg_m2: [f64; 6],
}

impl AttachedPayloadInertia {
    /// Validates a positive mass, finite tool-frame CoG and positive-definite
    /// symmetric inertia tensor `[ixx, ixy, ixz, iyy, iyz, izz]`.
    pub fn new(
        mass_kg: f64,
        cog_m: [f64; 3],
        inertia_kg_m2: [f64; 6],
    ) -> Result<Self, AttachedPayloadError> {
        if !mass_kg.is_finite() || mass_kg <= 0.0 || cog_m.iter().any(|v| !v.is_finite()) {
            return Err(AttachedPayloadError::InvalidMassOrCog);
        }
        if inertia_kg_m2.iter().any(|v| !v.is_finite()) {
            return Err(AttachedPayloadError::NonFiniteInertia);
        }
        let [ixx, ixy, ixz, iyy, iyz, izz] = inertia_kg_m2;
        // Sylvester criterion for a symmetric 3x3 tensor.
        let d1 = ixx;
        let d2 = ixx * iyy - ixy * ixy;
        let d3 = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - iyz * ixz)
            + ixz * (ixy * iyz - iyy * ixz);
        if d1 <= 0.0 || d2 <= 0.0 || d3 <= 0.0 {
            return Err(AttachedPayloadError::NonPositiveDefiniteInertia);
        }
        Ok(Self {
            mass_kg,
            cog_m,
            inertia_kg_m2,
        })
    }

    /// Payload mass in kilograms.
    #[must_use]
    pub const fn mass_kg(self) -> f64 {
        self.mass_kg
    }

    /// Tool-frame center of gravity in metres.
    #[must_use]
    pub const fn cog_m(self) -> [f64; 3] {
        self.cog_m
    }

    /// Payload inertia about its CoG in kg m².
    #[must_use]
    pub const fn inertia_kg_m2(self) -> [f64; 6] {
        self.inertia_kg_m2
    }
}
