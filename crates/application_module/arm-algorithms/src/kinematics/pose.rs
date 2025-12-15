//! Finite public Cartesian pose values.

use arm_model::ArmLinkId;
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};

use crate::ArmAlgorithmError;

const UNIT_QUATERNION_TOLERANCE: f64 = 1.0e-9;

/// Finite Cartesian pose with translation in metres and quaternion `[x,y,z,w]`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmPose {
    translation_m: [f64; 3],
    quaternion_xyzw: [f64; 4],
}

impl ArmPose {
    /// Validates a finite, normalized Cartesian pose.
    pub fn new(
        translation_m: [f64; 3],
        quaternion_xyzw: [f64; 4],
    ) -> Result<Self, ArmAlgorithmError> {
        if translation_m.iter().any(|value| !value.is_finite())
            || quaternion_xyzw.iter().any(|value| !value.is_finite())
        {
            return Err(ArmAlgorithmError::NonFinite { field: "arm.pose" });
        }
        let norm_squared = quaternion_xyzw
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        if (norm_squared - 1.0).abs() > UNIT_QUATERNION_TOLERANCE {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "pose quaternion is not normalized",
            });
        }
        Ok(Self {
            translation_m,
            quaternion_xyzw,
        })
    }

    /// Returns the identity pose.
    #[must_use]
    pub const fn identity() -> Self {
        Self {
            translation_m: [0.0; 3],
            quaternion_xyzw: [0.0, 0.0, 0.0, 1.0],
        }
    }

    /// Returns translation in metres.
    #[must_use]
    pub const fn translation_m(self) -> [f64; 3] {
        self.translation_m
    }

    /// Returns the normalized quaternion in `[x,y,z,w]` order.
    #[must_use]
    pub const fn quaternion_xyzw(self) -> [f64; 4] {
        self.quaternion_xyzw
    }

    pub(crate) fn from_isometry(pose: &Isometry3<f64>) -> Result<Self, ArmAlgorithmError> {
        let translation = pose.translation.vector;
        let quaternion = pose.rotation.quaternion();
        Self::new(
            [translation.x, translation.y, translation.z],
            [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
        )
    }

    pub(crate) fn to_isometry(self) -> Isometry3<f64> {
        let [x, y, z, w] = self.quaternion_xyzw;
        Isometry3::from_parts(
            Translation3::new(
                self.translation_m[0],
                self.translation_m[1],
                self.translation_m[2],
            ),
            UnitQuaternion::new_normalize(Quaternion::new(w, x, y, z)),
        )
    }
}

/// Pose of one model link in the selected group's base frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmLinkPose {
    link_id: ArmLinkId,
    pose: ArmPose,
}

impl ArmLinkPose {
    pub(crate) const fn new(link_id: ArmLinkId, pose: ArmPose) -> Self {
        Self { link_id, pose }
    }

    /// Returns the stable link identity.
    #[must_use]
    pub const fn link_id(self) -> ArmLinkId {
        self.link_id
    }

    /// Returns the link pose in the group base frame.
    #[must_use]
    pub const fn pose(self) -> ArmPose {
        self.pose
    }
}
