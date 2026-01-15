//! Deterministic primitive-geometry grasp candidate generation.

use std::marker::PhantomData;

use message_model::{ArmJointVector, ArmPosition, AsciiId, MAX_ID_LEN};
use nalgebra::UnitQuaternion;

use crate::{
    ArmAlgorithmError, CollisionUnchecked,
    kinematics::{ArmPose, DampedLeastSquaresConfig, IkSolution, IkSolverKind, KinematicChain},
};

/// Maximum generated candidates per primitive object.
pub const MAX_GRASP_CANDIDATES: usize = 64;

/// Stable object identity.
pub type GraspObjectId = AsciiId<MAX_ID_LEN>;

/// Bounded primitive geometry in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PrimitiveShape {
    /// Box half extents.
    Box { half_extents_m: [f64; 3] },
    /// Sphere radius.
    Sphere { radius_m: f64 },
    /// Z-aligned cylinder.
    Cylinder { radius_m: f64, half_height_m: f64 },
}

impl PrimitiveShape {
    /// Validates finite positive dimensions.
    pub fn validate(self) -> Result<Self, ArmAlgorithmError> {
        let valid = match self {
            Self::Box { half_extents_m } => half_extents_m
                .iter()
                .all(|value| value.is_finite() && *value > 0.0),
            Self::Sphere { radius_m } => radius_m.is_finite() && radius_m > 0.0,
            Self::Cylinder {
                radius_m,
                half_height_m,
            } => {
                radius_m.is_finite()
                    && radius_m > 0.0
                    && half_height_m.is_finite()
                    && half_height_m > 0.0
            }
        };
        if valid {
            Ok(self)
        } else {
            Err(ArmAlgorithmError::NonFinite {
                field: "grasp.primitive_shape",
            })
        }
    }
}

/// One primitive object with finite world/base pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PrimitiveObject {
    id: GraspObjectId,
    pose: ArmPose,
    shape: PrimitiveShape,
}

impl PrimitiveObject {
    /// Validates primitive dimensions.
    pub fn new(
        id: GraspObjectId,
        pose: ArmPose,
        shape: PrimitiveShape,
    ) -> Result<Self, ArmAlgorithmError> {
        Ok(Self {
            id,
            pose,
            shape: shape.validate()?,
        })
    }

    /// Returns object identity.
    #[must_use]
    pub const fn id(self) -> GraspObjectId {
        self.id
    }

    /// Returns object pose.
    #[must_use]
    pub const fn pose(self) -> ArmPose {
        self.pose
    }

    /// Returns primitive geometry.
    #[must_use]
    pub const fn shape(self) -> PrimitiveShape {
        self.shape
    }
}

/// Bounded parallel-jaw/suction tool geometry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GripperDescription {
    maximum_opening_m: f64,
    finger_depth_m: f64,
    suction_radius_m: Option<f64>,
}

impl GripperDescription {
    /// Validates positive tool dimensions.
    pub fn new(
        maximum_opening_m: f64,
        finger_depth_m: f64,
        suction_radius_m: Option<f64>,
    ) -> Result<Self, ArmAlgorithmError> {
        if !maximum_opening_m.is_finite()
            || maximum_opening_m <= 0.0
            || !finger_depth_m.is_finite()
            || finger_depth_m <= 0.0
            || suction_radius_m.is_some_and(|value| !value.is_finite() || value <= 0.0)
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "grasp.gripper",
            });
        }
        Ok(Self {
            maximum_opening_m,
            finger_depth_m,
            suction_radius_m,
        })
    }
}

/// Deterministic grasp family.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GraspKind {
    /// Opposing-contact parallel-jaw candidate.
    Antipodal,
    /// Vertical approach.
    TopDown,
    /// Horizontal approach.
    Side,
    /// Surface-normal suction candidate.
    Suction,
}

/// One ranked candidate carrying collision typestate.
#[derive(Clone, Debug, PartialEq)]
pub struct GraspCandidate<Evidence> {
    object_id: GraspObjectId,
    kind: GraspKind,
    pose: ArmPose,
    approach_direction: [f64; 3],
    required_opening_m: f64,
    score: f64,
    _evidence: PhantomData<Evidence>,
}

impl<Evidence> GraspCandidate<Evidence> {
    /// Returns object identity.
    #[must_use]
    pub const fn object_id(&self) -> GraspObjectId {
        self.object_id
    }

    /// Returns candidate family.
    #[must_use]
    pub const fn kind(&self) -> GraspKind {
        self.kind
    }

    /// Returns tool pose.
    #[must_use]
    pub const fn pose(&self) -> ArmPose {
        self.pose
    }

    /// Returns unit approach direction.
    #[must_use]
    pub const fn approach_direction(&self) -> [f64; 3] {
        self.approach_direction
    }

    /// Returns required jaw opening, zero for suction.
    #[must_use]
    pub const fn required_opening_m(&self) -> f64 {
        self.required_opening_m
    }

    /// Returns deterministic quality in `[0,1]`.
    #[must_use]
    pub const fn score(&self) -> f64 {
        self.score
    }
}

/// Candidate and optional deterministic IK reachability evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct ReachableGrasp {
    candidate: GraspCandidate<CollisionUnchecked>,
    solution: Option<IkSolution>,
}

impl ReachableGrasp {
    /// Returns the collision-unchecked candidate.
    #[must_use]
    pub const fn candidate(&self) -> &GraspCandidate<CollisionUnchecked> {
        &self.candidate
    }

    /// Returns converged IK when reachable.
    #[must_use]
    pub const fn solution(&self) -> Option<&IkSolution> {
        self.solution.as_ref()
    }
}

/// Generates and deterministically ranks primitive grasp candidates.
pub fn generate_grasp_candidates(
    object: PrimitiveObject,
    gripper: GripperDescription,
) -> Result<Vec<GraspCandidate<CollisionUnchecked>>, ArmAlgorithmError> {
    let mut candidates = Vec::new();
    let center = object.pose.translation_m();
    let (opening_x, opening_y, top_offset) = match object.shape {
        PrimitiveShape::Box { half_extents_m } => (
            2.0 * half_extents_m[0],
            2.0 * half_extents_m[1],
            half_extents_m[2],
        ),
        PrimitiveShape::Sphere { radius_m } => (2.0 * radius_m, 2.0 * radius_m, radius_m),
        PrimitiveShape::Cylinder {
            radius_m,
            half_height_m,
        } => (2.0 * radius_m, 2.0 * radius_m, half_height_m),
    };
    if opening_x <= gripper.maximum_opening_m {
        candidates.push(candidate(
            object.id,
            GraspKind::Antipodal,
            [center[0], center[1], center[2]],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            opening_x,
            0.85,
        )?);
    }
    if opening_y <= gripper.maximum_opening_m {
        candidates.push(candidate(
            object.id,
            GraspKind::Antipodal,
            [center[0], center[1], center[2]],
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            opening_y,
            0.84,
        )?);
    }
    candidates.push(candidate(
        object.id,
        GraspKind::TopDown,
        [
            center[0],
            center[1],
            center[2] + top_offset + 0.5 * gripper.finger_depth_m,
        ],
        [0.0, 0.0, -1.0],
        [std::f64::consts::PI, 0.0, 0.0],
        opening_x.min(opening_y),
        0.8,
    )?);
    for (translation, approach, rpy) in [
        (
            [center[0] + opening_x * 0.5, center[1], center[2]],
            [-1.0, 0.0, 0.0],
            [0.0, std::f64::consts::FRAC_PI_2, 0.0],
        ),
        (
            [center[0], center[1] + opening_y * 0.5, center[2]],
            [0.0, -1.0, 0.0],
            [std::f64::consts::FRAC_PI_2, 0.0, 0.0],
        ),
    ] {
        candidates.push(candidate(
            object.id,
            GraspKind::Side,
            translation,
            approach,
            rpy,
            opening_x.min(opening_y),
            0.7,
        )?);
    }
    if let Some(suction_radius) = gripper.suction_radius_m {
        let surface_radius = opening_x.min(opening_y) * 0.5;
        if suction_radius <= surface_radius {
            candidates.push(candidate(
                object.id,
                GraspKind::Suction,
                [center[0], center[1], center[2] + top_offset],
                [0.0, 0.0, -1.0],
                [std::f64::consts::PI, 0.0, 0.0],
                0.0,
                0.9,
            )?);
        }
    }
    if candidates.len() > MAX_GRASP_CANDIDATES {
        return Err(ArmAlgorithmError::Capacity {
            field: "grasp.candidates",
            actual: candidates.len(),
            maximum: MAX_GRASP_CANDIDATES,
        });
    }
    candidates.sort_by(|left, right| {
        right
            .score
            .total_cmp(&left.score)
            .then_with(|| (left.kind as u8).cmp(&(right.kind as u8)))
    });
    Ok(candidates)
}

/// Evaluates deterministic DLS reachability without adding collision evidence.
pub fn evaluate_grasp_reachability(
    chain: KinematicChain<'_>,
    candidates: &[GraspCandidate<CollisionUnchecked>],
    initial: &ArmJointVector<ArmPosition>,
    config: DampedLeastSquaresConfig,
) -> Vec<ReachableGrasp> {
    candidates
        .iter()
        .cloned()
        .map(|candidate| {
            let solution = chain
                .solve_ik(
                    IkSolverKind::DampedLeastSquares,
                    candidate.pose,
                    initial,
                    config,
                )
                .ok();
            ReachableGrasp {
                candidate,
                solution,
            }
        })
        .collect()
}

/// Explicitly rejects collision filtering in this delivery.
pub fn require_grasp_collision_filter() -> Result<(), ArmAlgorithmError> {
    Err(ArmAlgorithmError::CollisionCapabilityUnavailable {
        surface: "grasp_filter",
    })
}

#[allow(clippy::too_many_arguments)]
fn candidate(
    object_id: GraspObjectId,
    kind: GraspKind,
    translation: [f64; 3],
    approach_direction: [f64; 3],
    rpy: [f64; 3],
    required_opening_m: f64,
    score: f64,
) -> Result<GraspCandidate<CollisionUnchecked>, ArmAlgorithmError> {
    let quaternion = UnitQuaternion::from_euler_angles(rpy[0], rpy[1], rpy[2]);
    let q = quaternion.quaternion();
    Ok(GraspCandidate {
        object_id,
        kind,
        pose: ArmPose::new(translation, [q.i, q.j, q.k, q.w])?,
        approach_direction,
        required_opening_m,
        score,
        _evidence: PhantomData,
    })
}
