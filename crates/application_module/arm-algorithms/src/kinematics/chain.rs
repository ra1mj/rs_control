//! Validated open-chain extraction and forward kinematics.

use std::collections::{BTreeMap, BTreeSet};

use arm_model::{ArmJoint, ArmModel, KinematicGroup, ModelJointKind};
use arrayvec::ArrayVec;
use message_model::{
    ArmJointId, ArmJointKind, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId,
    MAX_JOINTS,
};
use nalgebra::{Isometry3, Translation3, Unit, UnitQuaternion, Vector3};

use super::{ArmLinkPose, ArmPose};
use crate::ArmAlgorithmError;

/// Forward-kinematics result in the selected group's base frame.
#[derive(Clone, Debug, PartialEq)]
pub struct ForwardKinematics {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    link_poses: Vec<ArmLinkPose>,
    tool_pose: ArmPose,
}

impl ForwardKinematics {
    /// Returns the immutable model identity used for evaluation.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the selected group.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns base, intermediate and tip link poses in path order.
    #[must_use]
    pub fn link_poses(&self) -> &[ArmLinkPose] {
        &self.link_poses
    }

    /// Returns the tip/tool pose.
    #[must_use]
    pub const fn tool_pose(&self) -> ArmPose {
        self.tool_pose
    }
}

/// Immutable view of one compiled base-to-tip group.
#[derive(Clone, Copy, Debug)]
pub struct KinematicChain<'model> {
    model: &'model ArmModel,
    group: &'model KinematicGroup,
}

impl<'model> KinematicChain<'model> {
    /// Selects one named group without assuming a global arm.
    pub fn new(
        model: &'model ArmModel,
        group_id: KinematicGroupId,
    ) -> Result<Self, ArmAlgorithmError> {
        let group = model
            .group(group_id)
            .ok_or(ArmAlgorithmError::UnknownGroup { group: group_id })?;
        Ok(Self { model, group })
    }

    /// Returns the compiled model.
    #[must_use]
    pub const fn model(self) -> &'model ArmModel {
        self.model
    }

    /// Returns the selected group.
    #[must_use]
    pub const fn group(self) -> &'model KinematicGroup {
        self.group
    }

    /// Returns the exact independent degree of freedom.
    #[must_use]
    pub fn dof(self) -> usize {
        self.group.joint_order().len()
    }

    /// Builds a tagged stable-order position vector from raw SI coordinates.
    pub fn positions_from_values(
        self,
        values_si: &[f64],
    ) -> Result<ArmJointVector<ArmPosition>, ArmAlgorithmError> {
        if values_si.len() != self.dof() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "joint_positions",
                expected: self.dof(),
                actual: values_si.len(),
            });
        }
        let mut positions = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
        for (descriptor, value) in self
            .group
            .joint_order()
            .joints()
            .iter()
            .zip(values_si.iter().copied())
        {
            let position = match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmPosition::angular_rad(value)
                }
                ArmJointKind::Prismatic => ArmPosition::linear_m(value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "joint_positions",
            })?;
            positions.push(position);
        }
        ArmJointVector::new(self.group.joint_order().clone(), positions).map_err(|_| {
            ArmAlgorithmError::InvalidModel {
                reason: "compiled joint order exceeds project bound",
            }
        })
    }

    /// Evaluates base-to-tip and all-link forward kinematics.
    pub fn forward(
        self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<ForwardKinematics, ArmAlgorithmError> {
        let evaluated = self.evaluate(positions)?;
        let link_poses = evaluated
            .link_poses
            .iter()
            .map(|(link_id, transform)| {
                Ok(ArmLinkPose::new(
                    *link_id,
                    ArmPose::from_isometry(transform)?,
                ))
            })
            .collect::<Result<Vec<_>, ArmAlgorithmError>>()?;
        let tool_pose = ArmPose::from_isometry(&evaluated.tool_transform)?;
        Ok(ForwardKinematics {
            profile_id: self.model.profile_id(),
            group_id: self.group.id(),
            link_poses,
            tool_pose,
        })
    }

    /// Evaluates a bounded caller-owned batch in stable input order.
    pub fn forward_batch(
        self,
        batch: &[ArmJointVector<ArmPosition>],
    ) -> Result<Vec<ForwardKinematics>, ArmAlgorithmError> {
        batch
            .iter()
            .map(|positions| self.forward(positions))
            .collect()
    }

    pub(crate) fn values(
        self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<Vec<f64>, ArmAlgorithmError> {
        self.validate_positions(positions)?;
        Ok(positions
            .values()
            .iter()
            .map(|position| position.value_si())
            .collect())
    }

    pub(crate) fn evaluate(
        self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<EvaluatedChain, ArmAlgorithmError> {
        self.validate_positions(positions)?;
        let independent = self
            .group
            .joint_order()
            .joints()
            .iter()
            .zip(positions.values())
            .enumerate()
            .map(|(column, (descriptor, value))| {
                (
                    descriptor.joint_id(),
                    IndependentCoordinate {
                        value: value.value_si(),
                        column,
                    },
                )
            })
            .collect::<BTreeMap<_, _>>();

        let mut current_link = self.group.base_link();
        let mut transform = Isometry3::identity();
        let mut link_poses = vec![(current_link, transform)];
        let mut joint_frames = Vec::with_capacity(self.group.path_joint_indices().len());

        for &joint_index in self.group.path_joint_indices() {
            let joint =
                self.model
                    .joints()
                    .get(joint_index)
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "group path joint index is out of bounds",
                    })?;
            if joint.parent_link() != current_link {
                return Err(ArmAlgorithmError::InvalidModel {
                    reason: "group path is not contiguous",
                });
            }
            let origin = Isometry3::from_parts(
                Translation3::from(Vector3::from(joint.origin_xyz_m())),
                UnitQuaternion::from_euler_angles(
                    joint.origin_rpy_rad()[0],
                    joint.origin_rpy_rad()[1],
                    joint.origin_rpy_rad()[2],
                ),
            );
            let joint_frame = transform * origin;
            let resolved = resolve_coordinate(self.model, joint, &independent)?;
            if let Some(coordinate) = resolved {
                validate_joint_limit(joint, coordinate.value)?;
                joint_frames.push(EvaluatedJointFrame {
                    joint_index,
                    frame_before_motion: joint_frame,
                    source_column: coordinate.source_column,
                    source_multiplier: coordinate.source_multiplier,
                    kind: joint
                        .kind()
                        .active()
                        .ok_or(ArmAlgorithmError::InvalidModel {
                            reason: "resolved coordinate belongs to a fixed joint",
                        })?,
                });
                transform = joint_frame * motion_transform(joint, coordinate.value)?;
            } else {
                transform = joint_frame;
            }
            current_link = joint.child_link();
            link_poses.push((current_link, transform));
        }
        if current_link != self.group.tip_link() {
            return Err(ArmAlgorithmError::InvalidModel {
                reason: "group path does not end at its tip",
            });
        }
        Ok(EvaluatedChain {
            link_poses,
            joint_frames,
            tool_transform: transform,
        })
    }

    fn validate_positions(
        self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<(), ArmAlgorithmError> {
        if positions.order() != self.group.joint_order() {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group.id(),
            });
        }
        for (descriptor, position) in self
            .group
            .joint_order()
            .joints()
            .iter()
            .zip(positions.values())
        {
            if position.kind() != descriptor.kind().coordinate_kind() {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            let joint =
                self.model
                    .joint(descriptor.joint_id())
                    .ok_or(ArmAlgorithmError::InvalidModel {
                        reason: "group order references an unknown joint",
                    })?;
            validate_joint_limit(joint, position.value_si())?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct IndependentCoordinate {
    value: f64,
    column: usize,
}

#[derive(Clone, Copy)]
struct ResolvedCoordinate {
    value: f64,
    source_column: usize,
    source_multiplier: f64,
}

pub(crate) struct EvaluatedJointFrame {
    pub joint_index: usize,
    pub frame_before_motion: Isometry3<f64>,
    pub source_column: usize,
    pub source_multiplier: f64,
    pub kind: ArmJointKind,
}

pub(crate) struct EvaluatedChain {
    pub link_poses: Vec<(arm_model::ArmLinkId, Isometry3<f64>)>,
    pub joint_frames: Vec<EvaluatedJointFrame>,
    pub tool_transform: Isometry3<f64>,
}

fn resolve_coordinate(
    model: &ArmModel,
    joint: &ArmJoint,
    independent: &BTreeMap<ArmJointId, IndependentCoordinate>,
) -> Result<Option<ResolvedCoordinate>, ArmAlgorithmError> {
    if joint.kind() == ModelJointKind::Fixed {
        return Ok(None);
    }
    let mut visited = BTreeSet::new();
    resolve_active_coordinate(model, joint, independent, &mut visited).map(Some)
}

fn resolve_active_coordinate(
    model: &ArmModel,
    joint: &ArmJoint,
    independent: &BTreeMap<ArmJointId, IndependentCoordinate>,
    visited: &mut BTreeSet<ArmJointId>,
) -> Result<ResolvedCoordinate, ArmAlgorithmError> {
    if !visited.insert(joint.id()) {
        return Err(ArmAlgorithmError::InvalidModel {
            reason: "mimic cycle passed model compilation",
        });
    }
    if let Some(coordinate) = independent.get(&joint.id()).copied() {
        return Ok(ResolvedCoordinate {
            value: coordinate.value,
            source_column: coordinate.column,
            source_multiplier: 1.0,
        });
    }
    let mimic = joint
        .mimic()
        .ok_or(ArmAlgorithmError::MimicSourceUnavailable {
            joint: Box::new(joint.id()),
            source_joint: Box::new(joint.id()),
        })?;
    let source =
        model
            .joint(mimic.source_joint())
            .ok_or(ArmAlgorithmError::MimicSourceUnavailable {
                joint: Box::new(joint.id()),
                source_joint: Box::new(mimic.source_joint()),
            })?;
    let source_coordinate = resolve_active_coordinate(model, source, independent, visited)
        .map_err(|_| ArmAlgorithmError::MimicSourceUnavailable {
            joint: Box::new(joint.id()),
            source_joint: Box::new(mimic.source_joint()),
        })?;
    Ok(ResolvedCoordinate {
        value: mimic
            .multiplier()
            .mul_add(source_coordinate.value, mimic.offset()),
        source_column: source_coordinate.source_column,
        source_multiplier: mimic.multiplier() * source_coordinate.source_multiplier,
    })
}

fn validate_joint_limit(joint: &ArmJoint, value: f64) -> Result<(), ArmAlgorithmError> {
    if !value.is_finite() {
        return Err(ArmAlgorithmError::NonFinite {
            field: "joint_position",
        });
    }
    if let Some([lower, upper]) = joint.limits().and_then(|limits| limits.position())
        && !(lower..=upper).contains(&value)
    {
        return Err(ArmAlgorithmError::JointLimit {
            joint: joint.id(),
            value,
            lower,
            upper,
        });
    }
    Ok(())
}

fn motion_transform(
    joint: &ArmJoint,
    coordinate: f64,
) -> Result<Isometry3<f64>, ArmAlgorithmError> {
    let axis = Unit::try_new(Vector3::from(joint.axis()), 1.0e-15).ok_or(
        ArmAlgorithmError::InvalidModel {
            reason: "active joint axis is invalid",
        },
    )?;
    match joint.kind().active() {
        Some(ArmJointKind::Revolute | ArmJointKind::Continuous) => Ok(Isometry3::from_parts(
            Translation3::identity(),
            UnitQuaternion::from_axis_angle(&axis, coordinate),
        )),
        Some(ArmJointKind::Prismatic) => Ok(Isometry3::from_parts(
            Translation3::from(axis.into_inner() * coordinate),
            UnitQuaternion::identity(),
        )),
        None => Ok(Isometry3::identity()),
    }
}
