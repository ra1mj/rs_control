//! Geometric Jacobian and typed singularity analysis.

use message_model::{ArmJointVector, ArmPosition};
use nalgebra::{DMatrix, Vector3};

use super::KinematicChain;
use crate::ArmAlgorithmError;

/// Bounded 6×N geometric Jacobian in row-major order.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmJacobian {
    columns: usize,
    data: Vec<f64>,
}

impl ArmJacobian {
    /// Number of twist rows: linear xyz followed by angular xyz.
    pub const ROWS: usize = 6;

    /// Returns the active group degree of freedom.
    #[must_use]
    pub const fn columns(&self) -> usize {
        self.columns
    }

    /// Returns one matrix entry.
    #[must_use]
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row < Self::ROWS && column < self.columns {
            Some(self.data[row * self.columns + column])
        } else {
            None
        }
    }

    /// Returns validated row-major storage.
    #[must_use]
    pub fn as_row_major(&self) -> &[f64] {
        &self.data
    }

    pub(crate) fn as_matrix(&self) -> DMatrix<f64> {
        DMatrix::from_row_slice(Self::ROWS, self.columns, &self.data)
    }
}

/// Numerical rank and conditioning evidence for a Jacobian.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JacobianAnalysis {
    rank: usize,
    required_rank: usize,
    smallest_singular_value: f64,
    largest_singular_value: f64,
    condition_number: Option<f64>,
    manipulability: f64,
}

impl JacobianAnalysis {
    /// Returns the numerical rank at the requested tolerance.
    #[must_use]
    pub const fn rank(self) -> usize {
        self.rank
    }

    /// Returns `min(6, dof)`.
    #[must_use]
    pub const fn required_rank(self) -> usize {
        self.required_rank
    }

    /// Returns whether the group is singular at the requested tolerance.
    #[must_use]
    pub const fn is_singular(self) -> bool {
        self.rank < self.required_rank
    }

    /// Returns the smallest singular value in the required subspace.
    #[must_use]
    pub const fn smallest_singular_value(self) -> f64 {
        self.smallest_singular_value
    }

    /// Returns the largest singular value.
    #[must_use]
    pub const fn largest_singular_value(self) -> f64 {
        self.largest_singular_value
    }

    /// Returns a finite condition number, or `None` for a singular matrix.
    #[must_use]
    pub const fn condition_number(self) -> Option<f64> {
        self.condition_number
    }

    /// Returns the product of singular values in the required subspace.
    #[must_use]
    pub const fn manipulability(self) -> f64 {
        self.manipulability
    }
}

impl KinematicChain<'_> {
    /// Computes the 6×N base-frame geometric Jacobian.
    pub fn jacobian(
        self,
        positions: &ArmJointVector<ArmPosition>,
    ) -> Result<ArmJacobian, ArmAlgorithmError> {
        let evaluated = self.evaluate(positions)?;
        let columns = self.dof();
        let mut data = vec![0.0; ArmJacobian::ROWS * columns];
        let tip = evaluated.tool_transform.translation.vector;

        for frame in &evaluated.joint_frames {
            let joint = self.model().joints().get(frame.joint_index).ok_or(
                ArmAlgorithmError::InvalidModel {
                    reason: "evaluated joint index is out of bounds",
                },
            )?;
            let local_axis = Vector3::from(joint.axis());
            let world_axis = frame.frame_before_motion.rotation * local_axis;
            let origin = frame.frame_before_motion.translation.vector;
            let (linear, angular) = match frame.kind {
                message_model::ArmJointKind::Revolute | message_model::ArmJointKind::Continuous => {
                    (world_axis.cross(&(tip - origin)), world_axis)
                }
                message_model::ArmJointKind::Prismatic => (world_axis, Vector3::zeros()),
            };
            let scale = frame.source_multiplier;
            let column = frame.source_column;
            for row in 0..3 {
                data[row * columns + column] += linear[row] * scale;
                data[(row + 3) * columns + column] += angular[row] * scale;
            }
        }
        if data.iter().any(|value| !value.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite { field: "jacobian" });
        }
        Ok(ArmJacobian { columns, data })
    }
}

impl ArmJacobian {
    /// Computes rank, condition number and manipulability.
    pub fn analyze(
        &self,
        singular_value_tolerance: f64,
    ) -> Result<JacobianAnalysis, ArmAlgorithmError> {
        if !singular_value_tolerance.is_finite() || singular_value_tolerance <= 0.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "singular_value_tolerance",
            });
        }
        let singular_values = self.as_matrix().svd(false, false).singular_values;
        let required_rank = Self::ROWS.min(self.columns);
        let rank = singular_values
            .iter()
            .take(required_rank)
            .filter(|value| **value > singular_value_tolerance)
            .count();
        let largest = singular_values.get(0).copied().unwrap_or(0.0);
        let smallest = singular_values
            .get(required_rank.saturating_sub(1))
            .copied()
            .unwrap_or(0.0);
        let condition_number =
            (rank == required_rank && smallest > 0.0).then_some(largest / smallest);
        let manipulability = singular_values
            .iter()
            .take(required_rank)
            .copied()
            .product();
        Ok(JacobianAnalysis {
            rank,
            required_rank,
            smallest_singular_value: smallest,
            largest_singular_value: largest,
            condition_number,
            manipulability,
        })
    }
}
