// SPDX-License-Identifier: Apache-2.0
//
// Project-native fixed-capacity validation glue for analytical IK kernels
// adapted from softmata/kinetic commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646. This file is not an additional
// solver: it only applies project FK, limits, tolerance and stable selection.

use message_model::MAX_JOINTS;

use super::{ArmPose, IkSolveConfig, IkWorkspace, KernelOutcome, PreparedKinematics, pose_error};
use crate::ArmAlgorithmError;

pub(super) struct CandidateAccumulator {
    seed: [f64; MAX_JOINTS],
    best: [f64; MAX_JOINTS],
    best_combined: f64,
    best_seed_distance: f64,
    best_position_error_m: f64,
    best_orientation_error_rad: f64,
    generated: u32,
    evaluations: u32,
    has_best: bool,
}

impl CandidateAccumulator {
    pub const fn new(seed: [f64; MAX_JOINTS]) -> Self {
        Self {
            seed,
            best: [0.0; MAX_JOINTS],
            best_combined: f64::INFINITY,
            best_seed_distance: f64::INFINITY,
            best_position_error_m: f64::INFINITY,
            best_orientation_error_rad: f64::INFINITY,
            generated: 0,
            evaluations: 0,
            has_best: false,
        }
    }

    pub fn consider(
        &mut self,
        prepared: &PreparedKinematics<'_>,
        workspace: &mut IkWorkspace,
        target: ArmPose,
        candidate: [f64; MAX_JOINTS],
    ) -> Result<(), ArmAlgorithmError> {
        self.generated = self.generated.saturating_add(1);
        let dof = prepared.dof();
        if prepared.validate_values(&candidate[..dof]).is_err() {
            return Ok(());
        }
        let pose = prepared.evaluate(workspace, &candidate[..dof])?;
        self.evaluations = self.evaluations.saturating_add(1);
        let (_, position_error_m, orientation_error_rad) = pose_error(&pose, target);
        if !position_error_m.is_finite() || !orientation_error_rad.is_finite() {
            return Ok(());
        }
        let combined = position_error_m + 0.1 * orientation_error_rad;
        let seed_distance = candidate[..dof]
            .iter()
            .zip(&self.seed[..dof])
            .map(|(candidate, seed)| (candidate - seed).powi(2))
            .sum::<f64>();
        if !self.has_best
            || combined < self.best_combined
            || (combined == self.best_combined && seed_distance < self.best_seed_distance)
            || (combined == self.best_combined
                && seed_distance == self.best_seed_distance
                && candidate[..dof] < self.best[..dof])
        {
            self.best = candidate;
            self.best_combined = combined;
            self.best_seed_distance = seed_distance;
            self.best_position_error_m = position_error_m;
            self.best_orientation_error_rad = orientation_error_rad;
            self.has_best = true;
        }
        Ok(())
    }

    pub fn finish(
        self,
        config: IkSolveConfig,
        iterations: u32,
    ) -> Result<KernelOutcome, ArmAlgorithmError> {
        if self.has_best
            && self.best_position_error_m < config.position_tolerance_m
            && self.best_orientation_error_rad < config.orientation_tolerance_rad
        {
            return Ok(KernelOutcome {
                coordinates: self.best,
                iterations,
                evaluations: self.evaluations,
                generations: 0,
                candidates: self.generated,
                position_error_m: self.best_position_error_m,
                orientation_error_rad: self.best_orientation_error_rad,
                degraded: false,
            });
        }
        Err(ArmAlgorithmError::NonConvergent {
            iterations,
            position_error_m: self.best_position_error_m,
            orientation_error_rad: self.best_orientation_error_rad,
        })
    }
}
