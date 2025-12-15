//! Explicit-seed, bounded workspace sampling.

use message_model::{ArmJointVector, ArmPosition};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

use super::{ArmPose, KinematicChain};
use crate::ArmAlgorithmError;

/// Maximum accepted samples in one workspace map.
pub const MAX_WORKSPACE_SAMPLES: usize = 100_000;

/// Deterministic workspace sampling configuration.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WorkspaceSamplingConfig {
    seed: [u8; 32],
    sample_count: usize,
    continuous_position_bounds_rad: [f64; 2],
}

impl WorkspaceSamplingConfig {
    /// Validates an explicit seed, work bound and finite continuous-joint range.
    pub fn new(
        seed: [u8; 32],
        sample_count: usize,
        continuous_position_bounds_rad: [f64; 2],
    ) -> Result<Self, ArmAlgorithmError> {
        if sample_count == 0 || sample_count > MAX_WORKSPACE_SAMPLES {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "workspace.sample_count",
            });
        }
        let [lower, upper] = continuous_position_bounds_rad;
        if !lower.is_finite() || !upper.is_finite() || lower >= upper {
            return Err(ArmAlgorithmError::NonFinite {
                field: "workspace.continuous_bounds",
            });
        }
        Ok(Self {
            seed,
            sample_count,
            continuous_position_bounds_rad,
        })
    }

    /// Returns the exact portable seed.
    #[must_use]
    pub const fn seed(self) -> [u8; 32] {
        self.seed
    }

    /// Returns the bounded sample count.
    #[must_use]
    pub const fn sample_count(self) -> usize {
        self.sample_count
    }
}

/// One deterministic sampled configuration and tool pose.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkspaceSample {
    positions: ArmJointVector<ArmPosition>,
    tool_pose: ArmPose,
}

impl WorkspaceSample {
    /// Returns the sampled stable-order configuration.
    #[must_use]
    pub const fn positions(&self) -> &ArmJointVector<ArmPosition> {
        &self.positions
    }

    /// Returns the corresponding tool pose.
    #[must_use]
    pub const fn tool_pose(&self) -> ArmPose {
        self.tool_pose
    }
}

/// Bounded deterministic workspace map.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkspaceMap {
    seed: [u8; 32],
    samples: Vec<WorkspaceSample>,
}

impl WorkspaceMap {
    /// Returns the seed that fully determines the sampling stream.
    #[must_use]
    pub const fn seed(&self) -> &[u8; 32] {
        &self.seed
    }

    /// Returns samples in deterministic generation order.
    #[must_use]
    pub fn samples(&self) -> &[WorkspaceSample] {
        &self.samples
    }
}

impl KinematicChain<'_> {
    /// Samples joint limits and evaluates the corresponding tool poses.
    pub fn sample_workspace(
        self,
        config: WorkspaceSamplingConfig,
    ) -> Result<WorkspaceMap, ArmAlgorithmError> {
        let mut rng = ChaCha8Rng::from_seed(config.seed);
        let [continuous_lower, continuous_upper] = config.continuous_position_bounds_rad;
        let mut samples = Vec::with_capacity(config.sample_count);

        for _ in 0..config.sample_count {
            let mut values = Vec::with_capacity(self.dof());
            for descriptor in self.group().joint_order().joints() {
                let joint = self.model().joint(descriptor.joint_id()).ok_or(
                    ArmAlgorithmError::InvalidModel {
                        reason: "group order references missing joint",
                    },
                )?;
                let [lower, upper] = joint
                    .limits()
                    .and_then(|limits| limits.position())
                    .unwrap_or([continuous_lower, continuous_upper]);
                values.push(rng.random_range(lower..=upper));
            }
            let positions = self.positions_from_values(&values)?;
            let tool_pose = self.forward(&positions)?.tool_pose();
            samples.push(WorkspaceSample {
                positions,
                tool_pose,
            });
        }
        Ok(WorkspaceMap {
            seed: config.seed,
            samples,
        })
    }
}
