//! KINETIC-informed deterministic constraint-region preparation.
//!
//! Source: `kinetic-planning/src/iris.rs` at KINETIC commit
//! `3dbaa61579c4359ce4c35f246fc83e1d3110b646` (source SHA-256
//! `3f044d967643333d71d5cbaa71e93a2d8bc1986eead91d410bce2c7b193a6e99`).
//! Regions are caller-certified semantic boxes and never claim collision freedom.

use message_model::{ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId, KinematicGroupId};

use crate::ArmAlgorithmError;

use super::{
    ArmStateValidator, PlannerConfig, PlannerEvidence, PlannerKind, PlannerStatistics,
    PreparedPlanning,
    core::{PlannerWorkspace, State, copy_position, sample_uniform, validate_state},
};

/// Provider that can certify every point in an axis-aligned semantic region.
pub trait RegionConstraintProvider {
    /// Returns the semantic constraint generation.
    fn generation(&self) -> u64;

    /// Validates one point without allocation.
    fn validate(&self, state_si: &[f64]) -> Result<(), &'static str>;

    /// Certifies the complete closed box `[lower, upper]`.
    fn certify_box(&self, lower: &[f64], upper: &[f64]) -> Result<bool, &'static str>;
}

/// Bounded deterministic IRIS preparation settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IrisConfig {
    maximum_regions: usize,
    initial_radius: f64,
    minimum_radius: f64,
    maximum_shrinks: u32,
}

impl IrisConfig {
    /// Validates region capacity and inflation bounds.
    pub fn new(
        maximum_regions: usize,
        initial_radius: f64,
        minimum_radius: f64,
        maximum_shrinks: u32,
    ) -> Result<Self, ArmAlgorithmError> {
        if maximum_regions == 0 || maximum_regions > 4_096 || maximum_shrinks == 0 {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "planner.iris_work",
            });
        }
        if !initial_radius.is_finite()
            || !minimum_radius.is_finite()
            || initial_radius <= 0.0
            || minimum_radius <= 0.0
            || minimum_radius > initial_radius
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "planner.iris_radius",
            });
        }
        Ok(Self {
            maximum_regions,
            initial_radius,
            minimum_radius,
            maximum_shrinks,
        })
    }

    /// Returns conservative nominal preparation settings.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            maximum_regions: 64,
            initial_radius: 0.25,
            minimum_radius: 1.0e-4,
            maximum_shrinks: 16,
        }
    }
}

/// Caller-certified axis-aligned constraint-feasible region.
#[derive(Clone, Debug)]
pub struct ConstraintRegion {
    center: State,
    lower: State,
    upper: State,
    dof: usize,
}

impl ConstraintRegion {
    /// Returns the certified region center in stable SI joint order.
    #[must_use]
    pub fn center_values_si(&self) -> &[f64] {
        self.center.as_slice(self.dof)
    }

    /// Returns the inclusive certified lower bounds in stable SI joint order.
    #[must_use]
    pub fn lower_bounds_si(&self) -> &[f64] {
        self.lower.as_slice(self.dof)
    }

    /// Returns the inclusive certified upper bounds in stable SI joint order.
    #[must_use]
    pub fn upper_bounds_si(&self) -> &[f64] {
        self.upper.as_slice(self.dof)
    }

    /// Returns whether raw SI coordinates are inside every prepared halfspace.
    #[must_use]
    pub fn contains_values(&self, values_si: &[f64]) -> bool {
        values_si.len() == self.dof
            && values_si.iter().enumerate().all(|(index, value)| {
                (self.lower.as_slice(self.dof)[index]..=self.upper.as_slice(self.dof)[index])
                    .contains(value)
            })
    }

    /// Returns the fixed number of box halfspaces.
    #[must_use]
    pub const fn halfspace_count(&self) -> usize {
        self.dof * 2
    }

    pub(crate) const fn center_state(&self) -> State {
        self.center
    }
}

/// Prepared collection of caller-certified constraint regions.
#[derive(Clone)]
pub struct PreparedConstraintRegions {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    model_generation: u64,
    constraint_generation: u64,
    regions: Vec<ConstraintRegion>,
    samples: u32,
}

impl PreparedConstraintRegions {
    /// Inflates conservative regions around explicit valid seeds.
    pub fn build(
        prepared: &PreparedPlanning<'_, '_>,
        seeds: &[ArmJointVector<ArmPosition>],
        config: IrisConfig,
        provider: &impl RegionConstraintProvider,
    ) -> Result<Self, ArmAlgorithmError> {
        require_provider_generation(prepared, provider.generation())?;
        let mut regions = Vec::with_capacity(config.maximum_regions.min(seeds.len()));
        for seed in seeds.iter().take(config.maximum_regions) {
            let state = copy_position(prepared, seed)?;
            if provider.validate(state.as_slice(prepared.dof())).is_err() {
                continue;
            }
            if let Some(region) = inflate(prepared, state, config, provider)? {
                regions.push(region);
            }
        }
        if regions.is_empty() {
            return Err(ArmAlgorithmError::WorkBudgetExhausted {
                planner: PlannerKind::Iris.name(),
                iterations: config.maximum_shrinks,
                samples: seeds.len() as u32,
                nodes: 0,
            });
        }
        Ok(Self::from_regions(
            prepared,
            regions,
            seeds.len().min(config.maximum_regions) as u32,
        ))
    }

    /// Returns prepared regions in deterministic seed order.
    #[must_use]
    pub fn regions(&self) -> &[ConstraintRegion] {
        &self.regions
    }

    /// Returns the number of consumed preparation samples.
    #[must_use]
    pub const fn samples(&self) -> u32 {
        self.samples
    }

    pub(crate) fn require_context(
        &self,
        prepared: &PreparedPlanning<'_, '_>,
    ) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || &self.order != prepared.order()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "constraint-region profile, group or order changed",
            });
        }
        if self.model_generation != prepared.model_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.model_generation,
                actual: prepared.model_generation(),
            });
        }
        if self.constraint_generation != prepared.constraint_generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: self.constraint_generation,
                actual: prepared.constraint_generation(),
            });
        }
        Ok(())
    }

    pub(crate) fn from_point_states(
        prepared: &PreparedPlanning<'_, '_>,
        states: &[State],
        samples: u32,
    ) -> Self {
        let regions = states
            .iter()
            .copied()
            .map(|state| ConstraintRegion {
                center: state,
                lower: state,
                upper: state,
                dof: prepared.dof(),
            })
            .collect();
        Self::from_regions(prepared, regions, samples)
    }

    fn from_regions(
        prepared: &PreparedPlanning<'_, '_>,
        regions: Vec<ConstraintRegion>,
        samples: u32,
    ) -> Self {
        Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            model_generation: prepared.model_generation(),
            constraint_generation: prepared.constraint_generation(),
            regions,
            samples,
        }
    }
}

fn require_provider_generation(
    prepared: &PreparedPlanning<'_, '_>,
    generation: u64,
) -> Result<(), ArmAlgorithmError> {
    if generation != prepared.constraint_generation() {
        return Err(ArmAlgorithmError::PreparedGenerationMismatch {
            expected: prepared.constraint_generation(),
            actual: generation,
        });
    }
    Ok(())
}

fn inflate(
    prepared: &PreparedPlanning<'_, '_>,
    center: State,
    config: IrisConfig,
    provider: &impl RegionConstraintProvider,
) -> Result<Option<ConstraintRegion>, ArmAlgorithmError> {
    let mut radius = config.initial_radius;
    for _ in 0..config.maximum_shrinks {
        let mut lower = center;
        let mut upper = center;
        for joint in 0..prepared.dof() {
            let [minimum, maximum] = prepared.kinematics.sampling_bound(joint);
            lower.as_mut_slice(prepared.dof())[joint] =
                (center.as_slice(prepared.dof())[joint] - radius).max(minimum);
            upper.as_mut_slice(prepared.dof())[joint] =
                (center.as_slice(prepared.dof())[joint] + radius).min(maximum);
        }
        let certified = provider
            .certify_box(
                lower.as_slice(prepared.dof()),
                upper.as_slice(prepared.dof()),
            )
            .map_err(|reason| ArmAlgorithmError::ConstraintViolation {
                sample_index: 0,
                reason,
            })?;
        if certified {
            return Ok(Some(ConstraintRegion {
                center,
                lower,
                upper,
                dof: prepared.dof(),
            }));
        }
        radius *= 0.5;
        if radius < config.minimum_radius {
            break;
        }
    }
    Ok(None)
}

pub(super) fn sample_point_regions(
    prepared: &PreparedPlanning<'_, '_>,
    config: PlannerConfig,
    validator: &impl ArmStateValidator,
    workspace: &mut PlannerWorkspace,
) -> Result<PreparedConstraintRegions, ArmAlgorithmError> {
    workspace.first.clear();
    let target = 64_usize
        .min(config.maximum_samples as usize)
        .min(config.maximum_nodes as usize);
    let mut samples = 0_u32;
    while workspace.first.len() < target && samples < config.maximum_samples {
        let mut state = State::zero();
        sample_uniform(prepared, &mut workspace.rng, &mut state);
        samples += 1;
        if validate_state(prepared, &state, validator, samples as usize).is_ok() {
            workspace.first.push(super::core::Node::root(state));
        }
    }
    if workspace.first.len() < 2 {
        return Err(ArmAlgorithmError::WorkBudgetExhausted {
            planner: PlannerKind::Iris.name(),
            iterations: 0,
            samples,
            nodes: workspace.first.len() as u32,
        });
    }
    let states = workspace
        .first
        .iter()
        .map(|node| node.state)
        .collect::<Vec<_>>();
    Ok(PreparedConstraintRegions::from_point_states(
        prepared, &states, samples,
    ))
}

pub(super) fn statistics(regions: &PreparedConstraintRegions) -> PlannerStatistics {
    PlannerStatistics::new(
        0,
        regions.samples,
        regions.regions.len() as u32,
        PlannerEvidence::Iris {
            regions: regions.regions.len() as u32,
            halfspaces: regions
                .regions
                .iter()
                .map(ConstraintRegion::halfspace_count)
                .sum::<usize>() as u32,
            samples: regions.samples,
        },
    )
}
