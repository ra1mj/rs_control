//! Bounded payload-gravity identification and projected recursive refinement.
//!
//! The estimator identifies only `[m, m*cx, m*cy, m*cz]`. It consumes
//! estimator-visible gravity regressors and effort residuals; no simulator
//! plant-truth type is accepted by this module.

use arrayvec::ArrayVec;
use message_model::{ArmJointOrder, ArmProfileId, KinematicGroupId, MAX_JOINTS};
use thiserror::Error;

use crate::dynamics::PreparedGravity;

/// Maximum accepted settled calibration poses.
pub const MAX_CALIBRATION_WINDOWS: usize = 24;
/// Maximum scalar joint rows retained by the bounded batch solver.
pub const MAX_BATCH_ROWS: usize = MAX_CALIBRATION_WINDOWS * MAX_JOINTS;

/// Public calibration/refinement state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdaptiveGravityState {
    /// No model-bound preparation exists (reserved for diagnostic snapshots).
    Unprepared,
    Ready,
    Collecting,
    Solving,
    Candidate,
    Active,
    Frozen,
    Rejected,
    Faulted,
}

/// Typed non-fault reason for a skipped observation/update.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdaptiveSkipReason {
    IdentityMismatch,
    Stale,
    Moving,
    Saturated,
    Contact,
    NonFinite,
    InsufficientExcitation,
    Innovation,
    Capacity,
}

/// Bounded counters suitable for a diagnostic snapshot.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AdaptiveGravityCounters {
    pub accepted_windows: u32,
    pub accepted_rows: u32,
    pub accepted_rls_updates: u32,
    pub identity_mismatch: u32,
    pub stale: u32,
    pub moving: u32,
    pub saturated: u32,
    pub contact: u32,
    pub non_finite: u32,
    pub insufficient_excitation: u32,
    pub innovation: u32,
    pub capacity: u32,
}

impl AdaptiveGravityCounters {
    fn skip(&mut self, reason: AdaptiveSkipReason) {
        let counter = match reason {
            AdaptiveSkipReason::IdentityMismatch => &mut self.identity_mismatch,
            AdaptiveSkipReason::Stale => &mut self.stale,
            AdaptiveSkipReason::Moving => &mut self.moving,
            AdaptiveSkipReason::Saturated => &mut self.saturated,
            AdaptiveSkipReason::Contact => &mut self.contact,
            AdaptiveSkipReason::NonFinite => &mut self.non_finite,
            AdaptiveSkipReason::InsufficientExcitation => &mut self.insufficient_excitation,
            AdaptiveSkipReason::Innovation => &mut self.innovation,
            AdaptiveSkipReason::Capacity => &mut self.capacity,
        };
        *counter = counter.saturating_add(1);
    }
}

/// Estimator-visible gate evidence for one settled window or RLS update.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AdaptiveSampleContext {
    pub generation: u64,
    pub fresh: bool,
    pub settled: bool,
    pub saturated: bool,
    pub contact_free: bool,
}

/// Validated estimator limits and numerical gates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AdaptiveGravityConfig {
    mass_bounds_kg: [f64; 2],
    cog_bounds_m: [[f64; 2]; 3],
    minimum_rank_diagonal: f64,
    maximum_condition: f64,
    forgetting_factor: f64,
    initial_covariance: f64,
    minimum_excitation: f64,
    maximum_innovation: f64,
    maximum_parameter_step: f64,
    maximum_payload_effort: f64,
    maximum_rls_updates: u32,
}

impl AdaptiveGravityConfig {
    /// Conservative host qualification defaults for a small industrial tool.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            mass_bounds_kg: [0.05, 20.0],
            cog_bounds_m: [[-0.5, 0.5], [-0.5, 0.5], [-0.5, 0.5]],
            minimum_rank_diagonal: 1.0e-7,
            maximum_condition: 1.0e8,
            forgetting_factor: 0.999,
            initial_covariance: 100.0,
            minimum_excitation: 1.0e-10,
            maximum_innovation: 20.0,
            maximum_parameter_step: 0.05,
            maximum_payload_effort: 50.0,
            maximum_rls_updates: 2_048,
        }
    }

    /// Replaces physical payload bounds and validates the full configuration.
    pub fn with_payload_bounds(
        mut self,
        mass_bounds_kg: [f64; 2],
        cog_bounds_m: [[f64; 2]; 3],
    ) -> Result<Self, AdaptiveGravityError> {
        self.mass_bounds_kg = mass_bounds_kg;
        self.cog_bounds_m = cog_bounds_m;
        self.validate()?;
        Ok(self)
    }

    fn validate(self) -> Result<(), AdaptiveGravityError> {
        if self.mass_bounds_kg.iter().any(|v| !v.is_finite())
            || self.mass_bounds_kg[0] <= 0.0
            || self.mass_bounds_kg[0] > self.mass_bounds_kg[1]
            || self.cog_bounds_m.iter().flatten().any(|v| !v.is_finite())
            || self.cog_bounds_m.iter().any(|v| v[0] > v[1])
            || !self.minimum_rank_diagonal.is_finite()
            || self.minimum_rank_diagonal <= 0.0
            || !self.maximum_condition.is_finite()
            || self.maximum_condition <= 1.0
            || !(0.9..=1.0).contains(&self.forgetting_factor)
            || !self.initial_covariance.is_finite()
            || self.initial_covariance <= 0.0
            || !self.minimum_excitation.is_finite()
            || self.minimum_excitation <= 0.0
            || !self.maximum_innovation.is_finite()
            || self.maximum_innovation <= 0.0
            || !self.maximum_parameter_step.is_finite()
            || self.maximum_parameter_step <= 0.0
            || !self.maximum_payload_effort.is_finite()
            || self.maximum_payload_effort <= 0.0
            || self.maximum_rls_updates == 0
        {
            return Err(AdaptiveGravityError::InvalidConfiguration);
        }
        Ok(())
    }
}

/// Physical four-parameter payload snapshot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PayloadParameters([f64; 4]);

impl PayloadParameters {
    /// Encodes mass and tool-frame CoG as mass plus first moments.
    pub fn from_mass_cog(mass_kg: f64, cog_m: [f64; 3]) -> Result<Self, AdaptiveGravityError> {
        if !mass_kg.is_finite() || mass_kg <= 0.0 || cog_m.iter().any(|v| !v.is_finite()) {
            return Err(AdaptiveGravityError::NonFinite);
        }
        let moments = [
            mass_kg,
            mass_kg * cog_m[0],
            mass_kg * cog_m[1],
            mass_kg * cog_m[2],
        ];
        if moments.iter().any(|v| !v.is_finite()) {
            return Err(AdaptiveGravityError::NonFinite);
        }
        Ok(Self(moments))
    }

    /// Returns `[m, m*cx, m*cy, m*cz]`.
    #[must_use]
    pub const fn as_array(self) -> [f64; 4] {
        self.0
    }

    /// Returns mass in kilograms.
    #[must_use]
    pub const fn mass_kg(self) -> f64 {
        self.0[0]
    }

    /// Decodes the tool-frame CoG in metres.
    #[must_use]
    pub fn cog_m(self) -> [f64; 3] {
        [
            self.0[1] / self.0[0],
            self.0[2] / self.0[0],
            self.0[3] / self.0[0],
        ]
    }
}

#[derive(Clone, Copy, Debug)]
struct BatchRow {
    phi: [f64; 4],
    value: f64,
    weight: f64,
}

/// Bounded adaptive payload-gravity estimator.
pub struct AdaptiveGravityEstimator {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    generation: u64,
    order: ArmJointOrder,
    config: AdaptiveGravityConfig,
    state: AdaptiveGravityState,
    rows: ArrayVec<BatchRow, MAX_BATCH_ROWS>,
    windows: usize,
    candidate: Option<PayloadParameters>,
    candidate_validated: bool,
    active: Option<PayloadParameters>,
    covariance: [[f64; 4]; 4],
    counters: AdaptiveGravityCounters,
}

impl AdaptiveGravityEstimator {
    /// Prepares a fresh estimator for the exact prepared model generation.
    pub fn new(
        prepared: &PreparedGravity,
        config: AdaptiveGravityConfig,
    ) -> Result<Self, AdaptiveGravityError> {
        config.validate()?;
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            generation: prepared.generation(),
            order: prepared.order().clone(),
            config,
            state: AdaptiveGravityState::Ready,
            rows: ArrayVec::new(),
            windows: 0,
            candidate: None,
            candidate_validated: false,
            active: None,
            covariance: diagonal(config.initial_covariance),
            counters: AdaptiveGravityCounters::default(),
        })
    }

    /// Returns current state.
    #[must_use]
    pub const fn state(&self) -> AdaptiveGravityState {
        self.state
    }

    /// Returns the immutable profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the immutable group identity.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the exact model generation.
    #[must_use]
    pub const fn generation(&self) -> u64 {
        self.generation
    }

    /// Returns stable prepared joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns bounded diagnostic counters.
    #[must_use]
    pub const fn counters(&self) -> AdaptiveGravityCounters {
        self.counters
    }

    /// Returns the active immutable parameter snapshot.
    #[must_use]
    pub const fn active_parameters(&self) -> Option<PayloadParameters> {
        self.active
    }

    /// Clears calibration rows without changing immutable identity/config.
    pub fn start_collection(&mut self) {
        self.rows.clear();
        self.windows = 0;
        self.candidate = None;
        self.candidate_validated = false;
        self.state = AdaptiveGravityState::Collecting;
    }

    /// Adds one already-aggregated settled pose window in stable joint order.
    pub fn observe_window(
        &mut self,
        context: AdaptiveSampleContext,
        regressor: &[f64],
        residual_effort: &[f64],
        weight: f64,
    ) -> Result<bool, AdaptiveGravityError> {
        if self.state != AdaptiveGravityState::Collecting {
            return Err(AdaptiveGravityError::InvalidState);
        }
        if let Some(reason) = self.gate(context) {
            self.counters.skip(reason);
            return Ok(false);
        }
        let dof = self.order.len();
        if regressor.len() != dof * 4 || residual_effort.len() != dof {
            return Err(AdaptiveGravityError::DimensionMismatch);
        }
        if self.windows >= MAX_CALIBRATION_WINDOWS || self.rows.remaining_capacity() < dof {
            self.counters.skip(AdaptiveSkipReason::Capacity);
            return Err(AdaptiveGravityError::Capacity);
        }
        if !weight.is_finite()
            || weight <= 0.0
            || regressor
                .iter()
                .chain(residual_effort)
                .any(|v| !v.is_finite())
        {
            self.counters.skip(AdaptiveSkipReason::NonFinite);
            return Ok(false);
        }
        if self.window_is_duplicate(regressor, dof) {
            self.counters
                .skip(AdaptiveSkipReason::InsufficientExcitation);
            return Ok(false);
        }
        for (joint, value) in residual_effort.iter().copied().enumerate() {
            let base = joint * 4;
            self.rows.push(BatchRow {
                phi: [
                    regressor[base],
                    regressor[base + 1],
                    regressor[base + 2],
                    regressor[base + 3],
                ],
                value,
                weight,
            });
        }
        self.windows += 1;
        self.counters.accepted_windows = self.counters.accepted_windows.saturating_add(1);
        self.counters.accepted_rows = self.counters.accepted_rows.saturating_add(dof as u32);
        Ok(true)
    }

    /// Solves the bounded weighted least-squares problem with incremental
    /// Givens QR and creates a physically projected candidate.
    pub fn solve_batch(&mut self) -> Result<PayloadParameters, AdaptiveGravityError> {
        if self.state != AdaptiveGravityState::Collecting || self.rows.len() < 4 {
            return Err(AdaptiveGravityError::InvalidState);
        }
        let mut r = [[0.0_f64; 4]; 4];
        let mut z = [0.0_f64; 4];
        for row in &self.rows {
            let scale = row.weight.sqrt();
            let mut a = row.phi.map(|v| v * scale);
            let mut b = row.value * scale;
            for column in 0..4 {
                let radius = r[column][column].hypot(a[column]);
                if radius == 0.0 {
                    continue;
                }
                let cosine = r[column][column] / radius;
                let sine = a[column] / radius;
                r[column][column] = radius;
                for next in column + 1..4 {
                    let old_r = r[column][next];
                    r[column][next] = cosine * old_r + sine * a[next];
                    a[next] = -sine * old_r + cosine * a[next];
                }
                let old_z = z[column];
                z[column] = cosine * old_z + sine * b;
                b = -sine * old_z + cosine * b;
            }
        }
        let diagonals = [r[0][0].abs(), r[1][1].abs(), r[2][2].abs(), r[3][3].abs()];
        let minimum = diagonals.into_iter().fold(f64::INFINITY, f64::min);
        let maximum = diagonals.into_iter().fold(0.0_f64, f64::max);
        if minimum < self.config.minimum_rank_diagonal
            || maximum / minimum > self.config.maximum_condition
        {
            self.state = AdaptiveGravityState::Rejected;
            return Err(AdaptiveGravityError::Unobservable);
        }
        let mut theta = [0.0_f64; 4];
        for row in (0..4).rev() {
            let known = ((row + 1)..4)
                .map(|column| r[row][column] * theta[column])
                .sum::<f64>();
            theta[row] = (z[row] - known) / r[row][row];
        }
        let projected = self.project(theta)?;
        self.candidate = Some(projected);
        self.candidate_validated = false;
        self.state = AdaptiveGravityState::Candidate;
        Ok(projected)
    }

    /// Validates the candidate on caller-owned held-out estimator observations.
    pub fn validate_candidate(
        &mut self,
        regressor: &[f64],
        residual_effort: &[f64],
        maximum_normalized_rmse: f64,
    ) -> Result<f64, AdaptiveGravityError> {
        if self.state != AdaptiveGravityState::Candidate
            || regressor.len() != residual_effort.len() * 4
            || residual_effort.is_empty()
            || !maximum_normalized_rmse.is_finite()
            || maximum_normalized_rmse <= 0.0
        {
            return Err(AdaptiveGravityError::InvalidState);
        }
        let candidate = self
            .candidate
            .ok_or(AdaptiveGravityError::InvalidState)?
            .as_array();
        let mut sum = 0.0;
        let mut peak = 0.0_f64;
        for (row, measured) in residual_effort.iter().copied().enumerate() {
            let base = row * 4;
            let predicted = dot4(&regressor[base..base + 4], candidate);
            let error = predicted - measured;
            sum += error * error;
            peak = peak.max(measured.abs());
        }
        if !sum.is_finite() || peak <= 1.0e-12 {
            self.state = AdaptiveGravityState::Rejected;
            return Err(AdaptiveGravityError::ValidationFailed);
        }
        let normalized = (sum / residual_effort.len() as f64).sqrt() / peak;
        if normalized > maximum_normalized_rmse {
            self.state = AdaptiveGravityState::Rejected;
            return Err(AdaptiveGravityError::ValidationFailed);
        }
        self.candidate_validated = true;
        Ok(normalized)
    }

    /// Atomically promotes a validated candidate at a caller-selected cycle boundary.
    pub fn activate(&mut self) -> Result<PayloadParameters, AdaptiveGravityError> {
        if self.state != AdaptiveGravityState::Candidate || !self.candidate_validated {
            return Err(AdaptiveGravityError::InvalidState);
        }
        let active = self.candidate.ok_or(AdaptiveGravityError::InvalidState)?;
        self.active = Some(active);
        self.covariance = diagonal(self.config.initial_covariance);
        self.state = AdaptiveGravityState::Active;
        Ok(active)
    }

    /// Performs one scalar projected, gated RLS update.
    pub fn update_rls(
        &mut self,
        context: AdaptiveSampleContext,
        phi: [f64; 4],
        measured_residual: f64,
    ) -> Result<bool, AdaptiveGravityError> {
        if !matches!(
            self.state,
            AdaptiveGravityState::Active | AdaptiveGravityState::Frozen
        ) {
            return Err(AdaptiveGravityError::InvalidState);
        }
        if let Some(reason) = self.gate(context) {
            self.counters.skip(reason);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        if phi.iter().any(|v| !v.is_finite()) || !measured_residual.is_finite() {
            self.counters.skip(AdaptiveSkipReason::NonFinite);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        if self.counters.accepted_rls_updates >= self.config.maximum_rls_updates {
            self.counters.skip(AdaptiveSkipReason::Capacity);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        let mut p_phi = [0.0; 4];
        for (row, value) in p_phi.iter_mut().enumerate() {
            *value = (0..4)
                .map(|column| self.covariance[row][column] * phi[column])
                .sum();
        }
        let excitation = (0..4).map(|i| phi[i] * p_phi[i]).sum::<f64>();
        if !excitation.is_finite() || excitation < self.config.minimum_excitation {
            self.counters.skip(if excitation.is_finite() {
                AdaptiveSkipReason::InsufficientExcitation
            } else {
                AdaptiveSkipReason::NonFinite
            });
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        let active = self
            .active
            .ok_or(AdaptiveGravityError::InvalidState)?
            .as_array();
        let prediction = dot4(&phi, active);
        let raw_innovation = measured_residual - prediction;
        if !raw_innovation.is_finite() {
            self.counters.skip(AdaptiveSkipReason::NonFinite);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        if raw_innovation.abs() > self.config.maximum_innovation * 4.0 {
            self.counters.skip(AdaptiveSkipReason::Innovation);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        let innovation = raw_innovation.clamp(
            -self.config.maximum_innovation,
            self.config.maximum_innovation,
        );
        let denominator = self.config.forgetting_factor + excitation;
        let gain = p_phi.map(|v| v / denominator);
        let mut next = active;
        for index in 0..4 {
            next[index] += (gain[index] * innovation).clamp(
                -self.config.maximum_parameter_step,
                self.config.maximum_parameter_step,
            );
        }
        let next = self.project(next)?;
        let mut covariance = self.covariance;
        for (row, covariance_row) in covariance.iter_mut().enumerate() {
            for (column, value) in covariance_row.iter_mut().enumerate() {
                *value = (self.covariance[row][column] - gain[row] * p_phi[column])
                    / self.config.forgetting_factor;
            }
        }
        if covariance.iter().flatten().any(|value| !value.is_finite()) {
            self.counters.skip(AdaptiveSkipReason::NonFinite);
            self.state = AdaptiveGravityState::Frozen;
            return Ok(false);
        }
        let mut row = 0;
        while row < 4 {
            covariance[row][row] =
                covariance[row][row].clamp(1.0e-9, self.config.initial_covariance);
            let mut column = row + 1;
            while column < 4 {
                let symmetric = 0.5 * (covariance[row][column] + covariance[column][row]);
                covariance[row][column] = symmetric;
                covariance[column][row] = symmetric;
                column += 1;
            }
            row += 1;
        }
        self.active = Some(next);
        self.covariance = covariance;
        self.counters.accepted_rls_updates = self.counters.accepted_rls_updates.saturating_add(1);
        self.state = AdaptiveGravityState::Active;
        Ok(true)
    }

    /// Evaluates the active snapshot against a row-major payload regressor.
    pub fn evaluate(
        &self,
        regressor: &[f64],
        output: &mut [f64],
    ) -> Result<(), AdaptiveGravityError> {
        let active = self
            .active
            .ok_or(AdaptiveGravityError::InvalidState)?
            .as_array();
        if regressor.len() != self.order.len() * 4 || output.len() < self.order.len() {
            return Err(AdaptiveGravityError::DimensionMismatch);
        }
        for (row, value) in output[..self.order.len()].iter_mut().enumerate() {
            *value = dot4(&regressor[row * 4..row * 4 + 4], active);
            if !value.is_finite() || value.abs() > self.config.maximum_payload_effort {
                return Err(AdaptiveGravityError::OutputBound);
            }
        }
        Ok(())
    }

    fn gate(&self, context: AdaptiveSampleContext) -> Option<AdaptiveSkipReason> {
        if context.generation != self.generation {
            Some(AdaptiveSkipReason::IdentityMismatch)
        } else if !context.fresh {
            Some(AdaptiveSkipReason::Stale)
        } else if !context.settled {
            Some(AdaptiveSkipReason::Moving)
        } else if context.saturated {
            Some(AdaptiveSkipReason::Saturated)
        } else if !context.contact_free {
            Some(AdaptiveSkipReason::Contact)
        } else {
            None
        }
    }

    fn window_is_duplicate(&self, regressor: &[f64], dof: usize) -> bool {
        self.rows.chunks(dof).any(|window| {
            window.iter().enumerate().all(|(row, stored)| {
                let base = row * 4;
                stored
                    .phi
                    .iter()
                    .zip(&regressor[base..base + 4])
                    .all(|(left, right)| (left - right).abs() <= 1.0e-12)
            })
        })
    }

    fn project(&self, mut theta: [f64; 4]) -> Result<PayloadParameters, AdaptiveGravityError> {
        if theta.iter().any(|v| !v.is_finite()) {
            return Err(AdaptiveGravityError::NonFinite);
        }
        theta[0] = theta[0].clamp(self.config.mass_bounds_kg[0], self.config.mass_bounds_kg[1]);
        for axis in 0..3 {
            let cog = (theta[axis + 1] / theta[0]).clamp(
                self.config.cog_bounds_m[axis][0],
                self.config.cog_bounds_m[axis][1],
            );
            theta[axis + 1] = theta[0] * cog;
        }
        Ok(PayloadParameters(theta))
    }
}

/// Estimator failure that cannot be represented as a skipped update.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum AdaptiveGravityError {
    #[error("adaptive gravity configuration is invalid")]
    InvalidConfiguration,
    #[error("adaptive gravity operation is invalid in the current state")]
    InvalidState,
    #[error("adaptive gravity input dimensions differ from the prepared group")]
    DimensionMismatch,
    #[error("adaptive gravity bounded storage is full")]
    Capacity,
    #[error("adaptive gravity input or result is non-finite")]
    NonFinite,
    #[error("calibration poses do not provide a full-rank well-conditioned regressor")]
    Unobservable,
    #[error("candidate failed held-out validation")]
    ValidationFailed,
    #[error("adaptive effort exceeds the configured output bound")]
    OutputBound,
}

fn diagonal(value: f64) -> [[f64; 4]; 4] {
    [
        [value, 0.0, 0.0, 0.0],
        [0.0, value, 0.0, 0.0],
        [0.0, 0.0, value, 0.0],
        [0.0, 0.0, 0.0, value],
    ]
}

fn dot4(values: &[f64], theta: [f64; 4]) -> f64 {
    values[0] * theta[0] + values[1] * theta[1] + values[2] * theta[2] + values[3] * theta[3]
}
