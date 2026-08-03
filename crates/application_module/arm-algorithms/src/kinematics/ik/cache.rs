// SPDX-License-Identifier: Apache-2.0
//
// Adapted exclusively from softmata/kinetic
// crates/kinetic-kinematics/src/cached_ik.rs at commit
// 3dbaa61579c4359ce4c35f246fc83e1d3110b646.
// Local modifications: deterministic quantized key, profile/group/order/
// generation identity, fixed capacity, byte ceiling and no implicit eviction.

//! Bounded context-bound warm IK cache.

use message_model::{ArmJointOrder, ArmProfileId, KinematicGroupId};

use super::{ArmPose, IkSolution, PreparedKinematics};
use crate::ArmAlgorithmError;

/// Hard preparation-time entry ceiling for one IK cache.
pub const MAX_IK_CACHE_ENTRIES: usize = 4_096;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PoseKey([i64; 7]);

#[derive(Clone)]
struct CacheEntry {
    key: PoseKey,
    solution: IkSolution,
}

/// Fixed-capacity cache bound to one prepared morphology generation.
pub struct IkCache {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    generation: u64,
    translation_resolution_m: f64,
    rotation_resolution: f64,
    maximum_entries: usize,
    entries: Vec<CacheEntry>,
}

impl IkCache {
    /// Preallocates a cache within explicit entry and byte ceilings.
    pub fn new(
        prepared: &PreparedKinematics<'_>,
        maximum_entries: usize,
        maximum_bytes: usize,
        translation_resolution_m: f64,
        rotation_resolution: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if maximum_entries == 0 || maximum_entries > MAX_IK_CACHE_ENTRIES {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.cache_entries",
                actual: maximum_entries,
                maximum: MAX_IK_CACHE_ENTRIES,
            });
        }
        for (field, resolution) in [
            ("ik.cache_translation_resolution", translation_resolution_m),
            ("ik.cache_rotation_resolution", rotation_resolution),
        ] {
            if !resolution.is_finite() || resolution <= 0.0 {
                return Err(ArmAlgorithmError::NonFinite { field });
            }
        }
        let required =
            std::mem::size_of::<Self>() + maximum_entries * std::mem::size_of::<CacheEntry>();
        if required > maximum_bytes {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.cache_bytes",
                actual: required,
                maximum: maximum_bytes,
            });
        }
        Ok(Self {
            profile_id: prepared.profile_id(),
            group_id: prepared.group_id(),
            order: prepared.order().clone(),
            generation: prepared.generation(),
            translation_resolution_m,
            rotation_resolution,
            maximum_entries,
            entries: Vec::with_capacity(maximum_entries),
        })
    }

    /// Returns the current deterministic entry count.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns whether no pose has been inserted.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Looks up a cloned fixed-storage solution without mutating cache order.
    pub fn lookup(
        &self,
        prepared: &PreparedKinematics<'_>,
        target: ArmPose,
    ) -> Result<Option<IkSolution>, ArmAlgorithmError> {
        self.require_context(prepared)?;
        let key = self.key(target)?;
        Ok(self
            .entries
            .iter()
            .find(|entry| entry.key == key)
            .map(|entry| {
                let mut solution = entry.solution.clone();
                solution.work.cache_hit = true;
                solution
            }))
    }

    /// Inserts or replaces an identical quantized key. Full caches reject new
    /// keys rather than evicting an entry implicitly.
    pub fn insert(
        &mut self,
        prepared: &PreparedKinematics<'_>,
        target: ArmPose,
        solution: &IkSolution,
    ) -> Result<(), ArmAlgorithmError> {
        self.require_context(prepared)?;
        if solution.positions().order() != &self.order {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group_id,
            });
        }
        let key = self.key(target)?;
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.key == key) {
            entry.solution = solution.clone();
            return Ok(());
        }
        if self.entries.len() == self.maximum_entries {
            return Err(ArmAlgorithmError::Capacity {
                field: "ik.cache_entries",
                actual: self.entries.len() + 1,
                maximum: self.maximum_entries,
            });
        }
        self.entries.push(CacheEntry {
            key,
            solution: solution.clone(),
        });
        Ok(())
    }

    fn require_context(&self, prepared: &PreparedKinematics<'_>) -> Result<(), ArmAlgorithmError> {
        if self.generation != prepared.generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: prepared.generation(),
                actual: self.generation,
            });
        }
        if self.profile_id != prepared.profile_id()
            || self.group_id != prepared.group_id()
            || self.order != *prepared.order()
        {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "IK cache profile, group or order differs",
            });
        }
        Ok(())
    }

    fn key(&self, target: ArmPose) -> Result<PoseKey, ArmAlgorithmError> {
        let translation = target.translation_m();
        let mut quaternion = target.quaternion_xyzw();
        if quaternion[3] < 0.0
            || (quaternion[3] == 0.0
                && quaternion[..3]
                    .iter()
                    .copied()
                    .find(|value| *value != 0.0)
                    .is_some_and(|value| value < 0.0))
        {
            for value in &mut quaternion {
                *value = -*value;
            }
        }
        let values = [
            translation[0] / self.translation_resolution_m,
            translation[1] / self.translation_resolution_m,
            translation[2] / self.translation_resolution_m,
            quaternion[0] / self.rotation_resolution,
            quaternion[1] / self.rotation_resolution,
            quaternion[2] / self.rotation_resolution,
            quaternion[3] / self.rotation_resolution,
        ];
        let mut key = [0_i64; 7];
        for (output, value) in key.iter_mut().zip(values) {
            if !value.is_finite() || value < i64::MIN as f64 || value > i64::MAX as f64 {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "ik.cache_key",
                });
            }
            *output = value.round() as i64;
        }
        Ok(PoseKey(key))
    }
}
