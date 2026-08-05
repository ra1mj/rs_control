use arrayvec::ArrayVec;
use message_model::{DataQuality, Header, Id128, SourceId};
use thiserror::Error;

use crate::{NavigationIpcError, wire::validate_header_at};

/// Bounded source-ordering rejection counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NavigationGuardCounters {
    /// Foreign publisher identity.
    pub foreign_source: u64,
    /// Exact sequence replay.
    pub duplicate: u64,
    /// Sequence moved backwards in the current boot.
    pub reordered: u64,
    /// A retired boot attempted to return.
    pub prior_boot: u64,
    /// Header validity/quality rejection.
    pub freshness: u64,
}

/// Source/boot/sequence/freshness admission error.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum NavigationGuardError {
    /// Stable source identity did not match the configured writer.
    #[error("foreign navigation IPC source")]
    ForeignSource,
    /// The last accepted sequence was repeated.
    #[error("duplicate navigation IPC sequence")]
    Duplicate,
    /// A sequence moved backwards within one boot.
    #[error("reordered navigation IPC sequence")]
    Reordered,
    /// A previously retired boot identity attempted to return.
    #[error("prior navigation IPC boot identity")]
    PriorBoot,
    /// Header was expired, overflowing or not valid quality.
    #[error("unusable navigation IPC freshness/quality")]
    Freshness,
}

/// Single-writer guard with fixed retired-boot history.
pub struct NavigationSourceGuard<const RETIRED_BOOTS: usize = 4> {
    expected_source: SourceId,
    active: Option<(Id128, u64)>,
    retired: ArrayVec<Id128, RETIRED_BOOTS>,
    counters: NavigationGuardCounters,
}

impl<const RETIRED_BOOTS: usize> NavigationSourceGuard<RETIRED_BOOTS> {
    /// Creates an empty guard for one exact stable publisher identity.
    #[must_use]
    pub const fn new(expected_source: SourceId) -> Self {
        Self {
            expected_source,
            active: None,
            retired: ArrayVec::new_const(),
            counters: NavigationGuardCounters {
                foreign_source: 0,
                duplicate: 0,
                reordered: 0,
                prior_boot: 0,
                freshness: 0,
            },
        }
    }

    /// Validates and advances source/boot/sequence state.
    pub fn accept(
        &mut self,
        header: Header,
        now_monotonic_ns: u64,
    ) -> Result<(), NavigationGuardError> {
        if header.source_id() != self.expected_source {
            saturating_increment(&mut self.counters.foreign_source);
            return Err(NavigationGuardError::ForeignSource);
        }
        if validate_header_at(header, now_monotonic_ns).is_err()
            || header.quality() != DataQuality::Valid
        {
            saturating_increment(&mut self.counters.freshness);
            return Err(NavigationGuardError::Freshness);
        }
        let boot = header.boot_id();
        if self.retired.contains(&boot) {
            saturating_increment(&mut self.counters.prior_boot);
            return Err(NavigationGuardError::PriorBoot);
        }
        match self.active {
            Some((active_boot, last_sequence)) if active_boot == boot => {
                if header.sequence() == last_sequence {
                    saturating_increment(&mut self.counters.duplicate);
                    return Err(NavigationGuardError::Duplicate);
                }
                if header.sequence() < last_sequence {
                    saturating_increment(&mut self.counters.reordered);
                    return Err(NavigationGuardError::Reordered);
                }
            }
            Some((active_boot, _)) => self.retire(active_boot),
            None => {}
        }
        self.active = Some((boot, header.sequence()));
        Ok(())
    }

    /// Retires the active boot after peer loss so its backlog cannot return.
    pub fn disconnect(&mut self) {
        if let Some((boot, _)) = self.active.take() {
            self.retire(boot);
        }
    }

    /// Returns bounded saturating observations.
    #[must_use]
    pub const fn counters(&self) -> NavigationGuardCounters {
        self.counters
    }

    fn retire(&mut self, boot: Id128) {
        if RETIRED_BOOTS == 0 || self.retired.contains(&boot) {
            return;
        }
        if self.retired.is_full() {
            let _ = self.retired.remove(0);
        }
        self.retired.push(boot);
    }
}

fn saturating_increment(value: &mut u64) {
    *value = value.saturating_add(1);
}

impl From<NavigationIpcError> for NavigationGuardError {
    fn from(_: NavigationIpcError) -> Self {
        Self::Freshness
    }
}
