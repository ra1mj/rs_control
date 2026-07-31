//! Fixed-capacity local enforcement for exclusive resource leases.

use arrayvec::ArrayVec;
use message_model::{CommandEnvelope, Id128, ResourceId, ResourceLease, SemanticError};
use thiserror::Error;

/// Exact identity used by release, preempt, and safe-confirm events.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LeaseKey {
    resource_id: ResourceId,
    lease_id: Id128,
    session_epoch: u64,
}

impl LeaseKey {
    /// Constructs a key with an explicit nonzero session epoch.
    pub fn new(
        resource_id: ResourceId,
        lease_id: Id128,
        session_epoch: u64,
    ) -> Result<Self, LeaseCacheError> {
        if session_epoch == 0 {
            return Err(LeaseCacheError::ZeroSessionEpoch);
        }
        Ok(Self {
            resource_id,
            lease_id,
            session_epoch,
        })
    }

    /// Extracts the exact key of a validated lease.
    #[must_use]
    pub const fn from_lease(lease: ResourceLease) -> Self {
        Self {
            resource_id: lease.resource_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
        }
    }

    /// Returns the exclusive resource.
    #[must_use]
    pub const fn resource_id(self) -> ResourceId {
        self.resource_id
    }

    /// Returns the unique lease identity.
    #[must_use]
    pub const fn lease_id(self) -> Id128 {
        self.lease_id
    }

    /// Returns the authority-service session.
    #[must_use]
    pub const fn session_epoch(self) -> u64 {
        self.session_epoch
    }
}

/// Deterministic authority event consumed by the motion enforcement owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseEvent {
    /// Starts a strictly newer authority-service session.
    StartSession {
        /// New nonzero epoch.
        session_epoch: u64,
    },
    /// Activates a lease on an unoccupied resource.
    Grant(ResourceLease),
    /// Extends the exact active lease.
    Renew(ResourceLease),
    /// Removes exact active or pending authority.
    Release(LeaseKey),
    /// Revokes the incumbent and stores a replacement pending safe confirmation.
    Preempt {
        /// Exact active authority being revoked.
        current: LeaseKey,
        /// Replacement for the same resource and current session.
        replacement: ResourceLease,
    },
    /// Activates an exact pending replacement after the resource is safe.
    ConfirmSafe(LeaseKey),
    /// Invalidates one exact current authority-service session.
    SessionLost {
        /// Session observed lost.
        session_epoch: u64,
    },
    /// Removes every entry expired at the supplied `apply` time.
    Tick,
}

/// Stable classification of one successful reducer step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseEffectKind {
    /// A newer authority session became current.
    SessionStarted,
    /// A lease became active.
    Granted,
    /// An exact active lease expiration was extended.
    Renewed,
    /// Exact authority was explicitly released.
    Released,
    /// The incumbent was revoked and its replacement awaits safe confirmation.
    PreemptPending,
    /// A pending replacement became active.
    Activated,
    /// One or more expired entries were removed.
    Expired,
    /// The current authority service was lost.
    SessionLost,
    /// A tick found no expired entries.
    Noop,
}

/// Why an active or pending entry left the cache.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseRemovalReason {
    /// Explicit release.
    Released,
    /// Exclusive local-monotonic expiry.
    Expired,
    /// Revoked by a two-phase takeover.
    Preempted,
    /// Authority-service loss.
    SessionLost,
    /// A strictly newer authority-service session replaced the prior session.
    SessionReplaced,
}

/// Typed removal evidence returned to the safety/diagnostic owner.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LeaseRemoval {
    lease: ResourceLease,
    reason: LeaseRemovalReason,
    was_active: bool,
}

impl LeaseRemoval {
    /// Returns the removed authority.
    #[must_use]
    pub const fn lease(self) -> ResourceLease {
        self.lease
    }

    /// Returns why the entry was removed.
    #[must_use]
    pub const fn reason(self) -> LeaseRemovalReason {
        self.reason
    }

    /// Returns whether the entry could authorize commands before removal.
    #[must_use]
    pub const fn was_active(self) -> bool {
        self.was_active
    }
}

/// Fixed-capacity result of one applied lease event.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LeaseEffect<const CAPACITY: usize> {
    kind: LeaseEffectKind,
    lease: Option<ResourceLease>,
    removals: ArrayVec<LeaseRemoval, CAPACITY>,
}

impl<const CAPACITY: usize> LeaseEffect<CAPACITY> {
    fn new(kind: LeaseEffectKind) -> Self {
        Self {
            kind,
            lease: None,
            removals: ArrayVec::new(),
        }
    }

    fn with_lease(kind: LeaseEffectKind, lease: ResourceLease) -> Self {
        Self {
            kind,
            lease: Some(lease),
            removals: ArrayVec::new(),
        }
    }

    fn push_removal(&mut self, removal: LeaseRemoval) {
        self.removals
            .try_push(removal)
            .expect("effect capacity matches cache capacity");
    }

    /// Returns the transition classification.
    #[must_use]
    pub const fn kind(&self) -> LeaseEffectKind {
        self.kind
    }

    /// Returns the granted, renewed, pending, or activated lease when present.
    #[must_use]
    pub const fn lease(&self) -> Option<ResourceLease> {
        self.lease
    }

    /// Returns bounded removal evidence in deterministic cache order.
    #[must_use]
    pub fn removals(&self) -> &[LeaseRemoval] {
        self.removals.as_slice()
    }
}

/// Enforcement state exposed for a resource.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaseStatus {
    /// Commands may be authorized while lease and command deadlines remain valid.
    Active,
    /// Replacement exists but commands remain inhibited until safe confirmation.
    AwaitingSafe,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum LeaseEntryState {
    Active(ResourceLease),
    AwaitingSafe(ResourceLease),
}

impl LeaseEntryState {
    const fn lease(self) -> ResourceLease {
        match self {
            Self::Active(lease) | Self::AwaitingSafe(lease) => lease,
        }
    }

    const fn status(self) -> LeaseStatus {
        match self {
            Self::Active(_) => LeaseStatus::Active,
            Self::AwaitingSafe(_) => LeaseStatus::AwaitingSafe,
        }
    }

    const fn was_active(self) -> bool {
        matches!(self, Self::Active(_))
    }
}

/// Single-owner fixed-capacity authority cache for a motion runtime.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct LeaseEnforcementCache<const CAPACITY: usize> {
    highest_session_epoch: u64,
    current_session_epoch: Option<u64>,
    entries: ArrayVec<LeaseEntryState, CAPACITY>,
}

impl<const CAPACITY: usize> Default for LeaseEnforcementCache<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> LeaseEnforcementCache<CAPACITY> {
    /// Creates an empty cache with no authorized authority-service session.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            highest_session_epoch: 0,
            current_session_epoch: None,
            entries: ArrayVec::new_const(),
        }
    }

    /// Returns the current session, or `None` after startup/loss.
    #[must_use]
    pub const fn current_session_epoch(&self) -> Option<u64> {
        self.current_session_epoch
    }

    /// Returns the highest session epoch ever accepted.
    #[must_use]
    pub const fn highest_session_epoch(&self) -> u64 {
        self.highest_session_epoch
    }

    /// Returns the number of active or pending resource entries.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns whether the cache contains no active or pending authority.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Returns one resource's current enforcement status.
    #[must_use]
    pub fn status(&self, resource_id: ResourceId) -> Option<LeaseStatus> {
        self.entry_index(resource_id)
            .map(|index| self.entries[index].status())
    }

    /// Returns a currently active and unexpired lease.
    #[must_use]
    pub fn active_lease(
        &self,
        resource_id: ResourceId,
        now_monotonic_ns: u64,
    ) -> Option<ResourceLease> {
        let entry = self.entries.get(self.entry_index(resource_id)?)?;
        match *entry {
            LeaseEntryState::Active(lease)
                if Some(lease.session_epoch()) == self.current_session_epoch
                    && lease.validate_at(now_monotonic_ns).is_ok() =>
            {
                Some(lease)
            }
            LeaseEntryState::Active(_) | LeaseEntryState::AwaitingSafe(_) => None,
        }
    }

    /// Applies one event at an explicit local-monotonic enforcement time.
    pub fn apply(
        &mut self,
        event: LeaseEvent,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        match event {
            LeaseEvent::StartSession { session_epoch } => self.start_session(session_epoch),
            LeaseEvent::Grant(lease) => self.grant(lease, now_monotonic_ns),
            LeaseEvent::Renew(lease) => self.renew(lease, now_monotonic_ns),
            LeaseEvent::Release(key) => self.release(key),
            LeaseEvent::Preempt {
                current,
                replacement,
            } => self.preempt(current, replacement, now_monotonic_ns),
            LeaseEvent::ConfirmSafe(key) => self.confirm_safe(key, now_monotonic_ns),
            LeaseEvent::SessionLost { session_epoch } => self.session_lost(session_epoch),
            LeaseEvent::Tick => Ok(self.expire(now_monotonic_ns)),
        }
    }

    /// Authorizes an exact command without changing cache state.
    pub fn authorize(
        &self,
        command: &CommandEnvelope,
        now_monotonic_ns: u64,
    ) -> Result<ResourceLease, LeaseCacheError> {
        command.validate_at(now_monotonic_ns)?;
        self.require_current_epoch(command.session_epoch())?;
        let index = self
            .entry_index(command.resource_id())
            .ok_or(LeaseCacheError::ResourceNotFound)?;
        let lease = match self.entries[index] {
            LeaseEntryState::Active(lease) => lease,
            LeaseEntryState::AwaitingSafe(_) => return Err(LeaseCacheError::LeaseNotActive),
        };
        lease.validate_at(now_monotonic_ns)?;
        if command.owner_id() != lease.owner_id() {
            return Err(LeaseCacheError::AuthorityMismatch { field: "owner_id" });
        }
        if command.lease_id() != lease.lease_id() {
            return Err(LeaseCacheError::AuthorityMismatch { field: "lease_id" });
        }
        if command.session_epoch() != lease.session_epoch() {
            return Err(LeaseCacheError::AuthorityMismatch {
                field: "session_epoch",
            });
        }
        Ok(lease)
    }

    fn start_session(
        &mut self,
        session_epoch: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        if session_epoch == 0 {
            return Err(LeaseCacheError::ZeroSessionEpoch);
        }
        if session_epoch <= self.highest_session_epoch {
            return Err(LeaseCacheError::NonIncreasingSession {
                highest: self.highest_session_epoch,
                proposed: session_epoch,
            });
        }
        let mut effect = self.remove_all(
            LeaseEffectKind::SessionStarted,
            LeaseRemovalReason::SessionReplaced,
        );
        self.highest_session_epoch = session_epoch;
        self.current_session_epoch = Some(session_epoch);
        effect.lease = None;
        Ok(effect)
    }

    fn grant(
        &mut self,
        lease: ResourceLease,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(lease.session_epoch())?;
        lease.validate_at(now_monotonic_ns)?;
        if self.entry_index(lease.resource_id()).is_some() {
            return Err(LeaseCacheError::ResourceOccupied);
        }
        self.entries
            .try_push(LeaseEntryState::Active(lease))
            .map_err(|_| LeaseCacheError::Capacity { maximum: CAPACITY })?;
        Ok(LeaseEffect::with_lease(LeaseEffectKind::Granted, lease))
    }

    fn renew(
        &mut self,
        lease: ResourceLease,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(lease.session_epoch())?;
        lease.validate_at(now_monotonic_ns)?;
        let index = self
            .entry_index(lease.resource_id())
            .ok_or(LeaseCacheError::ResourceNotFound)?;
        let current = match self.entries[index] {
            LeaseEntryState::Active(current) => current,
            LeaseEntryState::AwaitingSafe(_) => return Err(LeaseCacheError::LeaseNotActive),
        };
        require_same_lease(current, lease)?;
        if lease.expires_monotonic_ns() <= current.expires_monotonic_ns() {
            return Err(LeaseCacheError::RenewalMustExtend {
                current: current.expires_monotonic_ns(),
                proposed: lease.expires_monotonic_ns(),
            });
        }
        self.entries[index] = LeaseEntryState::Active(lease);
        Ok(LeaseEffect::with_lease(LeaseEffectKind::Renewed, lease))
    }

    fn release(&mut self, key: LeaseKey) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(key.session_epoch)?;
        let index = self
            .entry_index(key.resource_id)
            .ok_or(LeaseCacheError::ResourceNotFound)?;
        require_key(self.entries[index].lease(), key)?;
        let removed = self.entries.remove(index);
        let mut effect = LeaseEffect::new(LeaseEffectKind::Released);
        effect.push_removal(removal(removed, LeaseRemovalReason::Released));
        Ok(effect)
    }

    fn preempt(
        &mut self,
        current: LeaseKey,
        replacement: ResourceLease,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(current.session_epoch)?;
        self.require_current_epoch(replacement.session_epoch())?;
        replacement.validate_at(now_monotonic_ns)?;
        let index = self
            .entry_index(current.resource_id)
            .ok_or(LeaseCacheError::ResourceNotFound)?;
        let incumbent = match self.entries[index] {
            LeaseEntryState::Active(lease) => lease,
            LeaseEntryState::AwaitingSafe(_) => return Err(LeaseCacheError::LeaseNotActive),
        };
        require_key(incumbent, current)?;
        if replacement.resource_id() != incumbent.resource_id() {
            return Err(LeaseCacheError::ReplacementResourceMismatch);
        }
        if replacement.lease_id() == incumbent.lease_id() {
            return Err(LeaseCacheError::ReplacementLeaseMustDiffer);
        }
        self.entries[index] = LeaseEntryState::AwaitingSafe(replacement);
        let mut effect = LeaseEffect::with_lease(LeaseEffectKind::PreemptPending, replacement);
        effect.push_removal(LeaseRemoval {
            lease: incumbent,
            reason: LeaseRemovalReason::Preempted,
            was_active: true,
        });
        Ok(effect)
    }

    fn confirm_safe(
        &mut self,
        key: LeaseKey,
        now_monotonic_ns: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(key.session_epoch)?;
        let index = self
            .entry_index(key.resource_id)
            .ok_or(LeaseCacheError::ResourceNotFound)?;
        let replacement = match self.entries[index] {
            LeaseEntryState::Active(_) => return Err(LeaseCacheError::LeaseNotAwaitingSafe),
            LeaseEntryState::AwaitingSafe(lease) => lease,
        };
        require_key(replacement, key)?;
        replacement.validate_at(now_monotonic_ns)?;
        self.entries[index] = LeaseEntryState::Active(replacement);
        Ok(LeaseEffect::with_lease(
            LeaseEffectKind::Activated,
            replacement,
        ))
    }

    fn session_lost(
        &mut self,
        session_epoch: u64,
    ) -> Result<LeaseEffect<CAPACITY>, LeaseCacheError> {
        self.require_current_epoch(session_epoch)?;
        let effect = self.remove_all(
            LeaseEffectKind::SessionLost,
            LeaseRemovalReason::SessionLost,
        );
        self.current_session_epoch = None;
        Ok(effect)
    }

    fn expire(&mut self, now_monotonic_ns: u64) -> LeaseEffect<CAPACITY> {
        let mut effect = LeaseEffect::new(LeaseEffectKind::Noop);
        let mut index = 0;
        while index < self.entries.len() {
            if self.entries[index].lease().expires_monotonic_ns() <= now_monotonic_ns {
                let removed = self.entries.remove(index);
                effect.push_removal(removal(removed, LeaseRemovalReason::Expired));
            } else {
                index += 1;
            }
        }
        if !effect.removals.is_empty() {
            effect.kind = LeaseEffectKind::Expired;
        }
        effect
    }

    fn remove_all(
        &mut self,
        kind: LeaseEffectKind,
        reason: LeaseRemovalReason,
    ) -> LeaseEffect<CAPACITY> {
        let mut effect = LeaseEffect::new(kind);
        for entry in &self.entries {
            effect.push_removal(removal(*entry, reason));
        }
        self.entries.clear();
        effect
    }

    fn entry_index(&self, resource_id: ResourceId) -> Option<usize> {
        self.entries
            .iter()
            .position(|entry| entry.lease().resource_id() == resource_id)
    }

    fn require_current_epoch(&self, provided: u64) -> Result<(), LeaseCacheError> {
        if provided == 0 {
            return Err(LeaseCacheError::ZeroSessionEpoch);
        }
        match self.current_session_epoch {
            Some(current) if current == provided => Ok(()),
            Some(current) => Err(LeaseCacheError::SessionMismatch { current, provided }),
            None => Err(LeaseCacheError::NoCurrentSession),
        }
    }
}

fn require_same_lease(
    current: ResourceLease,
    proposed: ResourceLease,
) -> Result<(), LeaseCacheError> {
    if proposed.owner_id() != current.owner_id() {
        return Err(LeaseCacheError::LeaseMismatch { field: "owner_id" });
    }
    if proposed.lease_id() != current.lease_id() {
        return Err(LeaseCacheError::LeaseMismatch { field: "lease_id" });
    }
    if proposed.session_epoch() != current.session_epoch() {
        return Err(LeaseCacheError::LeaseMismatch {
            field: "session_epoch",
        });
    }
    Ok(())
}

fn require_key(lease: ResourceLease, key: LeaseKey) -> Result<(), LeaseCacheError> {
    if lease.resource_id() != key.resource_id {
        return Err(LeaseCacheError::LeaseMismatch {
            field: "resource_id",
        });
    }
    if lease.lease_id() != key.lease_id {
        return Err(LeaseCacheError::LeaseMismatch { field: "lease_id" });
    }
    if lease.session_epoch() != key.session_epoch {
        return Err(LeaseCacheError::LeaseMismatch {
            field: "session_epoch",
        });
    }
    Ok(())
}

const fn removal(entry: LeaseEntryState, reason: LeaseRemovalReason) -> LeaseRemoval {
    LeaseRemoval {
        lease: entry.lease(),
        reason,
        was_active: entry.was_active(),
    }
}

/// Rejected authority event or command.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum LeaseCacheError {
    /// Session epoch zero is reserved.
    #[error("session epoch must be greater than zero")]
    ZeroSessionEpoch,
    /// A session epoch did not advance past all prior accepted epochs.
    #[error("session epoch {proposed} must be greater than prior maximum {highest}")]
    NonIncreasingSession {
        /// Highest epoch previously accepted.
        highest: u64,
        /// Rejected epoch.
        proposed: u64,
    },
    /// No authority-service session is currently available.
    #[error("authority service has no current session")]
    NoCurrentSession,
    /// Event or command belonged to a different session.
    #[error("session epoch {provided} does not match current epoch {current}")]
    SessionMismatch {
        /// Current session.
        current: u64,
        /// Supplied session.
        provided: u64,
    },
    /// Fixed resource capacity was exhausted.
    #[error("lease cache capacity {maximum} is exhausted")]
    Capacity {
        /// Configured fixed capacity.
        maximum: usize,
    },
    /// A grant attempted implicit last-writer-wins replacement.
    #[error("resource already has active or pending authority")]
    ResourceOccupied,
    /// No entry exists for the requested resource.
    #[error("resource has no active or pending authority")]
    ResourceNotFound,
    /// A renewal/release/preempt identity field differed.
    #[error("lease {field} does not match the cached authority")]
    LeaseMismatch {
        /// Mismatched field.
        field: &'static str,
    },
    /// Operation required active authority but the entry was pending safe confirmation.
    #[error("lease does not currently authorize commands")]
    LeaseNotActive,
    /// Safe confirmation was supplied for an already-active entry.
    #[error("lease is not awaiting safe confirmation")]
    LeaseNotAwaitingSafe,
    /// A renewal did not strictly increase the expiration.
    #[error("renewal expiration {proposed} must be greater than {current}")]
    RenewalMustExtend {
        /// Existing expiration.
        current: u64,
        /// Proposed expiration.
        proposed: u64,
    },
    /// A replacement targeted a different exclusive resource.
    #[error("preemption replacement must target the incumbent resource")]
    ReplacementResourceMismatch,
    /// A preemption attempted to reuse the incumbent grant identity.
    #[error("preemption replacement requires a distinct lease identity")]
    ReplacementLeaseMustDiffer,
    /// Command authority fields did not match the active lease.
    #[error("command {field} does not match active authority")]
    AuthorityMismatch {
        /// Mismatched field.
        field: &'static str,
    },
    /// Validated lease/command time semantics rejected the operation.
    #[error("authority semantic validation failed: {0}")]
    Semantic(#[from] SemanticError),
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use message_model::{
        AsciiId, ClockState, ClockStatus, CommandEnvelopeFields, DataQuality, Header, HeaderFields,
        Id128, ResourceLeaseFields,
    };

    use super::*;

    fn id(value: &str) -> ResourceId {
        AsciiId::parse(value).unwrap()
    }

    fn token(value: u8) -> Id128 {
        Id128::new([value; 16]).unwrap()
    }

    fn lease(
        resource: &str,
        owner: &str,
        lease_token: u8,
        epoch: u64,
        expires: u64,
    ) -> ResourceLease {
        ResourceLease::new(ResourceLeaseFields {
            resource_id: id(resource),
            owner_id: id(owner),
            lease_id: token(lease_token),
            session_epoch: epoch,
            expires_monotonic_ns: expires,
        })
        .unwrap()
    }

    fn command(
        authority: ResourceLease,
        owner: &str,
        lease_token: u8,
        epoch: u64,
        deadline: u64,
    ) -> CommandEnvelope {
        CommandEnvelope::new(CommandEnvelopeFields {
            header: Header::new(HeaderFields {
                source_id: id(owner),
                boot_id: token(90),
                sequence: 1,
                source_monotonic_ns: 1,
                publish_unix_ns: None,
                acquisition_timestamp: None,
                valid_for_ns: 100,
                frame_id: None,
                clock_status: ClockStatus {
                    state: ClockState::Local,
                    uncertainty_ns: 0,
                },
                quality: DataQuality::Valid,
            })
            .unwrap(),
            resource_id: authority.resource_id(),
            owner_id: id(owner),
            lease_id: token(lease_token),
            session_epoch: epoch,
            command_id: token(91),
            deadline_monotonic_ns: deadline,
        })
        .unwrap()
    }

    #[test]
    fn grant_renew_release_and_capacity_are_exact() {
        let mut cache = LeaseEnforcementCache::<1>::new();
        assert_eq!(
            cache.apply(LeaseEvent::StartSession { session_epoch: 1 }, 10),
            Ok(LeaseEffect::new(LeaseEffectKind::SessionStarted))
        );
        let first = lease("joint", "owner-a", 1, 1, 100);
        assert_eq!(
            cache.apply(LeaseEvent::Grant(first), 10).unwrap().kind(),
            LeaseEffectKind::Granted
        );
        assert_eq!(cache.status(id("joint")), Some(LeaseStatus::Active));
        assert_eq!(
            cache.apply(LeaseEvent::Grant(lease("joint", "owner-b", 2, 1, 100)), 10),
            Err(LeaseCacheError::ResourceOccupied)
        );
        assert_eq!(
            cache.apply(LeaseEvent::Grant(lease("other", "owner-b", 2, 1, 100)), 10),
            Err(LeaseCacheError::Capacity { maximum: 1 })
        );
        assert_eq!(
            cache.apply(LeaseEvent::Renew(lease("joint", "owner-a", 2, 1, 120)), 10),
            Err(LeaseCacheError::LeaseMismatch { field: "lease_id" })
        );
        assert_eq!(
            cache.apply(LeaseEvent::Renew(first), 10),
            Err(LeaseCacheError::RenewalMustExtend {
                current: 100,
                proposed: 100
            })
        );
        let renewed = lease("joint", "owner-a", 1, 1, 150);
        assert_eq!(
            cache.apply(LeaseEvent::Renew(renewed), 10).unwrap().lease(),
            Some(renewed)
        );
        let effect = cache
            .apply(LeaseEvent::Release(LeaseKey::from_lease(renewed)), 10)
            .unwrap();
        assert_eq!(effect.kind(), LeaseEffectKind::Released);
        assert_eq!(
            effect.removals(),
            &[LeaseRemoval {
                lease: renewed,
                reason: LeaseRemovalReason::Released,
                was_active: true
            }]
        );
        assert!(cache.is_empty());

        let mut zero = LeaseEnforcementCache::<0>::new();
        zero.apply(LeaseEvent::StartSession { session_epoch: 1 }, 10)
            .unwrap();
        assert_eq!(
            zero.apply(LeaseEvent::Grant(first), 10),
            Err(LeaseCacheError::Capacity { maximum: 0 })
        );
    }

    #[test]
    fn session_and_lease_event_validation_fail_closed() {
        assert_eq!(
            LeaseKey::new(id("joint"), token(1), 0),
            Err(LeaseCacheError::ZeroSessionEpoch)
        );

        let mut cache = LeaseEnforcementCache::<1>::new();
        let current = lease("joint", "owner-a", 1, 2, 100);
        assert_eq!(
            cache.apply(LeaseEvent::Grant(current), 10),
            Err(LeaseCacheError::NoCurrentSession)
        );
        assert_eq!(
            cache.apply(LeaseEvent::StartSession { session_epoch: 0 }, 10),
            Err(LeaseCacheError::ZeroSessionEpoch)
        );
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 2 }, 10)
            .unwrap();
        assert_eq!(cache.highest_session_epoch(), 2);
        assert_eq!(
            cache.apply(LeaseEvent::StartSession { session_epoch: 1 }, 10),
            Err(LeaseCacheError::NonIncreasingSession {
                highest: 2,
                proposed: 1,
            })
        );
        assert_eq!(
            cache.apply(LeaseEvent::Grant(lease("joint", "owner-a", 1, 3, 100)), 10),
            Err(LeaseCacheError::SessionMismatch {
                current: 2,
                provided: 3,
            })
        );
        assert_eq!(
            cache.apply(LeaseEvent::Grant(lease("joint", "owner-a", 1, 2, 10)), 10),
            Err(LeaseCacheError::Semantic(SemanticError::Expired {
                deadline_ns: 10,
                now_ns: 10,
            }))
        );

        cache.apply(LeaseEvent::Grant(current), 10).unwrap();
        assert_eq!(
            cache.apply(LeaseEvent::Renew(lease("joint", "owner-b", 1, 2, 120)), 20),
            Err(LeaseCacheError::LeaseMismatch { field: "owner_id" })
        );
        assert_eq!(
            cache.apply(LeaseEvent::ConfirmSafe(LeaseKey::from_lease(current)), 20),
            Err(LeaseCacheError::LeaseNotAwaitingSafe)
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Release(LeaseKey::new(id("joint"), token(2), 2).unwrap()),
                20
            ),
            Err(LeaseCacheError::LeaseMismatch { field: "lease_id" })
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Release(LeaseKey::new(id("other"), token(1), 2).unwrap()),
                20
            ),
            Err(LeaseCacheError::ResourceNotFound)
        );
    }

    #[test]
    fn expiry_is_exclusive_and_returns_active_and_pending_evidence() {
        let mut cache = LeaseEnforcementCache::<2>::new();
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 3 }, 10)
            .unwrap();
        let active = lease("joint-a", "owner-a", 1, 3, 100);
        let old = lease("joint-b", "owner-a", 2, 3, 80);
        let pending = lease("joint-b", "owner-b", 3, 3, 100);
        cache.apply(LeaseEvent::Grant(active), 10).unwrap();
        cache.apply(LeaseEvent::Grant(old), 10).unwrap();
        cache
            .apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(old),
                    replacement: pending,
                },
                10,
            )
            .unwrap();
        assert_eq!(cache.active_lease(id("joint-a"), 99), Some(active));
        assert_eq!(
            cache.apply(LeaseEvent::Tick, 99).unwrap().kind(),
            LeaseEffectKind::Noop
        );
        let effect = cache.apply(LeaseEvent::Tick, 100).unwrap();
        assert_eq!(effect.kind(), LeaseEffectKind::Expired);
        assert_eq!(
            effect.removals(),
            &[
                LeaseRemoval {
                    lease: active,
                    reason: LeaseRemovalReason::Expired,
                    was_active: true,
                },
                LeaseRemoval {
                    lease: pending,
                    reason: LeaseRemovalReason::Expired,
                    was_active: false,
                }
            ]
        );
        assert!(cache.is_empty());
    }

    #[test]
    fn invalid_preemption_is_rejected_and_pending_release_is_explicit() {
        let mut cache = LeaseEnforcementCache::<1>::new();
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 4 }, 10)
            .unwrap();
        let incumbent = lease("joint", "owner-a", 1, 4, 200);
        cache.apply(LeaseEvent::Grant(incumbent), 10).unwrap();

        assert_eq!(
            cache.apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::new(id("joint"), token(9), 4).unwrap(),
                    replacement: lease("joint", "owner-b", 2, 4, 200),
                },
                20
            ),
            Err(LeaseCacheError::LeaseMismatch { field: "lease_id" })
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(incumbent),
                    replacement: lease("other", "owner-b", 2, 4, 200),
                },
                20
            ),
            Err(LeaseCacheError::ReplacementResourceMismatch)
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(incumbent),
                    replacement: lease("joint", "owner-b", 1, 4, 200),
                },
                20
            ),
            Err(LeaseCacheError::ReplacementLeaseMustDiffer)
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(incumbent),
                    replacement: lease("joint", "owner-b", 2, 5, 200),
                },
                20
            ),
            Err(LeaseCacheError::SessionMismatch {
                current: 4,
                provided: 5,
            })
        );

        let pending = lease("joint", "owner-b", 2, 4, 200);
        cache
            .apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(incumbent),
                    replacement: pending,
                },
                20,
            )
            .unwrap();
        assert_eq!(
            cache.apply(LeaseEvent::Renew(lease("joint", "owner-b", 2, 4, 250)), 21),
            Err(LeaseCacheError::LeaseNotActive)
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(pending),
                    replacement: lease("joint", "owner-c", 3, 4, 250),
                },
                21
            ),
            Err(LeaseCacheError::LeaseNotActive)
        );

        let released = cache
            .apply(LeaseEvent::Release(LeaseKey::from_lease(pending)), 21)
            .unwrap();
        assert_eq!(
            released.removals(),
            &[LeaseRemoval {
                lease: pending,
                reason: LeaseRemovalReason::Released,
                was_active: false,
            }]
        );
    }

    #[test]
    fn preemption_inhibits_both_owners_until_exact_safe_confirmation() {
        let mut cache = LeaseEnforcementCache::<1>::new();
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 5 }, 10)
            .unwrap();
        let incumbent = lease("joint", "manual", 1, 5, 200);
        let replacement = lease("joint", "mission", 2, 5, 200);
        cache.apply(LeaseEvent::Grant(incumbent), 10).unwrap();
        let old_command = command(incumbent, "manual", 1, 5, 180);
        let new_command = command(replacement, "mission", 2, 5, 180);
        assert_eq!(cache.authorize(&old_command, 20), Ok(incumbent));

        let effect = cache
            .apply(
                LeaseEvent::Preempt {
                    current: LeaseKey::from_lease(incumbent),
                    replacement,
                },
                20,
            )
            .unwrap();
        assert_eq!(effect.kind(), LeaseEffectKind::PreemptPending);
        assert_eq!(
            effect.removals(),
            &[LeaseRemoval {
                lease: incumbent,
                reason: LeaseRemovalReason::Preempted,
                was_active: true
            }]
        );
        assert_eq!(cache.status(id("joint")), Some(LeaseStatus::AwaitingSafe));
        assert_eq!(
            cache.authorize(&old_command, 21),
            Err(LeaseCacheError::LeaseNotActive)
        );
        assert_eq!(
            cache.authorize(&new_command, 21),
            Err(LeaseCacheError::LeaseNotActive)
        );
        assert_eq!(
            cache.apply(
                LeaseEvent::ConfirmSafe(LeaseKey::new(id("joint"), token(3), 5).unwrap()),
                21
            ),
            Err(LeaseCacheError::LeaseMismatch { field: "lease_id" })
        );
        assert_eq!(
            cache
                .apply(
                    LeaseEvent::ConfirmSafe(LeaseKey::from_lease(replacement)),
                    21
                )
                .unwrap()
                .kind(),
            LeaseEffectKind::Activated
        );
        assert_eq!(
            cache.authorize(&old_command, 22),
            Err(LeaseCacheError::AuthorityMismatch { field: "owner_id" })
        );
        assert_eq!(cache.authorize(&new_command, 22), Ok(replacement));
    }

    #[test]
    fn interleaved_competing_commands_never_replace_authority() {
        let mut cache = LeaseEnforcementCache::<1>::new();
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 6 }, 10)
            .unwrap();
        let active = lease("joint", "owner-a", 1, 6, 100);
        cache.apply(LeaseEvent::Grant(active), 10).unwrap();

        let accepted = command(active, "owner-a", 1, 6, 90);
        let foreign_owner = command(active, "owner-b", 1, 6, 90);
        let foreign_lease = command(active, "owner-a", 2, 6, 90);
        let foreign_resource_lease = lease("other", "owner-a", 1, 6, 100);
        let foreign_resource = command(foreign_resource_lease, "owner-a", 1, 6, 90);
        let prior_session = command(active, "owner-a", 1, 5, 90);

        for now in 20..24 {
            assert_eq!(
                cache.authorize(&foreign_owner, now),
                Err(LeaseCacheError::AuthorityMismatch { field: "owner_id" })
            );
            assert_eq!(cache.authorize(&accepted, now), Ok(active));
            assert_eq!(
                cache.authorize(&foreign_lease, now),
                Err(LeaseCacheError::AuthorityMismatch { field: "lease_id" })
            );
            assert_eq!(cache.authorize(&accepted, now), Ok(active));
            assert_eq!(
                cache.authorize(&foreign_resource, now),
                Err(LeaseCacheError::ResourceNotFound)
            );
            assert_eq!(
                cache.authorize(&prior_session, now),
                Err(LeaseCacheError::SessionMismatch {
                    current: 6,
                    provided: 5,
                })
            );
        }
        assert_eq!(cache.active_lease(id("joint"), 24), Some(active));
        assert_eq!(
            cache.authorize(&command(active, "owner-a", 1, 6, 24), 24),
            Err(LeaseCacheError::Semantic(SemanticError::Expired {
                deadline_ns: 24,
                now_ns: 24,
            }))
        );
        assert_eq!(
            cache.authorize(&command(active, "owner-a", 1, 6, 101), 100),
            Err(LeaseCacheError::Semantic(SemanticError::Expired {
                deadline_ns: 100,
                now_ns: 100,
            }))
        );
    }

    #[test]
    fn session_loss_and_restart_never_restore_stale_authority() {
        let mut cache = LeaseEnforcementCache::<2>::new();
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 7 }, 10)
            .unwrap();
        let first = lease("joint-a", "owner-a", 1, 7, 200);
        let second = lease("joint-b", "owner-b", 2, 7, 200);
        cache.apply(LeaseEvent::Grant(first), 10).unwrap();
        cache.apply(LeaseEvent::Grant(second), 10).unwrap();
        let stale = command(first, "owner-a", 1, 7, 180);
        assert_eq!(
            cache.apply(LeaseEvent::SessionLost { session_epoch: 6 }, 20),
            Err(LeaseCacheError::SessionMismatch {
                current: 7,
                provided: 6
            })
        );
        let lost = cache
            .apply(LeaseEvent::SessionLost { session_epoch: 7 }, 20)
            .unwrap();
        assert_eq!(lost.kind(), LeaseEffectKind::SessionLost);
        assert_eq!(lost.removals().len(), 2);
        assert!(
            lost.removals().iter().all(|item| {
                item.reason() == LeaseRemovalReason::SessionLost && item.was_active()
            })
        );
        assert_eq!(
            cache.authorize(&stale, 21),
            Err(LeaseCacheError::NoCurrentSession)
        );
        assert_eq!(
            cache.apply(LeaseEvent::StartSession { session_epoch: 7 }, 21),
            Err(LeaseCacheError::NonIncreasingSession {
                highest: 7,
                proposed: 7
            })
        );
        cache
            .apply(LeaseEvent::StartSession { session_epoch: 8 }, 21)
            .unwrap();
        assert_eq!(
            cache.authorize(&stale, 22),
            Err(LeaseCacheError::SessionMismatch {
                current: 8,
                provided: 7
            })
        );
        let current = lease("joint-a", "owner-c", 3, 8, 300);
        cache.apply(LeaseEvent::Grant(current), 22).unwrap();
        let replaced = cache
            .apply(LeaseEvent::StartSession { session_epoch: 9 }, 23)
            .unwrap();
        assert_eq!(
            replaced.removals(),
            &[LeaseRemoval {
                lease: current,
                reason: LeaseRemovalReason::SessionReplaced,
                was_active: true
            }]
        );
        assert_eq!(cache.current_session_epoch(), Some(9));
    }

    #[test]
    fn reducer_and_authorization_hot_paths_allocate_nothing() {
        let first = lease("joint", "owner-a", 1, 10, 200);
        let renewed = lease("joint", "owner-a", 1, 10, 250);
        let replacement = lease("joint", "owner-b", 2, 10, 250);
        let replacement_command = command(replacement, "owner-b", 2, 10, 240);
        let mut cache = LeaseEnforcementCache::<2>::new();

        let allocations = measure(|| {
            cache
                .apply(LeaseEvent::StartSession { session_epoch: 10 }, 10)
                .unwrap();
            cache.apply(LeaseEvent::Grant(first), 10).unwrap();
            cache.apply(LeaseEvent::Renew(renewed), 20).unwrap();
            cache
                .apply(
                    LeaseEvent::Preempt {
                        current: LeaseKey::from_lease(renewed),
                        replacement,
                    },
                    30,
                )
                .unwrap();
            cache
                .apply(
                    LeaseEvent::ConfirmSafe(LeaseKey::from_lease(replacement)),
                    31,
                )
                .unwrap();
            assert_eq!(cache.authorize(&replacement_command, 32), Ok(replacement));
            cache
                .apply(LeaseEvent::Release(LeaseKey::from_lease(replacement)), 33)
                .unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
