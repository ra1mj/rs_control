use arrayvec::ArrayVec;
use message_model::{
    ChassisSetpoint, ChassisSetpointIntent, ClockState, CommandEnvelope, CommandIntent,
    DataQuality, Header, Id128, ResourceLease, SemanticError, SourceId,
};
use thiserror::Error;

/// Evidence used to evaluate network age at local reception.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FreshnessEvidence {
    /// No cross-host monotonic relationship; freshness starts at receipt.
    ReceptionOnly,
    /// A clock service translated source age into the local monotonic domain.
    TranslatedAge {
        /// Age at local reception.
        age_ns: u64,
        /// Estimated translation uncertainty.
        uncertainty_ns: u64,
    },
}

/// Route-owned freshness policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FreshnessPolicy {
    /// Accept reception-relative freshness, without claiming end-to-end age.
    ReceptionBounded,
    /// Require a synchronized source and bounded translation uncertainty.
    SynchronizedAgeRequired {
        /// Maximum accepted clock/translation uncertainty.
        max_uncertainty_ns: u64,
    },
}

/// Complete validation policy for one ingress route.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct IngressPolicy {
    /// Maximum route age independent of sender validity.
    pub max_age_ns: u64,
    /// Freshness evidence requirement.
    pub freshness: FreshnessPolicy,
    /// Whether explicitly degraded data remains eligible.
    pub allow_degraded: bool,
}

impl IngressPolicy {
    /// Validates the nonzero route age.
    pub fn validate(self) -> Result<Self, IngressError> {
        if self.max_age_ns == 0 {
            return Err(IngressError::ZeroMaximumAge);
        }
        Ok(self)
    }
}

/// Locally timestamped, validated network sample.
#[derive(Clone, Debug, PartialEq)]
pub struct ReceivedSample<T> {
    /// Decoded semantic value.
    pub value: T,
    /// Local monotonic reception time.
    pub received_monotonic_ns: u64,
    /// Exclusive local monotonic expiry.
    pub expires_monotonic_ns: u64,
    /// Freshness evidence retained for diagnostics.
    pub evidence: FreshnessEvidence,
}

impl<T> ReceivedSample<T> {
    /// Returns whether the cached sample remains usable at local time `now`.
    #[must_use]
    pub const fn usable_at(&self, now_monotonic_ns: u64) -> bool {
        now_monotonic_ns < self.expires_monotonic_ns
    }
}

/// Saturating ingress rejection/acceptance observations.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct IngressCounters {
    /// Accepted samples.
    pub accepted: u64,
    /// Unexpected-source rejections.
    pub unexpected_source: u64,
    /// Unavailable-route rejections.
    pub unavailable: u64,
    /// Unapproved or retired boot rejections.
    pub boot: u64,
    /// Duplicate sequence rejections.
    pub duplicate: u64,
    /// Reordered sequence rejections.
    pub reordered: u64,
    /// Missing sequence values observed across accepted samples.
    pub lost_sequences: u64,
    /// Expired/unsynchronized freshness rejections.
    pub freshness: u64,
    /// Disallowed quality rejections.
    pub quality: u64,
    /// Authority/lease rejections.
    pub authority: u64,
}

fn increment(value: &mut u64) {
    *value = value.saturating_add(1);
}

/// Single-source boot/sequence/freshness guard with bounded restart history.
#[derive(Clone, Debug)]
pub struct SourceGuard<const RETIRED_BOOTS: usize> {
    expected_source: SourceId,
    current_boot: Option<Id128>,
    retired_boots: ArrayVec<Id128, RETIRED_BOOTS>,
    last_sequence: Option<u64>,
    available: bool,
    counters: IngressCounters,
}

impl<const RETIRED_BOOTS: usize> SourceGuard<RETIRED_BOOTS> {
    /// Creates an unavailable route that requires explicit boot authorization.
    #[must_use]
    pub const fn new(expected_source: SourceId) -> Self {
        Self {
            expected_source,
            current_boot: None,
            retired_boots: ArrayVec::new_const(),
            last_sequence: None,
            available: false,
            counters: IngressCounters {
                accepted: 0,
                unexpected_source: 0,
                unavailable: 0,
                boot: 0,
                duplicate: 0,
                reordered: 0,
                lost_sequences: 0,
                freshness: 0,
                quality: 0,
                authority: 0,
            },
        }
    }

    /// Authorizes a boot learned through the presence/handshake control plane.
    pub fn authorize_boot(&mut self, boot_id: Id128) -> Result<(), IngressError> {
        if self.retired_boots.contains(&boot_id) {
            return Err(IngressError::RetiredBoot);
        }
        if self.current_boot == Some(boot_id) {
            self.available = true;
            return Ok(());
        }
        if let Some(current) = self.current_boot {
            self.retired_boots
                .try_push(current)
                .map_err(|_| IngressError::RestartHistoryExhausted)?;
        }
        self.current_boot = Some(boot_id);
        self.last_sequence = None;
        self.available = true;
        Ok(())
    }

    /// Marks the route unavailable while retaining anti-replay state.
    pub const fn mark_disconnected(&mut self) {
        self.available = false;
    }

    /// Returns current availability.
    #[must_use]
    pub const fn is_available(&self) -> bool {
        self.available
    }

    /// Returns a counter snapshot.
    #[must_use]
    pub const fn counters(&self) -> IngressCounters {
        self.counters
    }

    /// Validates source/boot/sequence/quality/freshness and wraps a decoded value.
    pub fn accept<T>(
        &mut self,
        header: Header,
        value: T,
        received_monotonic_ns: u64,
        policy: IngressPolicy,
        evidence: FreshnessEvidence,
    ) -> Result<ReceivedSample<T>, IngressError> {
        let policy = match policy.validate() {
            Ok(policy) => policy,
            Err(error) => {
                self.record(&error);
                return Err(error);
            }
        };
        let result = self.accept_inner(header, value, received_monotonic_ns, policy, evidence);
        if let Err(error) = &result {
            self.record(error);
        }
        result
    }

    fn accept_inner<T>(
        &mut self,
        header: Header,
        value: T,
        received_monotonic_ns: u64,
        policy: IngressPolicy,
        evidence: FreshnessEvidence,
    ) -> Result<ReceivedSample<T>, IngressError> {
        if header.source_id() != self.expected_source {
            return Err(IngressError::UnexpectedSource);
        }
        if !self.available {
            return Err(IngressError::Unavailable);
        }
        let Some(current_boot) = self.current_boot else {
            return Err(IngressError::UnapprovedBoot);
        };
        if header.boot_id() != current_boot {
            return Err(if self.retired_boots.contains(&header.boot_id()) {
                IngressError::RetiredBoot
            } else {
                IngressError::UnapprovedBoot
            });
        }
        if let Some(last) = self.last_sequence {
            if header.sequence() == last {
                return Err(IngressError::DuplicateSequence);
            }
            if header.sequence() < last {
                return Err(IngressError::ReorderedSequence);
            }
        }
        match header.quality() {
            DataQuality::Valid => {}
            DataQuality::Degraded if policy.allow_degraded => {}
            quality => return Err(IngressError::DisallowedQuality(quality)),
        }

        let maximum_lifetime = header.valid_for_ns().min(policy.max_age_ns);
        let remaining = match (policy.freshness, evidence) {
            (FreshnessPolicy::ReceptionBounded, FreshnessEvidence::ReceptionOnly) => {
                maximum_lifetime
            }
            (
                FreshnessPolicy::SynchronizedAgeRequired { max_uncertainty_ns },
                FreshnessEvidence::TranslatedAge {
                    age_ns,
                    uncertainty_ns,
                },
            ) if header.clock_status().state == ClockState::Synchronized
                && uncertainty_ns <= max_uncertainty_ns
                && header.clock_status().uncertainty_ns <= max_uncertainty_ns =>
            {
                maximum_lifetime
                    .checked_sub(age_ns)
                    .filter(|remaining| *remaining > 0)
                    .ok_or(IngressError::Expired)?
            }
            (FreshnessPolicy::ReceptionBounded, _) => {
                return Err(IngressError::UnexpectedFreshnessEvidence);
            }
            (FreshnessPolicy::SynchronizedAgeRequired { .. }, _) => {
                return Err(IngressError::UntrustedClock);
            }
        };
        let expires_monotonic_ns = received_monotonic_ns
            .checked_add(remaining)
            .ok_or(IngressError::TimeOverflow)?;
        if let Some(last) = self.last_sequence {
            self.counters.lost_sequences = self
                .counters
                .lost_sequences
                .saturating_add(header.sequence().saturating_sub(last).saturating_sub(1));
        }
        self.last_sequence = Some(header.sequence());
        increment(&mut self.counters.accepted);
        Ok(ReceivedSample {
            value,
            received_monotonic_ns,
            expires_monotonic_ns,
            evidence,
        })
    }

    fn record(&mut self, error: &IngressError) {
        match error {
            IngressError::UnexpectedSource => increment(&mut self.counters.unexpected_source),
            IngressError::Unavailable => increment(&mut self.counters.unavailable),
            IngressError::UnapprovedBoot
            | IngressError::RetiredBoot
            | IngressError::RestartHistoryExhausted => increment(&mut self.counters.boot),
            IngressError::DuplicateSequence => increment(&mut self.counters.duplicate),
            IngressError::ReorderedSequence => increment(&mut self.counters.reordered),
            IngressError::ZeroMaximumAge
            | IngressError::Expired
            | IngressError::UntrustedClock
            | IngressError::UnexpectedFreshnessEvidence
            | IngressError::TimeOverflow => increment(&mut self.counters.freshness),
            IngressError::DisallowedQuality(_) => increment(&mut self.counters.quality),
            IngressError::Authority(_) => increment(&mut self.counters.authority),
        }
    }
}

/// Materializes a validated command intent against local authority.
pub fn materialize_command(
    intent: CommandIntent,
    active_lease: ResourceLease,
    received_monotonic_ns: u64,
    maximum_ttl_ns: u64,
) -> Result<CommandEnvelope, IngressError> {
    intent
        .materialize(active_lease, received_monotonic_ns, maximum_ttl_ns)
        .map_err(IngressError::Authority)
}

/// Materializes a validated chassis intent against local authority.
pub fn materialize_chassis_setpoint(
    intent: ChassisSetpointIntent,
    active_lease: ResourceLease,
    received_monotonic_ns: u64,
    maximum_ttl_ns: u64,
) -> Result<ChassisSetpoint, IngressError> {
    intent
        .materialize(active_lease, received_monotonic_ns, maximum_ttl_ns)
        .map_err(IngressError::Authority)
}

/// Typed network-ingress rejection.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum IngressError {
    /// Route age must be explicitly nonzero.
    #[error("ingress maximum age must be nonzero")]
    ZeroMaximumAge,
    /// Payload source did not own the route.
    #[error("network payload source does not own this route")]
    UnexpectedSource,
    /// Route was disconnected or not yet announced.
    #[error("network route is unavailable")]
    Unavailable,
    /// Payload boot was not authorized by the control plane.
    #[error("publisher boot has not been authorized")]
    UnapprovedBoot,
    /// A delayed payload used a retired boot identity.
    #[error("publisher boot has already been retired")]
    RetiredBoot,
    /// Bounded anti-replay history was exhausted.
    #[error("publisher restart history is exhausted")]
    RestartHistoryExhausted,
    /// Sequence exactly repeated the last accepted value.
    #[error("duplicate publisher sequence")]
    DuplicateSequence,
    /// Sequence moved backwards within one boot.
    #[error("reordered publisher sequence")]
    ReorderedSequence,
    /// Payload validity was exhausted before receipt.
    #[error("network payload is expired")]
    Expired,
    /// Required clock synchronization evidence was missing or uncertain.
    #[error("network payload clock evidence is not trusted")]
    UntrustedClock,
    /// Evidence kind did not match the route policy.
    #[error("freshness evidence is incompatible with route policy")]
    UnexpectedFreshnessEvidence,
    /// Local expiry arithmetic overflowed.
    #[error("network sample expiry overflowed local monotonic time")]
    TimeOverflow,
    /// Semantic quality was not eligible for the route.
    #[error("network payload quality {0:?} is not allowed")]
    DisallowedQuality(DataQuality),
    /// Command assertion did not match local authority or TTL policy.
    #[error("network command authority rejected: {0}")]
    Authority(#[source] SemanticError),
}

#[cfg(test)]
mod tests {
    use message_model::{
        AsciiId, ClockStatus, CommandIntentFields, HeaderFields, OwnerId, ResourceId,
        ResourceLeaseFields,
    };

    use super::*;

    fn header(source: &str, boot: u8, sequence: u64, quality: DataQuality) -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse(source).unwrap(),
            boot_id: Id128::new([boot; 16]).unwrap(),
            sequence,
            source_monotonic_ns: 1_000,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 100,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Synchronized,
                uncertainty_ns: 5,
            },
            quality,
        })
        .unwrap()
    }

    fn reception_policy() -> IngressPolicy {
        IngressPolicy {
            max_age_ns: 80,
            freshness: FreshnessPolicy::ReceptionBounded,
            allow_degraded: false,
        }
    }

    #[test]
    fn guard_rejects_duplicate_reorder_and_unapproved_boot() {
        let mut guard = SourceGuard::<2>::new(AsciiId::parse("imu-driver").unwrap());
        guard.authorize_boot(Id128::new([1; 16]).unwrap()).unwrap();
        let accepted = guard
            .accept(
                header("imu-driver", 1, 2, DataQuality::Valid),
                42_u64,
                10_000,
                reception_policy(),
                FreshnessEvidence::ReceptionOnly,
            )
            .unwrap();
        assert_eq!(accepted.expires_monotonic_ns, 10_080);
        assert_eq!(
            guard.accept(
                header("imu-driver", 1, 2, DataQuality::Valid),
                43,
                10_001,
                reception_policy(),
                FreshnessEvidence::ReceptionOnly,
            ),
            Err(IngressError::DuplicateSequence)
        );
        assert_eq!(
            guard.accept(
                header("imu-driver", 1, 1, DataQuality::Valid),
                43,
                10_001,
                reception_policy(),
                FreshnessEvidence::ReceptionOnly,
            ),
            Err(IngressError::ReorderedSequence)
        );
        assert_eq!(
            guard.accept(
                header("imu-driver", 2, 3, DataQuality::Valid),
                43,
                10_001,
                reception_policy(),
                FreshnessEvidence::ReceptionOnly,
            ),
            Err(IngressError::UnapprovedBoot)
        );
    }

    #[test]
    fn authorized_restart_rejects_prior_boot_and_disconnect_is_fail_closed() {
        let mut guard = SourceGuard::<2>::new(AsciiId::parse("imu-driver").unwrap());
        guard.authorize_boot(Id128::new([1; 16]).unwrap()).unwrap();
        guard.authorize_boot(Id128::new([2; 16]).unwrap()).unwrap();
        assert_eq!(
            guard.authorize_boot(Id128::new([1; 16]).unwrap()),
            Err(IngressError::RetiredBoot)
        );
        guard.mark_disconnected();
        assert_eq!(
            guard.accept(
                header("imu-driver", 2, 1, DataQuality::Valid),
                1_u8,
                100,
                reception_policy(),
                FreshnessEvidence::ReceptionOnly,
            ),
            Err(IngressError::Unavailable)
        );
    }

    #[test]
    fn synchronized_age_and_quality_are_enforced() {
        let mut guard = SourceGuard::<2>::new(AsciiId::parse("imu-driver").unwrap());
        guard.authorize_boot(Id128::new([1; 16]).unwrap()).unwrap();
        let policy = IngressPolicy {
            max_age_ns: 100,
            freshness: FreshnessPolicy::SynchronizedAgeRequired {
                max_uncertainty_ns: 10,
            },
            allow_degraded: false,
        };
        assert_eq!(
            guard.accept(
                header("imu-driver", 1, 1, DataQuality::Valid),
                1_u8,
                1_000,
                policy,
                FreshnessEvidence::TranslatedAge {
                    age_ns: 100,
                    uncertainty_ns: 5,
                },
            ),
            Err(IngressError::Expired)
        );
        assert!(matches!(
            guard.accept(
                header("imu-driver", 1, 1, DataQuality::Faulted),
                1_u8,
                1_000,
                policy,
                FreshnessEvidence::TranslatedAge {
                    age_ns: 10,
                    uncertainty_ns: 5,
                },
            ),
            Err(IngressError::DisallowedQuality(DataQuality::Faulted))
        ));
    }

    fn lease() -> ResourceLease {
        ResourceLease::new(ResourceLeaseFields {
            resource_id: ResourceId::parse("chassis").unwrap(),
            owner_id: OwnerId::parse("remote-planner").unwrap(),
            lease_id: Id128::new([7; 16]).unwrap(),
            session_epoch: 4,
            expires_monotonic_ns: 1_150,
        })
        .unwrap()
    }

    #[test]
    fn command_materialization_uses_local_lease_and_reception_time() {
        let lease = lease();
        let intent = CommandIntent::new(CommandIntentFields {
            header: header("remote-planner", 1, 1, DataQuality::Valid),
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id: Id128::new([8; 16]).unwrap(),
            requested_ttl_ns: 100,
        })
        .unwrap();
        let envelope = materialize_command(intent, lease, 1_100, 100).unwrap();
        assert_eq!(envelope.deadline_monotonic_ns(), 1_150);

        let bad = CommandIntent::new(CommandIntentFields {
            owner_id: OwnerId::parse("other").unwrap(),
            ..CommandIntentFields {
                header: intent.header(),
                resource_id: intent.resource_id(),
                owner_id: intent.owner_id(),
                lease_id: intent.lease_id(),
                session_epoch: intent.session_epoch(),
                command_id: intent.command_id(),
                requested_ttl_ns: intent.requested_ttl_ns(),
            }
        })
        .unwrap();
        assert!(matches!(
            materialize_command(bad, lease, 1_100, 100),
            Err(IngressError::Authority(SemanticError::AuthorityMismatch {
                field: "owner_id"
            }))
        ));
    }
}
