//! Semantic messages exposed by the `protocol_module`.
//!
//! These types are independent of Protobuf-generated DTOs and avoid heap
//! allocation for stable identifiers used by local realtime channels.

mod action;
mod messages;
mod numeric;

pub use action::*;
pub use messages::*;
pub use numeric::*;

use std::{fmt, str};
use thiserror::Error;

/// Maximum length used by stable component, resource and frame identifiers.
pub const MAX_ID_LEN: usize = 64;

/// Fixed-capacity portable ASCII identifier.
#[derive(Clone, Copy, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct AsciiId<const N: usize> {
    bytes: [u8; N],
    len: u8,
}

impl<const N: usize> AsciiId<N> {
    /// Parses an identifier without allocating.
    pub fn parse(value: &str) -> Result<Self, IdError> {
        if value.is_empty() {
            return Err(IdError::Empty);
        }
        if value.len() > N || value.len() > usize::from(u8::MAX) {
            return Err(IdError::TooLong {
                actual: value.len(),
                maximum: N.min(usize::from(u8::MAX)),
            });
        }

        for (index, byte) in value.bytes().enumerate() {
            let valid = if index == 0 {
                byte.is_ascii_alphanumeric()
            } else {
                byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.')
            };
            if !valid {
                return Err(IdError::InvalidByte { index, byte });
            }
        }

        let mut bytes = [0; N];
        bytes[..value.len()].copy_from_slice(value.as_bytes());
        Ok(Self {
            bytes,
            len: value.len() as u8,
        })
    }

    /// Returns the identifier as UTF-8.
    #[must_use]
    pub fn as_str(&self) -> &str {
        // Construction accepts only ASCII and fields are private.
        str::from_utf8(&self.bytes[..usize::from(self.len)])
            .expect("AsciiId construction guarantees ASCII")
    }
}

impl<const N: usize> fmt::Debug for AsciiId<N> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_tuple("AsciiId")
            .field(&self.as_str())
            .finish()
    }
}

impl<const N: usize> fmt::Display for AsciiId<N> {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

impl<const N: usize> TryFrom<&str> for AsciiId<N> {
    type Error = IdError;

    fn try_from(value: &str) -> Result<Self, Self::Error> {
        Self::parse(value)
    }
}

/// Identifier validation failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum IdError {
    /// Empty identifiers are ambiguous and forbidden.
    #[error("identifier cannot be empty")]
    Empty,
    /// Identifier exceeded its fixed capacity.
    #[error("identifier length {actual} exceeds maximum {maximum}")]
    TooLong {
        /// Observed byte length.
        actual: usize,
        /// Supported byte length.
        maximum: usize,
    },
    /// Identifier contained a byte outside the portable routing alphabet.
    #[error("identifier byte 0x{byte:02x} at index {index} is not allowed")]
    InvalidByte {
        /// Byte offset.
        index: usize,
        /// Rejected byte.
        byte: u8,
    },
}

/// Non-zero 128-bit process/session/lease/command identity.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Id128([u8; 16]);

impl Id128 {
    /// Constructs an identity from exactly 16 non-zero bytes.
    pub fn new(bytes: [u8; 16]) -> Result<Self, Id128Error> {
        if bytes == [0; 16] {
            return Err(Id128Error::AllZero);
        }
        Ok(Self(bytes))
    }

    /// Parses an identity from wire bytes.
    pub fn from_slice(bytes: &[u8]) -> Result<Self, Id128Error> {
        let bytes: [u8; 16] = bytes.try_into().map_err(|_| Id128Error::InvalidLength {
            actual: bytes.len(),
        })?;
        Self::new(bytes)
    }

    /// Returns the canonical 16-byte representation.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 16] {
        &self.0
    }
}

/// Invalid fixed-width identity.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum Id128Error {
    /// Wire representation was not exactly 16 bytes.
    #[error("identity must contain exactly 16 bytes, got {actual}")]
    InvalidLength {
        /// Observed length.
        actual: usize,
    },
    /// All-zero identity is reserved as an unspecified sentinel.
    #[error("identity cannot be all zero")]
    AllZero,
}

/// Stable source identifier.
pub type SourceId = AsciiId<MAX_ID_LEN>;
/// Stable coordinate-frame identifier.
pub type FrameId = AsciiId<MAX_ID_LEN>;
/// Stable controlled-resource identifier.
pub type ResourceId = AsciiId<MAX_ID_LEN>;
/// Stable command-owner identifier.
pub type OwnerId = AsciiId<MAX_ID_LEN>;

/// Relationship between source and consumer clock domains.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum ClockState {
    /// Source and consumer use the same local monotonic clock.
    Local,
    /// A measured synchronization relationship is available.
    Synchronized,
    /// No trustworthy synchronization relationship is available.
    Unsynchronized,
}

/// Clock state plus its estimated uncertainty.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClockStatus {
    /// Clock relationship.
    pub state: ClockState,
    /// Estimated relationship uncertainty in nanoseconds.
    pub uncertainty_ns: u64,
}

/// Explicit clock domain for a source acquisition timestamp.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum TimestampDomain {
    /// Unix/system time.
    Unix,
    /// ROS time, which may pause or jump.
    Ros,
}

/// Source acquisition time, distinct from monotonic freshness and publication time.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AcquisitionTimestamp {
    /// Time in the selected domain, in nanoseconds.
    pub timestamp_ns: u64,
    /// Explicit clock domain.
    pub domain: TimestampDomain,
    /// Estimated timestamp uncertainty in nanoseconds.
    pub uncertainty_ns: u64,
}

/// Semantic quality exposed through protocol-module contracts.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum DataQuality {
    /// Data is valid for its documented use.
    Valid,
    /// Data may be used only by an explicit degraded policy.
    Degraded,
    /// Data has exceeded its freshness limit.
    Stale,
    /// Source is calibrating.
    Calibrating,
    /// Source has reported or inferred a fault.
    Faulted,
}

/// Semantic metadata shared by externally visible samples and commands.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Header {
    source_id: SourceId,
    boot_id: Id128,
    sequence: u64,
    source_monotonic_ns: u64,
    publish_unix_ns: Option<u64>,
    acquisition_timestamp: Option<AcquisitionTimestamp>,
    valid_for_ns: u64,
    frame_id: Option<FrameId>,
    clock_status: ClockStatus,
    quality: DataQuality,
}

/// Fields supplied to the validating [`Header`] constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HeaderFields {
    /// Stable publisher identity.
    pub source_id: SourceId,
    /// Identity regenerated on every publisher restart.
    pub boot_id: Id128,
    /// Monotonic sequence within one boot identity.
    pub sequence: u64,
    /// Source timestamp in its monotonic clock domain.
    pub source_monotonic_ns: u64,
    /// Optional Unix timestamp for correlation only.
    pub publish_unix_ns: Option<u64>,
    /// Optional source acquisition timestamp for external observation stamps.
    pub acquisition_timestamp: Option<AcquisitionTimestamp>,
    /// Maximum validity duration in nanoseconds.
    pub valid_for_ns: u64,
    /// Optional coordinate frame.
    pub frame_id: Option<FrameId>,
    /// Source/consumer clock relationship.
    pub clock_status: ClockStatus,
    /// Semantic sample quality.
    pub quality: DataQuality,
}

impl Header {
    /// Constructs a header after validating all cross-field invariants.
    pub fn new(fields: HeaderFields) -> Result<Self, SemanticError> {
        if fields.valid_for_ns == 0 {
            return Err(SemanticError::ZeroValidity);
        }
        if fields.clock_status.state == ClockState::Local && fields.clock_status.uncertainty_ns != 0
        {
            return Err(SemanticError::LocalClockUncertainty {
                uncertainty_ns: fields.clock_status.uncertainty_ns,
            });
        }
        Ok(Self {
            source_id: fields.source_id,
            boot_id: fields.boot_id,
            sequence: fields.sequence,
            source_monotonic_ns: fields.source_monotonic_ns,
            publish_unix_ns: fields.publish_unix_ns,
            acquisition_timestamp: fields.acquisition_timestamp,
            valid_for_ns: fields.valid_for_ns,
            frame_id: fields.frame_id,
            clock_status: fields.clock_status,
            quality: fields.quality,
        })
    }

    /// Returns the stable publisher identity.
    #[must_use]
    pub const fn source_id(self) -> SourceId {
        self.source_id
    }

    /// Returns the publisher boot identity.
    #[must_use]
    pub const fn boot_id(self) -> Id128 {
        self.boot_id
    }

    /// Returns the sequence within the boot identity.
    #[must_use]
    pub const fn sequence(self) -> u64 {
        self.sequence
    }

    /// Returns the timestamp in the source monotonic clock domain.
    #[must_use]
    pub const fn source_monotonic_ns(self) -> u64 {
        self.source_monotonic_ns
    }

    /// Returns the optional Unix correlation timestamp.
    #[must_use]
    pub const fn publish_unix_ns(self) -> Option<u64> {
        self.publish_unix_ns
    }

    /// Returns source acquisition time for an external observation stamp.
    #[must_use]
    pub const fn acquisition_timestamp(self) -> Option<AcquisitionTimestamp> {
        self.acquisition_timestamp
    }

    /// Returns the maximum validity duration.
    #[must_use]
    pub const fn valid_for_ns(self) -> u64 {
        self.valid_for_ns
    }

    /// Returns the optional coordinate frame.
    #[must_use]
    pub const fn frame_id(self) -> Option<FrameId> {
        self.frame_id
    }

    /// Returns the source/consumer clock relationship.
    #[must_use]
    pub const fn clock_status(self) -> ClockStatus {
        self.clock_status
    }

    /// Returns the semantic quality.
    #[must_use]
    pub const fn quality(self) -> DataQuality {
        self.quality
    }

    /// Returns the absolute source-clock expiration time, or `None` on overflow.
    #[must_use]
    pub const fn source_expiration_ns(&self) -> Option<u64> {
        self.source_monotonic_ns.checked_add(self.valid_for_ns)
    }
}

/// Exclusive control authority cached by the motion runtime.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResourceLease {
    resource_id: ResourceId,
    owner_id: OwnerId,
    lease_id: Id128,
    session_epoch: u64,
    expires_monotonic_ns: u64,
}

/// Fields supplied to the validating [`ResourceLease`] constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ResourceLeaseFields {
    /// Controlled resource.
    pub resource_id: ResourceId,
    /// Authorized owner.
    pub owner_id: OwnerId,
    /// Unique lease grant identity.
    pub lease_id: Id128,
    /// Authority service session.
    pub session_epoch: u64,
    /// Expiration in the local enforcement monotonic clock domain.
    pub expires_monotonic_ns: u64,
}

impl ResourceLease {
    /// Constructs a structurally valid lease.
    pub fn new(fields: ResourceLeaseFields) -> Result<Self, SemanticError> {
        if fields.session_epoch == 0 {
            return Err(SemanticError::ZeroSessionEpoch);
        }
        if fields.expires_monotonic_ns == 0 {
            return Err(SemanticError::ZeroDeadline {
                field: "expires_monotonic_ns",
            });
        }
        Ok(Self {
            resource_id: fields.resource_id,
            owner_id: fields.owner_id,
            lease_id: fields.lease_id,
            session_epoch: fields.session_epoch,
            expires_monotonic_ns: fields.expires_monotonic_ns,
        })
    }

    /// Returns the controlled resource.
    #[must_use]
    pub const fn resource_id(self) -> ResourceId {
        self.resource_id
    }

    /// Returns the authorized owner.
    #[must_use]
    pub const fn owner_id(self) -> OwnerId {
        self.owner_id
    }

    /// Returns the lease grant identity.
    #[must_use]
    pub const fn lease_id(self) -> Id128 {
        self.lease_id
    }

    /// Returns the authority session epoch.
    #[must_use]
    pub const fn session_epoch(self) -> u64 {
        self.session_epoch
    }

    /// Returns the local enforcement expiration time.
    #[must_use]
    pub const fn expires_monotonic_ns(self) -> u64 {
        self.expires_monotonic_ns
    }

    /// Validates static and time-relative lease invariants.
    pub fn validate_at(&self, now_monotonic_ns: u64) -> Result<(), SemanticError> {
        if self.session_epoch == 0 {
            return Err(SemanticError::ZeroSessionEpoch);
        }
        if self.expires_monotonic_ns <= now_monotonic_ns {
            return Err(SemanticError::Expired {
                deadline_ns: self.expires_monotonic_ns,
                now_ns: now_monotonic_ns,
            });
        }
        Ok(())
    }
}

/// Authority and correlation metadata for one motion command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandEnvelope {
    header: Header,
    resource_id: ResourceId,
    owner_id: OwnerId,
    lease_id: Id128,
    session_epoch: u64,
    command_id: Id128,
    deadline_monotonic_ns: u64,
}

/// Fields supplied to the validating [`CommandEnvelope`] constructor.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CommandEnvelopeFields {
    /// Common message metadata.
    pub header: Header,
    /// Controlled resource.
    pub resource_id: ResourceId,
    /// Command owner.
    pub owner_id: OwnerId,
    /// Lease grant identity.
    pub lease_id: Id128,
    /// Authority service session.
    pub session_epoch: u64,
    /// Unique command identity.
    pub command_id: Id128,
    /// Absolute local monotonic deadline.
    pub deadline_monotonic_ns: u64,
}

impl CommandEnvelope {
    /// Constructs a structurally valid command envelope.
    pub fn new(fields: CommandEnvelopeFields) -> Result<Self, SemanticError> {
        if fields.session_epoch == 0 {
            return Err(SemanticError::ZeroSessionEpoch);
        }
        if fields.deadline_monotonic_ns == 0 {
            return Err(SemanticError::ZeroDeadline {
                field: "deadline_monotonic_ns",
            });
        }
        Ok(Self {
            header: fields.header,
            resource_id: fields.resource_id,
            owner_id: fields.owner_id,
            lease_id: fields.lease_id,
            session_epoch: fields.session_epoch,
            command_id: fields.command_id,
            deadline_monotonic_ns: fields.deadline_monotonic_ns,
        })
    }

    /// Constructs an ingress command from an existing local lease and bounded TTL.
    ///
    /// The resulting deadline is never later than the lease expiration. No
    /// external timestamp is used for local enforcement.
    pub fn from_bridge_ingress(
        header: Header,
        lease: ResourceLease,
        command_id: Id128,
        received_monotonic_ns: u64,
        requested_ttl_ns: u64,
        maximum_ttl_ns: u64,
    ) -> Result<Self, SemanticError> {
        lease.validate_at(received_monotonic_ns)?;
        if requested_ttl_ns == 0 || maximum_ttl_ns == 0 || requested_ttl_ns > maximum_ttl_ns {
            return Err(SemanticError::InvalidDuration {
                field: "command.ttl_ns",
                maximum_ns: maximum_ttl_ns,
            });
        }
        let requested_deadline = received_monotonic_ns.checked_add(requested_ttl_ns).ok_or(
            SemanticError::TimeOverflow {
                field: "command.deadline_monotonic_ns",
            },
        )?;
        Self::new(CommandEnvelopeFields {
            header,
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id,
            deadline_monotonic_ns: requested_deadline.min(lease.expires_monotonic_ns()),
        })
    }

    /// Returns common message metadata.
    #[must_use]
    pub const fn header(self) -> Header {
        self.header
    }

    /// Returns the controlled resource.
    #[must_use]
    pub const fn resource_id(self) -> ResourceId {
        self.resource_id
    }

    /// Returns the command owner.
    #[must_use]
    pub const fn owner_id(self) -> OwnerId {
        self.owner_id
    }

    /// Returns the lease grant identity.
    #[must_use]
    pub const fn lease_id(self) -> Id128 {
        self.lease_id
    }

    /// Returns the authority session epoch.
    #[must_use]
    pub const fn session_epoch(self) -> u64 {
        self.session_epoch
    }

    /// Returns the command identity.
    #[must_use]
    pub const fn command_id(self) -> Id128 {
        self.command_id
    }

    /// Returns the absolute local monotonic deadline.
    #[must_use]
    pub const fn deadline_monotonic_ns(self) -> u64 {
        self.deadline_monotonic_ns
    }

    /// Validates the envelope against the current enforcement time.
    pub fn validate_at(&self, now_monotonic_ns: u64) -> Result<(), SemanticError> {
        if self.deadline_monotonic_ns <= now_monotonic_ns {
            return Err(SemanticError::Expired {
                deadline_ns: self.deadline_monotonic_ns,
                now_ns: now_monotonic_ns,
            });
        }
        Ok(())
    }
}

/// Semantic validation failure at the protocol-module boundary.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum SemanticError {
    /// A zero validity window would make freshness semantics ambiguous.
    #[error("valid_for_ns must be greater than zero")]
    ZeroValidity,
    /// A local clock is exact by definition within the process and uses zero uncertainty.
    #[error("local clock must report zero uncertainty, got {uncertainty_ns} ns")]
    LocalClockUncertainty {
        /// Rejected uncertainty.
        uncertainty_ns: u64,
    },
    /// Session epoch zero is reserved as unspecified.
    #[error("session_epoch must be greater than zero")]
    ZeroSessionEpoch,
    /// Absolute deadline/expiration zero is reserved as unspecified.
    #[error("{field} must be greater than zero")]
    ZeroDeadline {
        /// Rejected field.
        field: &'static str,
    },
    /// Lease or command has already expired.
    #[error("deadline {deadline_ns} ns is not after current time {now_ns} ns")]
    Expired {
        /// Rejected deadline.
        deadline_ns: u64,
        /// Enforcement time.
        now_ns: u64,
    },
    /// A numeric field contained NaN or infinity.
    #[error("field {field} must be finite")]
    NonFinite {
        /// Rejected field path.
        field: &'static str,
    },
    /// Quaternion norm was outside the accepted tolerance.
    #[error("quaternion must be unit length within the configured tolerance")]
    InvalidQuaternion,
    /// Covariance diagonal was negative.
    #[error("covariance diagonal at index {index} must be nonnegative")]
    NegativeCovarianceDiagonal {
        /// Rejected diagonal index.
        index: usize,
    },
    /// Covariance matrix was not symmetric.
    #[error("covariance entries ({row}, {column}) and ({column}, {row}) are not symmetric")]
    AsymmetricCovariance {
        /// Row index.
        row: usize,
        /// Column index.
        column: usize,
    },
    /// A frame-bearing sample omitted its frame.
    #[error("required frame field {field} is missing")]
    MissingFrame {
        /// Rejected field path.
        field: &'static str,
    },
    /// Header frame did not match the explicit parent frame.
    #[error("header frame does not match {field} parent frame")]
    FrameMismatch {
        /// Rejected message class.
        field: &'static str,
    },
    /// Parent and child frame were identical.
    #[error("parent and child frame must differ")]
    SelfFrame,
    /// A required bounded collection was empty.
    #[error("collection {field} cannot be empty")]
    EmptyCollection {
        /// Rejected collection.
        field: &'static str,
    },
    /// A collection contained a duplicate stable identifier.
    #[error("collection {field} contains a duplicate identifier")]
    DuplicateIdentifier {
        /// Rejected collection.
        field: &'static str,
    },
    /// A supplied bound was zero or exceeded the supported maximum.
    #[error("field {field} must be nonzero and no greater than {maximum}")]
    InvalidBound {
        /// Rejected field.
        field: &'static str,
        /// Maximum supported value.
        maximum: usize,
    },
    /// A requested duration was zero or exceeded policy.
    #[error("field {field} must be nonzero and no greater than {maximum_ns} ns")]
    InvalidDuration {
        /// Rejected field.
        field: &'static str,
        /// Maximum accepted duration.
        maximum_ns: u64,
    },
    /// Monotonic time arithmetic overflowed.
    #[error("field {field} overflowed monotonic time")]
    TimeOverflow {
        /// Rejected field.
        field: &'static str,
    },
    /// Static and dynamic transforms appeared in one route batch.
    #[error("transform batch cannot mix static and dynamic authority")]
    MixedTransformAuthority,
    /// A count that must describe an occurrence was zero.
    #[error("field {field} must be greater than zero")]
    ZeroCount {
        /// Rejected field.
        field: &'static str,
    },
    /// Ordered timestamps were reversed or duplicated.
    #[error("field {field} must be strictly time ordered")]
    InvalidTimeOrder {
        /// Rejected field.
        field: &'static str,
    },
    /// Action message used an invalid status for its lifecycle context.
    #[error("action status is invalid for {context}")]
    InvalidActionStatus {
        /// Lifecycle message context.
        context: &'static str,
    },
    /// Action lifecycle event was invalid in the current state.
    #[error("action lifecycle transition is invalid")]
    InvalidActionTransition,
    /// Numeric field was outside [0, 1].
    #[error("field {field} must be in the closed interval [0, 1]")]
    OutsideClosedUnitInterval {
        /// Rejected field.
        field: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use super::{
        AcquisitionTimestamp, AsciiId, ClockState, ClockStatus, CommandEnvelope, DataQuality,
        Header, HeaderFields, Id128, IdError, ResourceLease, ResourceLeaseFields, TimestampDomain,
    };

    fn header_fields() -> HeaderFields {
        HeaderFields {
            source_id: AsciiId::parse("motion-rt").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 7,
            source_monotonic_ns: 1_000,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 10_000,
            frame_id: Some(AsciiId::parse("base_link").unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        }
    }

    #[test]
    fn identifiers_are_fixed_capacity_and_routing_safe() {
        let id = AsciiId::<16>::parse("arm.left-1").unwrap();
        assert_eq!(id.as_str(), "arm.left-1");
        assert_eq!(
            AsciiId::<16>::parse("arm/left"),
            Err(IdError::InvalidByte {
                index: 3,
                byte: b'/'
            })
        );
    }

    #[test]
    fn header_requires_explicit_validity() {
        let mut fields = header_fields();
        fields.valid_for_ns = 0;
        assert!(Header::new(fields).is_err());
    }

    #[test]
    fn source_expiration_detects_overflow() {
        let mut fields = header_fields();
        fields.source_monotonic_ns = u64::MAX;
        let value = Header::new(fields).unwrap();
        assert_eq!(value.source_expiration_ns(), None);
    }

    #[test]
    fn bridge_ingress_requires_current_lease_and_local_bounded_deadline() {
        let lease = ResourceLease::new(ResourceLeaseFields {
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("ros2-adapter").unwrap(),
            lease_id: Id128::new([2; 16]).unwrap(),
            session_epoch: 3,
            expires_monotonic_ns: 1_150,
        })
        .unwrap();
        let mut fields = header_fields();
        fields.acquisition_timestamp = Some(AcquisitionTimestamp {
            timestamp_ns: u64::MAX,
            domain: TimestampDomain::Ros,
            uncertainty_ns: 0,
        });
        let envelope = CommandEnvelope::from_bridge_ingress(
            Header::new(fields).unwrap(),
            lease,
            Id128::new([3; 16]).unwrap(),
            1_000,
            200,
            250,
        )
        .unwrap();
        assert_eq!(envelope.deadline_monotonic_ns(), 1_150);

        assert!(
            CommandEnvelope::from_bridge_ingress(
                Header::new(fields).unwrap(),
                lease,
                Id128::new([3; 16]).unwrap(),
                1_150,
                10,
                250,
            )
            .is_err()
        );
        assert!(
            CommandEnvelope::from_bridge_ingress(
                Header::new(fields).unwrap(),
                lease,
                Id128::new([3; 16]).unwrap(),
                1_000,
                251,
                250,
            )
            .is_err()
        );
    }
}
