//! Sole conversion and validation owner between semantic and Protobuf messages.

mod mapping;
mod navigation;

pub use mapping::*;
pub use navigation::*;

use message_model::{
    AcquisitionTimestamp, AsciiId, ClockState, ClockStatus, CommandEnvelope, CommandEnvelopeFields,
    DataQuality, Header, HeaderFields, Id128, Id128Error, IdError, ResourceLease,
    ResourceLeaseFields, SemanticError, TimestampDomain,
};
use prost::Message;
use proto_generated::robot::v1 as wire;
use thiserror::Error;

/// Encodes a validated semantic header as Protobuf bytes.
pub fn encode_header(header: &Header) -> Result<Vec<u8>, CodecError> {
    Ok(header_to_wire(header).encode_to_vec())
}

/// Decodes Protobuf bytes and validates the resulting semantic header.
pub fn decode_header(bytes: &[u8]) -> Result<Header, CodecError> {
    let value = wire::Header::decode(bytes)?;
    header_from_wire(value)
}

/// Converts a semantic header into its wire DTO.
#[must_use]
pub fn header_to_wire(header: &Header) -> wire::Header {
    wire::Header {
        source_id: header.source_id().as_str().to_owned(),
        boot_id: header.boot_id().as_bytes().to_vec(),
        sequence: header.sequence(),
        source_monotonic_ns: header.source_monotonic_ns(),
        publish_unix_ns: header.publish_unix_ns(),
        valid_for_ns: header.valid_for_ns(),
        frame_id: header
            .frame_id()
            .map_or_else(String::new, |value| value.as_str().to_owned()),
        clock_status: Some(wire::ClockStatus {
            state: clock_state_to_wire(header.clock_status().state) as i32,
            uncertainty_ns: header.clock_status().uncertainty_ns,
        }),
        quality: quality_to_wire(header.quality()) as i32,
        acquisition_timestamp: header.acquisition_timestamp().map(|timestamp| {
            wire::AcquisitionTimestamp {
                timestamp_ns: timestamp.timestamp_ns,
                domain: timestamp_domain_to_wire(timestamp.domain) as i32,
                uncertainty_ns: timestamp.uncertainty_ns,
            }
        }),
    }
}

/// Converts and validates a wire DTO into the semantic contract.
pub fn header_from_wire(value: wire::Header) -> Result<Header, CodecError> {
    let clock = value
        .clock_status
        .ok_or(CodecError::MissingField("clock_status"))?;
    Header::new(HeaderFields {
        source_id: AsciiId::parse(&value.source_id)?,
        boot_id: Id128::from_slice(&value.boot_id)?,
        sequence: value.sequence,
        source_monotonic_ns: value.source_monotonic_ns,
        publish_unix_ns: value.publish_unix_ns,
        acquisition_timestamp: value
            .acquisition_timestamp
            .map(|timestamp| -> Result<AcquisitionTimestamp, CodecError> {
                Ok(AcquisitionTimestamp {
                    timestamp_ns: timestamp.timestamp_ns,
                    domain: timestamp_domain_from_wire(timestamp.domain)?,
                    uncertainty_ns: timestamp.uncertainty_ns,
                })
            })
            .transpose()?,
        valid_for_ns: value.valid_for_ns,
        frame_id: if value.frame_id.is_empty() {
            None
        } else {
            Some(AsciiId::parse(&value.frame_id)?)
        },
        clock_status: ClockStatus {
            state: clock_state_from_wire(clock.state)?,
            uncertainty_ns: clock.uncertainty_ns,
        },
        quality: quality_from_wire(value.quality)?,
    })
    .map_err(CodecError::from)
}

/// Encodes a lease after checking that it is still valid at `now_monotonic_ns`.
pub fn encode_resource_lease(
    lease: &ResourceLease,
    now_monotonic_ns: u64,
) -> Result<Vec<u8>, CodecError> {
    lease.validate_at(now_monotonic_ns)?;
    Ok(resource_lease_to_wire(lease).encode_to_vec())
}

/// Decodes and validates a resource lease at the enforcement time.
pub fn decode_resource_lease(
    bytes: &[u8],
    now_monotonic_ns: u64,
) -> Result<ResourceLease, CodecError> {
    resource_lease_from_wire(wire::ResourceLease::decode(bytes)?, now_monotonic_ns)
}

/// Converts a semantic resource lease into its wire DTO.
#[must_use]
pub fn resource_lease_to_wire(lease: &ResourceLease) -> wire::ResourceLease {
    wire::ResourceLease {
        resource_id: lease.resource_id().as_str().to_owned(),
        owner_id: lease.owner_id().as_str().to_owned(),
        lease_id: lease.lease_id().as_bytes().to_vec(),
        session_epoch: lease.session_epoch(),
        expires_monotonic_ns: lease.expires_monotonic_ns(),
    }
}

/// Converts and validates a resource-lease DTO.
pub fn resource_lease_from_wire(
    value: wire::ResourceLease,
    now_monotonic_ns: u64,
) -> Result<ResourceLease, CodecError> {
    let lease = ResourceLease::new(ResourceLeaseFields {
        resource_id: AsciiId::parse(&value.resource_id)?,
        owner_id: AsciiId::parse(&value.owner_id)?,
        lease_id: Id128::from_slice(&value.lease_id)?,
        session_epoch: value.session_epoch,
        expires_monotonic_ns: value.expires_monotonic_ns,
    })?;
    lease.validate_at(now_monotonic_ns)?;
    Ok(lease)
}

/// Encodes a command envelope after checking its local deadline.
pub fn encode_command_envelope(
    envelope: &CommandEnvelope,
    now_monotonic_ns: u64,
) -> Result<Vec<u8>, CodecError> {
    envelope.validate_at(now_monotonic_ns)?;
    Ok(command_envelope_to_wire(envelope).encode_to_vec())
}

/// Decodes and validates a command envelope at the enforcement time.
pub fn decode_command_envelope(
    bytes: &[u8],
    now_monotonic_ns: u64,
) -> Result<CommandEnvelope, CodecError> {
    command_envelope_from_wire(wire::CommandEnvelope::decode(bytes)?, now_monotonic_ns)
}

/// Converts a semantic command envelope into its wire DTO.
#[must_use]
pub fn command_envelope_to_wire(envelope: &CommandEnvelope) -> wire::CommandEnvelope {
    wire::CommandEnvelope {
        header: Some(header_to_wire(&envelope.header())),
        resource_id: envelope.resource_id().as_str().to_owned(),
        owner_id: envelope.owner_id().as_str().to_owned(),
        lease_id: envelope.lease_id().as_bytes().to_vec(),
        session_epoch: envelope.session_epoch(),
        command_id: envelope.command_id().as_bytes().to_vec(),
        deadline_monotonic_ns: envelope.deadline_monotonic_ns(),
    }
}

/// Converts and validates a command-envelope DTO.
pub fn command_envelope_from_wire(
    value: wire::CommandEnvelope,
    now_monotonic_ns: u64,
) -> Result<CommandEnvelope, CodecError> {
    let header = header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?;
    let envelope = CommandEnvelope::new(CommandEnvelopeFields {
        header,
        resource_id: AsciiId::parse(&value.resource_id)?,
        owner_id: AsciiId::parse(&value.owner_id)?,
        lease_id: Id128::from_slice(&value.lease_id)?,
        session_epoch: value.session_epoch,
        command_id: Id128::from_slice(&value.command_id)?,
        deadline_monotonic_ns: value.deadline_monotonic_ns,
    })?;
    envelope.validate_at(now_monotonic_ns)?;
    Ok(envelope)
}

fn clock_state_to_wire(value: ClockState) -> wire::ClockState {
    match value {
        ClockState::Local => wire::ClockState::Local,
        ClockState::Synchronized => wire::ClockState::Synchronized,
        ClockState::Unsynchronized => wire::ClockState::Unsynchronized,
    }
}

fn clock_state_from_wire(value: i32) -> Result<ClockState, CodecError> {
    match wire::ClockState::try_from(value) {
        Ok(wire::ClockState::Local) => Ok(ClockState::Local),
        Ok(wire::ClockState::Synchronized) => Ok(ClockState::Synchronized),
        Ok(wire::ClockState::Unsynchronized) => Ok(ClockState::Unsynchronized),
        Ok(wire::ClockState::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "clock_status.state",
            value,
        }),
    }
}

fn quality_to_wire(value: DataQuality) -> wire::DataQuality {
    match value {
        DataQuality::Valid => wire::DataQuality::Valid,
        DataQuality::Degraded => wire::DataQuality::Degraded,
        DataQuality::Stale => wire::DataQuality::Stale,
        DataQuality::Calibrating => wire::DataQuality::Calibrating,
        DataQuality::Faulted => wire::DataQuality::Faulted,
    }
}

fn quality_from_wire(value: i32) -> Result<DataQuality, CodecError> {
    match wire::DataQuality::try_from(value) {
        Ok(wire::DataQuality::Valid) => Ok(DataQuality::Valid),
        Ok(wire::DataQuality::Degraded) => Ok(DataQuality::Degraded),
        Ok(wire::DataQuality::Stale) => Ok(DataQuality::Stale),
        Ok(wire::DataQuality::Calibrating) => Ok(DataQuality::Calibrating),
        Ok(wire::DataQuality::Faulted) => Ok(DataQuality::Faulted),
        Ok(wire::DataQuality::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "quality",
            value,
        }),
    }
}

fn timestamp_domain_to_wire(value: TimestampDomain) -> wire::TimestampDomain {
    match value {
        TimestampDomain::Unix => wire::TimestampDomain::Unix,
        TimestampDomain::Ros => wire::TimestampDomain::Ros,
    }
}

fn timestamp_domain_from_wire(value: i32) -> Result<TimestampDomain, CodecError> {
    match wire::TimestampDomain::try_from(value) {
        Ok(wire::TimestampDomain::Unix) => Ok(TimestampDomain::Unix),
        Ok(wire::TimestampDomain::Ros) => Ok(TimestampDomain::Ros),
        Ok(wire::TimestampDomain::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "acquisition_timestamp.domain",
            value,
        }),
    }
}

/// Wire decoding or semantic validation failure.
#[derive(Debug, Error)]
pub enum CodecError {
    /// Protobuf payload was malformed.
    #[error("invalid protobuf payload: {0}")]
    Decode(#[from] prost::DecodeError),
    /// Required message field was absent.
    #[error("required field {0} is missing")]
    MissingField(&'static str),
    /// Enum value was unspecified or unknown.
    #[error("field {field} contains unsupported enum value {value}")]
    UnknownEnum {
        /// Field path.
        field: &'static str,
        /// Raw wire value.
        value: i32,
    },
    /// Portable identifier was invalid.
    #[error(transparent)]
    Identifier(#[from] IdError),
    /// Fixed-width identity was invalid.
    #[error(transparent)]
    Identity(#[from] Id128Error),
    /// Semantic invariant was invalid.
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    /// A repeated field had the wrong exact length.
    #[error("field {field} must contain exactly {expected} entries, got {actual}")]
    InvalidLength {
        /// Rejected field path.
        field: &'static str,
        /// Required length.
        expected: usize,
        /// Observed length.
        actual: usize,
    },
    /// A repeated field exceeded its semantic bound.
    #[error("field {field} contains {actual} entries, exceeding maximum {maximum}")]
    TooMany {
        /// Rejected field path.
        field: &'static str,
        /// Maximum accepted length.
        maximum: usize,
        /// Observed length.
        actual: usize,
    },
    /// A scalar or cross-field wire value was invalid.
    #[error("field {field} is invalid: {reason}")]
    InvalidField {
        /// Rejected field path.
        field: &'static str,
        /// Stable diagnostic reason.
        reason: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use super::{
        decode_command_envelope, decode_header, decode_resource_lease, encode_command_envelope,
        encode_header, encode_resource_lease, header_from_wire, header_to_wire,
    };
    use message_model::{
        AsciiId, ClockState, ClockStatus, CommandEnvelope, CommandEnvelopeFields, DataQuality,
        Header, HeaderFields, Id128, ResourceLease, ResourceLeaseFields,
    };
    use prost::Message;
    use proto_generated::robot::v1 as wire;

    fn sample() -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("motion-rt").unwrap(),
            boot_id: Id128::new([7; 16]).unwrap(),
            sequence: 42,
            source_monotonic_ns: 1_000_000,
            publish_unix_ns: Some(2_000_000),
            acquisition_timestamp: Some(message_model::AcquisitionTimestamp {
                timestamp_ns: 1_500_000,
                domain: message_model::TimestampDomain::Unix,
                uncertainty_ns: 100,
            }),
            valid_for_ns: 5_000_000,
            frame_id: Some(AsciiId::parse("base_link").unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    fn sample_lease() -> ResourceLease {
        ResourceLease::new(ResourceLeaseFields {
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("robot-fsm").unwrap(),
            lease_id: Id128::new([8; 16]).unwrap(),
            session_epoch: 3,
            expires_monotonic_ns: 2_000_000,
        })
        .unwrap()
    }

    fn sample_command() -> CommandEnvelope {
        CommandEnvelope::new(CommandEnvelopeFields {
            header: sample(),
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("robot-fsm").unwrap(),
            lease_id: Id128::new([8; 16]).unwrap(),
            session_epoch: 3,
            command_id: Id128::new([9; 16]).unwrap(),
            deadline_monotonic_ns: 1_500_000,
        })
        .unwrap()
    }

    #[test]
    fn protobuf_round_trip_preserves_header_semantics() {
        let value = sample();
        assert_eq!(
            decode_header(&encode_header(&value).unwrap()).unwrap(),
            value
        );
        assert_eq!(header_from_wire(header_to_wire(&value)).unwrap(), value);
    }

    #[test]
    fn protobuf_round_trip_preserves_lease_and_command_authority() {
        let lease = sample_lease();
        let encoded = encode_resource_lease(&lease, 1_000_000).unwrap();
        assert_eq!(decode_resource_lease(&encoded, 1_000_000).unwrap(), lease);

        let command = sample_command();
        let encoded = encode_command_envelope(&command, 1_000_000).unwrap();
        assert_eq!(
            decode_command_envelope(&encoded, 1_000_000).unwrap(),
            command
        );
    }

    #[test]
    fn expired_lease_and_command_are_rejected() {
        assert!(encode_resource_lease(&sample_lease(), 2_000_000).is_err());
        assert!(encode_command_envelope(&sample_command(), 1_500_000).is_err());
    }

    #[test]
    fn malformed_protobuf_is_rejected() {
        assert!(decode_header(&[0x0a, 0x05, 0x01]).is_err());
    }

    #[test]
    fn unknown_enum_is_rejected_at_the_single_boundary() {
        let mut value = header_to_wire(&sample());
        value.quality = 999;
        assert!(header_from_wire(value).is_err());
    }

    #[test]
    fn invalid_identity_length_is_rejected() {
        let mut value = header_to_wire(&sample());
        value.boot_id = vec![1, 2, 3];
        let bytes = value.encode_to_vec();
        assert!(decode_header(&bytes).is_err());
    }

    #[test]
    fn unspecified_clock_state_is_rejected() {
        let mut value = header_to_wire(&sample());
        value.clock_status = Some(wire::ClockStatus {
            state: wire::ClockState::Unspecified as i32,
            uncertainty_ns: 0,
        });
        assert!(header_from_wire(value).is_err());
    }
}
