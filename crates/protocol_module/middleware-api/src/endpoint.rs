use message_model::{AsciiId, MAX_ID_LEN, ProtobufTypeName, SourceId};
use thiserror::Error;

use crate::ConcreteKey;

/// Stable endpoint identity.
pub type EndpointId = AsciiId<MAX_ID_LEN>;

/// Direction of one middleware declaration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EndpointDirection {
    /// Publish values or events.
    Publish,
    /// Subscribe to values or events.
    Subscribe,
    /// Issue a bounded query.
    Query,
    /// Serve bounded queries.
    Queryable,
    /// Announce one liveliness identity.
    AnnounceLiveliness,
    /// Observe liveliness identities.
    ObserveLiveliness,
}

/// Semantic interaction independent of a middleware primitive.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InteractionClass {
    /// Latest-value stream.
    LatestStream,
    /// Ordered discrete event stream.
    OrderedEvent,
    /// Authority-bearing action request.
    ActionCommand,
    /// Correlated acknowledgement or terminal event.
    ActionAck,
    /// Bounded request.
    Query,
    /// Bounded request handler.
    Queryable,
    /// Presence observation.
    Liveliness,
}

/// Project-owned QoS intent, deliberately narrower than Zenoh QoS.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SemanticQos {
    /// Latest valid motion-control input.
    ControlLatest,
    /// High-rate state where sequence gaps are observable.
    StateLatest,
    /// Discrete event completed through application correlation.
    CorrelatedEvent,
    /// Bounded request/reply operation.
    RequestReply,
    /// Availability observation.
    Presence,
    /// Explicitly lossy bulk data.
    BulkLossy,
}

/// Required behavior after a network input becomes unavailable.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FallbackBehavior {
    /// Retain the last sample only until its local expiry.
    HoldUntilExpired,
    /// Immediately expose unavailable state.
    Unavailable,
    /// Emit a typed safe-stop request for the policy owner.
    SafeStopSignal,
}

/// Nonzero endpoint resource bounds.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointBounds {
    /// Maximum encoded payload accepted before decode.
    pub max_payload_bytes: usize,
    /// Maximum local/adapter queued items.
    pub queue_capacity: usize,
    /// Maximum accepted input age for stream/command routes.
    pub max_age_ns: Option<u64>,
    /// Maximum request/retry duration for request-like routes.
    pub timeout_ns: Option<u64>,
}

/// Fields supplied to [`EndpointDescriptor::new`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointFields {
    /// Stable endpoint identity.
    pub id: EndpointId,
    /// Exact concrete routing key.
    pub key: ConcreteKey,
    /// Exact registered Protobuf type.
    pub protobuf_type: ProtobufTypeName,
    /// Declaration direction.
    pub direction: EndpointDirection,
    /// Semantic interaction.
    pub interaction: InteractionClass,
    /// Semantic QoS intent.
    pub qos: SemanticQos,
    /// Resource limits.
    pub bounds: EndpointBounds,
    /// Exact accepted source for single-writer ingress.
    pub allowed_source: Option<SourceId>,
    /// Behavior exposed when input is unavailable.
    pub fallback: FallbackBehavior,
}

/// Fully validated endpoint contract.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EndpointDescriptor(EndpointFields);

impl EndpointDescriptor {
    /// Validates an endpoint contract.
    pub fn new(fields: EndpointFields) -> Result<Self, EndpointError> {
        if fields.bounds.max_payload_bytes == 0 {
            return Err(EndpointError::ZeroBound("max_payload_bytes"));
        }
        if fields.bounds.queue_capacity == 0 {
            return Err(EndpointError::ZeroBound("queue_capacity"));
        }
        validate_direction(fields.direction, fields.interaction)?;
        validate_qos(fields.qos, fields.interaction)?;

        let request_like = matches!(
            fields.interaction,
            InteractionClass::ActionCommand
                | InteractionClass::ActionAck
                | InteractionClass::Query
                | InteractionClass::Queryable
        );
        if request_like && fields.bounds.timeout_ns.is_none_or(|value| value == 0) {
            return Err(EndpointError::MissingBound("timeout_ns"));
        }
        let age_required = matches!(
            (fields.direction, fields.interaction),
            (
                EndpointDirection::Subscribe,
                InteractionClass::LatestStream | InteractionClass::ActionCommand
            )
        );
        if age_required && fields.bounds.max_age_ns.is_none_or(|value| value == 0) {
            return Err(EndpointError::MissingBound("max_age_ns"));
        }
        if age_required && fields.allowed_source.is_none() {
            return Err(EndpointError::MissingAllowedSource);
        }
        if fields.fallback == FallbackBehavior::SafeStopSignal
            && fields.direction != EndpointDirection::Subscribe
        {
            return Err(EndpointError::InvalidFallback);
        }
        Ok(Self(fields))
    }

    /// Returns the validated fields.
    #[must_use]
    pub const fn fields(self) -> EndpointFields {
        self.0
    }
}

fn validate_direction(
    direction: EndpointDirection,
    interaction: InteractionClass,
) -> Result<(), EndpointError> {
    let compatible = match interaction {
        InteractionClass::LatestStream
        | InteractionClass::OrderedEvent
        | InteractionClass::ActionCommand
        | InteractionClass::ActionAck => {
            matches!(
                direction,
                EndpointDirection::Publish | EndpointDirection::Subscribe
            )
        }
        InteractionClass::Query => direction == EndpointDirection::Query,
        InteractionClass::Queryable => direction == EndpointDirection::Queryable,
        InteractionClass::Liveliness => matches!(
            direction,
            EndpointDirection::AnnounceLiveliness | EndpointDirection::ObserveLiveliness
        ),
    };
    if compatible {
        Ok(())
    } else {
        Err(EndpointError::IncompatibleDirection)
    }
}

fn validate_qos(qos: SemanticQos, interaction: InteractionClass) -> Result<(), EndpointError> {
    let compatible = match qos {
        SemanticQos::ControlLatest | SemanticQos::StateLatest | SemanticQos::BulkLossy => {
            interaction == InteractionClass::LatestStream
        }
        SemanticQos::CorrelatedEvent => matches!(
            interaction,
            InteractionClass::OrderedEvent
                | InteractionClass::ActionCommand
                | InteractionClass::ActionAck
        ),
        SemanticQos::RequestReply => {
            matches!(
                interaction,
                InteractionClass::Query | InteractionClass::Queryable
            )
        }
        SemanticQos::Presence => interaction == InteractionClass::Liveliness,
    };
    if compatible {
        Ok(())
    } else {
        Err(EndpointError::IncompatibleQos)
    }
}

/// Validates duplicate ownership across one bounded route set.
pub fn validate_route_set(routes: &[EndpointDescriptor]) -> Result<(), EndpointError> {
    for (index, route) in routes.iter().enumerate() {
        let fields = route.fields();
        for prior in &routes[..index] {
            let prior = prior.fields();
            if prior.id == fields.id {
                return Err(EndpointError::DuplicateEndpointId);
            }
            if prior.key == fields.key && prior.direction == fields.direction {
                return Err(EndpointError::DuplicateKeyOwnership);
            }
        }
    }
    Ok(())
}

/// Invalid endpoint contract.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum EndpointError {
    /// A required numeric bound was zero.
    #[error("endpoint bound {0} must be nonzero")]
    ZeroBound(&'static str),
    /// A route class omitted a required optional bound.
    #[error("endpoint requires a nonzero {0}")]
    MissingBound(&'static str),
    /// Direction and interaction do not map to one primitive.
    #[error("endpoint direction is incompatible with its interaction class")]
    IncompatibleDirection,
    /// Semantic QoS and interaction disagree.
    #[error("semantic QoS is incompatible with the interaction class")]
    IncompatibleQos,
    /// A single-writer ingress route omitted its source owner.
    #[error("ingress latest/command endpoint requires an allowed source")]
    MissingAllowedSource,
    /// Safe-stop fallback was assigned to a non-ingress endpoint.
    #[error("safe-stop fallback is valid only for subscribed input")]
    InvalidFallback,
    /// Two routes reused one endpoint identity.
    #[error("duplicate endpoint id")]
    DuplicateEndpointId,
    /// Two declarations own the same key/direction pair.
    #[error("duplicate key/direction ownership")]
    DuplicateKeyOwnership,
}

#[cfg(test)]
mod tests {
    use message_model::{AsciiId, MAX_TYPE_NAME_LEN};

    use super::*;
    use crate::KeyDomain;

    fn id<const N: usize>(value: &str) -> AsciiId<N> {
        AsciiId::parse(value).expect("test identifier is valid")
    }

    fn fields() -> EndpointFields {
        EndpointFields {
            id: id("imu_state"),
            key: ConcreteKey::new(id("r1"), KeyDomain::Device, id("imu"), id("state")),
            protobuf_type: id::<MAX_TYPE_NAME_LEN>("robot.v1.ImuState"),
            direction: EndpointDirection::Subscribe,
            interaction: InteractionClass::LatestStream,
            qos: SemanticQos::StateLatest,
            bounds: EndpointBounds {
                max_payload_bytes: 4096,
                queue_capacity: 8,
                max_age_ns: Some(20_000_000),
                timeout_ns: None,
            },
            allowed_source: Some(id("imu_driver")),
            fallback: FallbackBehavior::HoldUntilExpired,
        }
    }

    #[test]
    fn valid_latest_route_is_accepted() {
        assert!(EndpointDescriptor::new(fields()).is_ok());
    }

    #[test]
    fn invalid_bounds_direction_and_qos_are_rejected() {
        let mut value = fields();
        value.bounds.max_payload_bytes = 0;
        assert_eq!(
            EndpointDescriptor::new(value),
            Err(EndpointError::ZeroBound("max_payload_bytes"))
        );

        let mut value = fields();
        value.direction = EndpointDirection::Query;
        assert_eq!(
            EndpointDescriptor::new(value),
            Err(EndpointError::IncompatibleDirection)
        );

        let mut value = fields();
        value.qos = SemanticQos::Presence;
        assert_eq!(
            EndpointDescriptor::new(value),
            Err(EndpointError::IncompatibleQos)
        );
    }

    #[test]
    fn route_set_rejects_duplicate_identity_and_ownership() {
        let first = EndpointDescriptor::new(fields()).expect("route is valid");
        assert_eq!(
            validate_route_set(&[first, first]),
            Err(EndpointError::DuplicateEndpointId)
        );

        let mut second = fields();
        second.id = id("imu_state_two");
        let second = EndpointDescriptor::new(second).expect("route is valid");
        assert_eq!(
            validate_route_set(&[first, second]),
            Err(EndpointError::DuplicateKeyOwnership)
        );
    }
}
