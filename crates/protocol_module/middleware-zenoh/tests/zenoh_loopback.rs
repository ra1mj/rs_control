use std::{net::TcpListener, sync::mpsc, time::Duration};

use contract_codec::{
    decode_chassis_setpoint_intent, decode_imu_state, encode_chassis_setpoint_intent,
    encode_imu_state,
};
use message_model::{
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    DataQuality, Header, HeaderFields, Id128, ImuMeasurement, ImuState, MAX_TYPE_NAME_LEN,
    ResourceLease, ResourceLeaseFields, Twist2,
};
use middleware_api::{
    ConcreteKey, EndpointBounds, EndpointDescriptor, EndpointDirection, EndpointFields,
    EventPublishError, EventPublisher, EventSubscriber, FallbackBehavior, InteractionClass,
    KeyDomain, LatestPublish, LatestPublisher, LatestSubscriber, SemanticQos,
};
use middleware_local::{latest_channel, ordered_channel};
use middleware_zenoh::{
    BridgeError, BridgeRoutes, FreshnessEvidence, FreshnessPolicy, InboundRoute, IngressPolicy,
    LivelinessObserverRoute, OutboundRoute, QueryError, QueryableRoute, RouteHandlingError,
    SourceGuard, ZenohBridge, ZenohBridgeConfig, materialize_chassis_setpoint,
};

const TEST_TIMEOUT: Duration = Duration::from_secs(5);
const LOCAL_RECEIVE_NS: u64 = 100_000;
const ROUTE_MAXIMUM_TTL_NS: u64 = 20_000;

fn id<const N: usize>(value: &str) -> AsciiId<N> {
    AsciiId::parse(value).expect("test identifier is valid")
}

fn header(source: &str, boot: u8, sequence: u64, frame: Option<&str>) -> Header {
    Header::new(HeaderFields {
        source_id: id(source),
        boot_id: Id128::new([boot; 16]).unwrap(),
        sequence,
        source_monotonic_ns: 9_000,
        publish_unix_ns: Some(1_000_000),
        acquisition_timestamp: None,
        valid_for_ns: 100_000_000,
        frame_id: frame.map(id),
        clock_status: ClockStatus {
            state: ClockState::Synchronized,
            uncertainty_ns: 100,
        },
        quality: DataQuality::Valid,
    })
    .unwrap()
}

struct RouteSpec<'a> {
    endpoint_id: &'a str,
    resource: &'a str,
    channel: &'a str,
    protobuf_type: &'a str,
    direction: EndpointDirection,
    interaction: InteractionClass,
    qos: SemanticQos,
    allowed_source: Option<&'a str>,
}

fn descriptor(spec: RouteSpec<'_>) -> EndpointDescriptor {
    let request_like = matches!(
        spec.interaction,
        InteractionClass::ActionCommand
            | InteractionClass::ActionAck
            | InteractionClass::Query
            | InteractionClass::Queryable
    );
    let age_required = matches!(
        (spec.direction, spec.interaction),
        (
            EndpointDirection::Subscribe,
            InteractionClass::LatestStream | InteractionClass::ActionCommand
        )
    );
    EndpointDescriptor::new(EndpointFields {
        id: id(spec.endpoint_id),
        key: ConcreteKey::new(
            id("loopback_robot"),
            KeyDomain::Device,
            id(spec.resource),
            id(spec.channel),
        ),
        protobuf_type: id::<MAX_TYPE_NAME_LEN>(spec.protobuf_type),
        direction: spec.direction,
        interaction: spec.interaction,
        qos: spec.qos,
        bounds: EndpointBounds {
            max_payload_bytes: 4_096,
            queue_capacity: 8,
            max_age_ns: age_required.then_some(50_000_000),
            timeout_ns: request_like.then_some(2_000_000_000),
        },
        allowed_source: spec.allowed_source.map(id),
        fallback: FallbackBehavior::Unavailable,
    })
    .unwrap()
}

fn timed_query_descriptor(
    endpoint_id: &str,
    channel: &str,
    direction: EndpointDirection,
    interaction: InteractionClass,
    timeout_ns: u64,
) -> EndpointDescriptor {
    EndpointDescriptor::new(EndpointFields {
        id: id(endpoint_id),
        key: ConcreteKey::new(
            id("loopback_robot"),
            KeyDomain::Health,
            id("health"),
            id(channel),
        ),
        protobuf_type: id::<MAX_TYPE_NAME_LEN>("robot.v1.HealthSnapshotRequest"),
        direction,
        interaction,
        qos: SemanticQos::RequestReply,
        bounds: EndpointBounds {
            max_payload_bytes: 4_096,
            queue_capacity: 8,
            max_age_ns: None,
            timeout_ns: Some(timeout_ns),
        },
        allowed_source: None,
        fallback: FallbackBehavior::Unavailable,
    })
    .unwrap()
}

fn router_config(port: u16) -> ZenohBridgeConfig {
    ZenohBridgeConfig::from_json5(&format!(
        r#"{{
            mode: "router",
            listen: {{ endpoints: ["tcp/127.0.0.1:{port}"] }},
            scouting: {{
                multicast: {{ enabled: false }},
                gossip: {{ enabled: false }}
            }}
        }}"#
    ))
    .unwrap()
}

fn client_config(port: u16) -> ZenohBridgeConfig {
    ZenohBridgeConfig::from_json5(&format!(
        r#"{{
            mode: "client",
            connect: {{ endpoints: ["tcp/127.0.0.1:{port}"] }},
            scouting: {{
                multicast: {{ enabled: false }},
                gossip: {{ enabled: false }}
            }}
        }}"#
    ))
    .unwrap()
}

#[test]
fn real_tcp_loopback_preserves_semantics_authority_queries_and_liveliness() {
    let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = reservation.local_addr().unwrap().port();
    drop(reservation);

    let state_publish = descriptor(RouteSpec {
        endpoint_id: "state_publish",
        resource: "imu",
        channel: "state",
        protobuf_type: "robot.v1.ImuState",
        direction: EndpointDirection::Publish,
        interaction: InteractionClass::LatestStream,
        qos: SemanticQos::StateLatest,
        allowed_source: None,
    });
    let state_subscribe = descriptor(RouteSpec {
        endpoint_id: "state_subscribe",
        resource: "imu",
        channel: "state",
        protobuf_type: "robot.v1.ImuState",
        direction: EndpointDirection::Subscribe,
        interaction: InteractionClass::LatestStream,
        qos: SemanticQos::StateLatest,
        allowed_source: Some("imu_driver"),
    });
    let command_publish = descriptor(RouteSpec {
        endpoint_id: "command_publish",
        resource: "chassis",
        channel: "command_intent",
        protobuf_type: "robot.v1.ChassisSetpointIntent",
        direction: EndpointDirection::Publish,
        interaction: InteractionClass::ActionCommand,
        qos: SemanticQos::CorrelatedEvent,
        allowed_source: None,
    });
    let command_subscribe = descriptor(RouteSpec {
        endpoint_id: "command_subscribe",
        resource: "chassis",
        channel: "command_intent",
        protobuf_type: "robot.v1.ChassisSetpointIntent",
        direction: EndpointDirection::Subscribe,
        interaction: InteractionClass::ActionCommand,
        qos: SemanticQos::CorrelatedEvent,
        allowed_source: Some("remote_controller"),
    });
    let query = descriptor(RouteSpec {
        endpoint_id: "health_query",
        resource: "health",
        channel: "snapshot",
        protobuf_type: "robot.v1.HealthSnapshotRequest",
        direction: EndpointDirection::Query,
        interaction: InteractionClass::Query,
        qos: SemanticQos::RequestReply,
        allowed_source: None,
    });
    let queryable = descriptor(RouteSpec {
        endpoint_id: "health_queryable",
        resource: "health",
        channel: "snapshot",
        protobuf_type: "robot.v1.HealthSnapshotRequest",
        direction: EndpointDirection::Queryable,
        interaction: InteractionClass::Queryable,
        qos: SemanticQos::RequestReply,
        allowed_source: None,
    });
    let slow_query = timed_query_descriptor(
        "slow_query",
        "slow_snapshot",
        EndpointDirection::Query,
        InteractionClass::Query,
        50_000_000,
    );
    let slow_queryable = timed_query_descriptor(
        "slow_queryable",
        "slow_snapshot",
        EndpointDirection::Queryable,
        InteractionClass::Queryable,
        50_000_000,
    );
    let announce = descriptor(RouteSpec {
        endpoint_id: "health_announce",
        resource: "bridge",
        channel: "presence",
        protobuf_type: "robot.v1.ComponentHealth",
        direction: EndpointDirection::AnnounceLiveliness,
        interaction: InteractionClass::Liveliness,
        qos: SemanticQos::Presence,
        allowed_source: None,
    });
    let observe = descriptor(RouteSpec {
        endpoint_id: "health_observe",
        resource: "bridge",
        channel: "presence",
        protobuf_type: "robot.v1.ComponentHealth",
        direction: EndpointDirection::ObserveLiveliness,
        interaction: InteractionClass::Liveliness,
        qos: SemanticQos::Presence,
        allowed_source: None,
    });

    let expected_state = ImuState::new(
        header("imu_driver", 1, 1, Some("imu_link")),
        ImuMeasurement::Unavailable,
        ImuMeasurement::Unavailable,
        ImuMeasurement::Unavailable,
    )
    .unwrap();
    let (mut state_writer, mut state_reader) = latest_channel();
    let (state_tx, state_rx) = mpsc::channel();

    let lease = ResourceLease::new(ResourceLeaseFields {
        resource_id: id("chassis"),
        owner_id: id("remote_operator"),
        lease_id: Id128::new([7; 16]).unwrap(),
        session_epoch: 4,
        expires_monotonic_ns: 200_000,
    })
    .unwrap();
    let command_intent = ChassisSetpointIntent::new(
        CommandIntent::new(CommandIntentFields {
            header: header("remote_controller", 2, 1, None),
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id: Id128::new([8; 16]).unwrap(),
            requested_ttl_ns: 80_000,
        })
        .unwrap(),
        Twist2::new(0.5, 0.0, 0.1).unwrap(),
    );
    let (mut command_writer, mut command_reader) = ordered_channel(4).unwrap();
    let (command_tx, command_rx) = mpsc::channel();
    let (presence_tx, presence_rx) = mpsc::channel();

    let mut guard = SourceGuard::<4>::new(id("remote_controller"));
    guard
        .authorize_boot(command_intent.intent().header().boot_id())
        .unwrap();

    let mut router_routes = BridgeRoutes::new();
    router_routes.add_inbound(
        InboundRoute::new(state_subscribe, move |payload| {
            let state = decode_imu_state(&payload).map_err(|_| RouteHandlingError::Malformed)?;
            state_tx
                .send(state)
                .map_err(|_| RouteHandlingError::Rejected)
        })
        .unwrap(),
    );
    router_routes.add_inbound(
        InboundRoute::new(command_subscribe, move |payload| {
            let intent = decode_chassis_setpoint_intent(&payload)
                .map_err(|_| RouteHandlingError::Malformed)?;
            let accepted = guard
                .accept(
                    intent.intent().header(),
                    intent,
                    LOCAL_RECEIVE_NS,
                    IngressPolicy {
                        max_age_ns: 50_000_000,
                        freshness: FreshnessPolicy::ReceptionBounded,
                        allow_degraded: false,
                    },
                    FreshnessEvidence::ReceptionOnly,
                )
                .map_err(|_| RouteHandlingError::Rejected)?;
            let setpoint = materialize_chassis_setpoint(
                accepted.value,
                lease,
                LOCAL_RECEIVE_NS,
                ROUTE_MAXIMUM_TTL_NS,
            )
            .map_err(|_| RouteHandlingError::Rejected)?;
            command_tx
                .send(setpoint)
                .map_err(|_| RouteHandlingError::Rejected)
        })
        .unwrap(),
    );
    router_routes.add_queryable(
        QueryableRoute::new(queryable, |request| {
            if request == b"health?" {
                Ok(b"healthy".to_vec())
            } else {
                Err(b"bad request".to_vec())
            }
        })
        .unwrap(),
    );
    router_routes.add_queryable(
        QueryableRoute::new(slow_queryable, |_| {
            std::thread::sleep(Duration::from_millis(200));
            Ok(b"too late".to_vec())
        })
        .unwrap(),
    );
    router_routes.add_liveliness_announcement(announce).unwrap();
    let router = ZenohBridge::spawn(router_config(port), router_routes).unwrap();

    let mut client_routes = BridgeRoutes::new();
    client_routes.add_outbound(
        OutboundRoute::new(state_publish, move || {
            Ok(state_reader
                .try_read_latest()
                .map(|sample| encode_imu_state(&sample.value)))
        })
        .unwrap(),
    );
    client_routes.add_outbound(
        OutboundRoute::new(command_publish, move || {
            Ok(command_reader
                .try_receive()
                .map(|sample| encode_chassis_setpoint_intent(&sample.value)))
        })
        .unwrap(),
    );
    client_routes.add_query(query).unwrap();
    client_routes.add_query(slow_query).unwrap();
    client_routes.add_liveliness_observer(
        LivelinessObserverRoute::new(observe, move |available| {
            let _ = presence_tx.send(available);
        })
        .unwrap(),
    );
    let client = ZenohBridge::spawn(client_config(port), client_routes).unwrap();

    state_writer.try_publish(expected_state);
    command_writer.try_publish(command_intent).unwrap();

    assert_eq!(state_rx.recv_timeout(TEST_TIMEOUT).unwrap(), expected_state);
    let local_command = command_rx.recv_timeout(TEST_TIMEOUT).unwrap();
    assert_eq!(local_command.target, command_intent.target());
    assert_eq!(
        local_command.envelope.deadline_monotonic_ns(),
        LOCAL_RECEIVE_NS + ROUTE_MAXIMUM_TTL_NS
    );
    assert_eq!(
        client
            .query(query.fields().id, b"health?".to_vec())
            .unwrap(),
        b"healthy"
    );
    assert!(matches!(
        client.query(slow_query.fields().id, b"wait".to_vec()),
        Err(BridgeError::Query(QueryError::Timeout))
    ));
    assert!(presence_rx.recv_timeout(TEST_TIMEOUT).unwrap());

    let router_counters = router.stop().unwrap();
    assert!(!presence_rx.recv_timeout(TEST_TIMEOUT).unwrap());
    let client_counters = client.stop().unwrap();
    assert_eq!(
        state_writer.try_publish(expected_state),
        LatestPublish::Disconnected
    );
    assert_eq!(
        command_writer.try_publish(command_intent),
        Err(EventPublishError::Disconnected(command_intent))
    );

    assert!(router_counters.received >= 3);
    assert_eq!(router_counters.malformed, 0);
    assert_eq!(router_counters.rejected, 0);
    assert_eq!(client_counters.published, 2);
    assert_eq!(client_counters.transport_errors, 0);
}
