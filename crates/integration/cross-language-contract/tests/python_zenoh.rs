use std::{
    ffi::OsStr,
    fs,
    io::{BufRead, BufReader},
    net::TcpListener,
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::mpsc::{self, Receiver, RecvTimeoutError, SyncSender, TrySendError},
    thread,
    time::Duration,
};

use contract_codec::{decode_chassis_setpoint_intent, encode_odometry_state};
use message_model::{
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, Covariance6, DataQuality, Header,
    HeaderFields, Id128, MAX_TYPE_NAME_LEN, OdometryState, OdometryStateFields, Pose3,
    ResourceLease, ResourceLeaseFields, Twist3, UnitQuaternion, Vector3,
};
use middleware_api::{
    ConcreteKey, EndpointBounds, EndpointDescriptor, EndpointDirection, EndpointFields,
    FallbackBehavior, InteractionClass, KeyDomain, LatestPublish, LatestPublisher,
    LatestSubscriber, SemanticQos,
};
use middleware_local::latest_channel;
use middleware_zenoh::{
    BridgeRoutes, FreshnessEvidence, FreshnessPolicy, InboundRoute, IngressPolicy, OutboundRoute,
    RouteHandlingError, SourceGuard, ZenohBridge, ZenohBridgeConfig, materialize_chassis_setpoint,
};

const TEST_TIMEOUT: Duration = Duration::from_secs(8);
const QUIET_TIMEOUT: Duration = Duration::from_millis(300);
const FIRST_BOOT: [u8; 16] = [0x11; 16];
const SECOND_BOOT: [u8; 16] = [0x22; 16];
const LEASE_ID: [u8; 16] = [0x07; 16];
const LOCAL_RECEIVE_NS: u64 = 1_000_000;
const ROUTE_MAXIMUM_TTL_NS: u64 = 5_000_000;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../..")
}

fn id<const N: usize>(value: &str) -> AsciiId<N> {
    AsciiId::parse(value).expect("test identifier is valid")
}

fn header(sequence: u64) -> Header {
    Header::new(HeaderFields {
        source_id: id("odometry_estimator"),
        boot_id: Id128::new([1; 16]).unwrap(),
        sequence,
        source_monotonic_ns: 100_000 + sequence,
        publish_unix_ns: Some(1_000_000 + sequence),
        acquisition_timestamp: None,
        valid_for_ns: 100_000_000,
        frame_id: Some(id("odom")),
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
    .unwrap()
}

fn odometry(sequence: u64) -> OdometryState {
    let half_yaw = 0.2_f64;
    OdometryState::new(OdometryStateFields {
        header: header(sequence),
        parent_frame_id: id("odom"),
        child_frame_id: id("base_link"),
        pose: Pose3 {
            position_m: Vector3::new(1.0, 2.0, 0.0).unwrap(),
            orientation: UnitQuaternion::new(0.0, 0.0, half_yaw.sin(), half_yaw.cos()).unwrap(),
        },
        twist: Twist3 {
            linear_mps: Vector3::new(0.0, 0.0, 0.0).unwrap(),
            angular_radps: Vector3::new(0.0, 0.0, 0.0).unwrap(),
        },
        pose_covariance: Covariance6::new([0.0; 36]).unwrap(),
        twist_covariance: Covariance6::new([0.0; 36]).unwrap(),
    })
    .unwrap()
}

struct RouteSpec<'a> {
    id: &'a str,
    domain: KeyDomain,
    resource: &'a str,
    channel: &'a str,
    protobuf_type: &'a str,
    direction: EndpointDirection,
    interaction: InteractionClass,
    qos: SemanticQos,
    allowed_source: Option<&'a str>,
}

fn descriptor(spec: RouteSpec<'_>) -> EndpointDescriptor {
    EndpointDescriptor::new(EndpointFields {
        id: id(spec.id),
        key: ConcreteKey::new(id("demo"), spec.domain, id(spec.resource), id(spec.channel)),
        protobuf_type: id::<MAX_TYPE_NAME_LEN>(spec.protobuf_type),
        direction: spec.direction,
        interaction: spec.interaction,
        qos: spec.qos,
        bounds: EndpointBounds {
            max_payload_bytes: 8_192,
            queue_capacity: 8,
            max_age_ns: (spec.direction == EndpointDirection::Subscribe).then_some(100_000_000),
            timeout_ns: matches!(spec.interaction, InteractionClass::ActionCommand)
                .then_some(100_000_000),
        },
        allowed_source: spec.allowed_source.map(id),
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

fn generate_python_clients(output_root: &Path) {
    let root = workspace_root();
    let proto_root = root.join("proto");
    let schema_root = proto_root.join("robot/v1");
    let mut schemas = fs::read_dir(&schema_root)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| path.extension() == Some(OsStr::new("proto")))
        .collect::<Vec<_>>();
    schemas.sort();
    let status = Command::new(protoc_bin_vendored::protoc_bin_path().unwrap())
        .arg(format!("--proto_path={}", proto_root.display()))
        .arg(format!("--python_out={}", output_root.display()))
        .args(schemas)
        .status()
        .unwrap();
    assert!(status.success(), "Python Protobuf generation failed");
}

struct AlgorithmProcess {
    child: Child,
    output: Receiver<String>,
}

impl AlgorithmProcess {
    fn spawn(port: u16, input_key: &str, output_key: &str, boot_id: [u8; 16]) -> Self {
        let root = workspace_root();
        let project = root.join("examples/python-zenoh-algorithm");
        let generated = GENERATED_ROOT
            .get()
            .expect("generated Python path was initialized");
        let mut child = Command::new(project.join(".venv/bin/python"))
            .arg(project.join("heading_hold.py"))
            .args(["--connect", &format!("tcp/127.0.0.1:{port}")])
            .args(["--input-key", input_key])
            .args(["--output-key", output_key])
            .args(["--owner-id", "remote_operator"])
            .args(["--lease-id", &hex_id(LEASE_ID)])
            .args(["--session-epoch", "4"])
            .args(["--requested-ttl-ns", "20000000"])
            .args(["--heading-gain", "0.5"])
            .args(["--maximum-yaw-rate-radps", "0.3"])
            .args(["--boot-id", &hex_id(boot_id)])
            .env("PYTHONPATH", generated)
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap();
        let stdout = child.stdout.take().unwrap();
        let (sender, output) = mpsc::sync_channel(16);
        thread::spawn(move || {
            for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                if sender.send(line).is_err() {
                    break;
                }
            }
        });
        Self { child, output }
    }

    fn wait_ready(&self) {
        loop {
            let line = self
                .output
                .recv_timeout(TEST_TIMEOUT)
                .expect("algorithm did not report readiness");
            if line.contains(r#""event": "ready""#) {
                break;
            }
        }
    }

    fn force_kill(&mut self) {
        self.child.kill().unwrap();
        let status = self.child.wait().unwrap();
        assert!(!status.success());
    }
}

impl Drop for AlgorithmProcess {
    fn drop(&mut self) {
        if self.child.try_wait().ok().flatten().is_none() {
            let _ = self.child.kill();
            let _ = self.child.wait();
        }
    }
}

fn hex_id(bytes: [u8; 16]) -> String {
    bytes.iter().map(|byte| format!("{byte:02x}")).collect()
}

static GENERATED_ROOT: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();

fn prepare_python_environment() {
    let project = workspace_root().join("examples/python-zenoh-algorithm");
    let status = Command::new("uv")
        .args(["sync", "--project"])
        .arg(&project)
        .arg("--frozen")
        .status()
        .unwrap();
    assert!(status.success(), "locked Python environment sync failed");
}

fn receive_intent(
    writer: &mut impl LatestPublisher<OdometryState>,
    receiver: &Receiver<ChassisSetpointIntent>,
    start_sequence: u64,
) -> ChassisSetpointIntent {
    for offset in 0..100 {
        assert_ne!(
            writer.try_publish(odometry(start_sequence + offset)),
            LatestPublish::Disconnected
        );
        match receiver.recv_timeout(Duration::from_millis(50)) {
            Ok(intent) => return intent,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => panic!("intent receiver disconnected"),
        }
    }
    panic!("algorithm did not publish a command intent");
}

fn bounded_send(
    sender: &SyncSender<ChassisSetpointIntent>,
    value: ChassisSetpointIntent,
) -> Result<(), RouteHandlingError> {
    sender.try_send(value).map_err(|error| match error {
        TrySendError::Full(_) | TrySendError::Disconnected(_) => RouteHandlingError::Rejected,
    })
}

#[test]
fn generated_python_process_is_crash_isolated_and_reauthorizes_restart() {
    prepare_python_environment();
    let generated = tempfile::tempdir().unwrap();
    generate_python_clients(generated.path());
    GENERATED_ROOT
        .set(generated.path().to_path_buf())
        .expect("test initializes generated root only once");

    let reservation = TcpListener::bind("127.0.0.1:0").unwrap();
    let port = reservation.local_addr().unwrap().port();
    drop(reservation);

    let odometry_publish = descriptor(RouteSpec {
        id: "odometry_publish",
        domain: KeyDomain::Navigation,
        resource: "odometry",
        channel: "state",
        protobuf_type: "robot.v1.OdometryState",
        direction: EndpointDirection::Publish,
        interaction: InteractionClass::LatestStream,
        qos: SemanticQos::StateLatest,
        allowed_source: None,
    });
    let command_subscribe = descriptor(RouteSpec {
        id: "heading_command_subscribe",
        domain: KeyDomain::Control,
        resource: "chassis",
        channel: "command_intent",
        protobuf_type: "robot.v1.ChassisSetpointIntent",
        direction: EndpointDirection::Subscribe,
        interaction: InteractionClass::ActionCommand,
        qos: SemanticQos::CorrelatedEvent,
        allowed_source: Some("python_heading_hold"),
    });
    let input_key = odometry_publish.fields().key.to_string();
    let output_key = command_subscribe.fields().key.to_string();

    let (mut odometry_writer, mut odometry_reader) = latest_channel();
    let (intent_sender, intent_receiver) = mpsc::sync_channel(8);
    let mut routes = BridgeRoutes::new();
    routes.add_outbound(
        OutboundRoute::new(odometry_publish, move || {
            Ok(odometry_reader
                .try_read_latest()
                .map(|sample| encode_odometry_state(&sample.value)))
        })
        .unwrap(),
    );
    routes.add_inbound(
        InboundRoute::new(command_subscribe, move |payload| {
            let intent = decode_chassis_setpoint_intent(&payload)
                .map_err(|_| RouteHandlingError::Malformed)?;
            bounded_send(&intent_sender, intent)
        })
        .unwrap(),
    );
    let bridge = ZenohBridge::spawn(router_config(port), routes).unwrap();

    let lease = ResourceLease::new(ResourceLeaseFields {
        resource_id: id("chassis"),
        owner_id: id("remote_operator"),
        lease_id: Id128::new(LEASE_ID).unwrap(),
        session_epoch: 4,
        expires_monotonic_ns: 100_000_000,
    })
    .unwrap();
    let policy = IngressPolicy {
        max_age_ns: 100_000_000,
        freshness: FreshnessPolicy::ReceptionBounded,
        allow_degraded: false,
    };
    let mut guard = SourceGuard::<4>::new(id("python_heading_hold"));
    guard
        .authorize_boot(Id128::new(FIRST_BOOT).unwrap())
        .unwrap();

    let mut first = AlgorithmProcess::spawn(port, &input_key, &output_key, FIRST_BOOT);
    first.wait_ready();
    let first_intent = receive_intent(&mut odometry_writer, &intent_receiver, 1);
    let accepted = guard
        .accept(
            first_intent.intent().header(),
            first_intent,
            LOCAL_RECEIVE_NS,
            policy,
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();
    let setpoint = materialize_chassis_setpoint(
        *accepted.value(),
        lease,
        LOCAL_RECEIVE_NS,
        ROUTE_MAXIMUM_TTL_NS,
    )
    .unwrap();
    let target = setpoint.target.components();
    assert_eq!(target[0..2], [0.0, 0.0]);
    assert!((target[2] + 0.2).abs() <= f64::EPSILON);
    assert_eq!(
        setpoint.envelope.deadline_monotonic_ns(),
        LOCAL_RECEIVE_NS + ROUTE_MAXIMUM_TTL_NS
    );

    first.force_kill();
    guard.mark_disconnected();
    while intent_receiver
        .recv_timeout(Duration::from_millis(50))
        .is_ok()
    {}
    for sequence in 101..1_101 {
        assert_ne!(
            odometry_writer.try_publish(odometry(sequence)),
            LatestPublish::Disconnected
        );
    }
    assert!(matches!(
        intent_receiver.recv_timeout(QUIET_TIMEOUT),
        Err(RecvTimeoutError::Timeout)
    ));
    assert!(
        setpoint
            .envelope
            .validate_at(setpoint.envelope.deadline_monotonic_ns())
            .is_err()
    );
    assert!(matches!(
        guard.evaluate_input(
            FallbackBehavior::Unavailable,
            Some(&accepted),
            LOCAL_RECEIVE_NS + 1,
        ),
        middleware_api::NetworkInputDecision::Unavailable
    ));

    guard
        .authorize_boot(Id128::new(SECOND_BOOT).unwrap())
        .unwrap();
    let mut second = AlgorithmProcess::spawn(port, &input_key, &output_key, SECOND_BOOT);
    second.wait_ready();
    let second_intent = receive_intent(&mut odometry_writer, &intent_receiver, 2_000);
    assert_eq!(
        second_intent.intent().header().boot_id(),
        Id128::new(SECOND_BOOT).unwrap()
    );
    guard
        .accept(
            second_intent.intent().header(),
            second_intent,
            LOCAL_RECEIVE_NS + 10_000_000,
            policy,
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();
    second.force_kill();

    let counters = bridge.stop().unwrap();
    assert!(counters.published > 0);
    assert!(counters.received >= 2);
    assert_eq!(counters.malformed, 0);
    assert_eq!(guard.counters().accepted, 2);
}

#[test]
fn python_dependencies_and_import_boundary_are_exact() {
    let root = workspace_root();
    let project = root.join("examples/python-zenoh-algorithm");
    let pyproject = fs::read_to_string(project.join("pyproject.toml")).unwrap();
    let lock = fs::read_to_string(project.join("uv.lock")).unwrap();
    assert!(pyproject.contains(r#""eclipse-zenoh==1.9.0""#));
    assert!(pyproject.contains(r#""protobuf==7.35.1""#));
    assert!(lock.contains("name = \"eclipse-zenoh\"\nversion = \"1.9.0\""));
    assert!(lock.contains("name = \"protobuf\"\nversion = \"7.35.1\""));

    const AST_CHECK: &str = r#"
import ast
import pathlib
import sys

allowed = {
    "__future__", "argparse", "collections", "google", "json", "math",
    "robot", "signal", "sys", "threading", "time", "uuid", "zenoh"
}
tree = ast.parse(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
roots = set()
for node in ast.walk(tree):
    if isinstance(node, ast.Import):
        roots.update(alias.name.split(".", 1)[0] for alias in node.names)
    elif isinstance(node, ast.ImportFrom) and node.module:
        roots.add(node.module.split(".", 1)[0])
unexpected = sorted(roots - allowed)
if unexpected:
    raise SystemExit("unexpected imports: " + ", ".join(unexpected))
"#;
    let status = Command::new("python3")
        .args(["-c", AST_CHECK])
        .arg(project.join("heading_hold.py"))
        .status()
        .unwrap();
    assert!(status.success(), "Python import boundary check failed");
}
