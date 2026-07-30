# Robot Cross-Layer Contracts

## Scenario: Add or change a robot contract

### 1. Scope / Trigger

Use this contract whenever work changes a `hardware_module` trait, robot YAML field, semantic `protocol_module` message, Protobuf field, command authority field, codec or composition-root startup mapping. These are safety/cross-layer changes even when the diff is small.

### 2. Signatures

Current executable commands:

```text
robotctl config validate <path>
robotctl config print-normalized <path>
architecture-check check [--workspace-root <path>] [--policy <path>]
```

Core Rust boundaries:

```rust
RobotConfig::parse(yaml: &str) -> Result<RobotConfig, ConfigError>
Header::new(fields: HeaderFields) -> Result<Header, SemanticError>
ResourceLease::new(fields: ResourceLeaseFields) -> Result<ResourceLease, SemanticError>
CommandEnvelope::new(fields: CommandEnvelopeFields) -> Result<CommandEnvelope, SemanticError>
FrameTransport::{receive, transmit, health}
DeviceDriver::{lifecycle, start_configure, start_enable, start_disable,
              handle_input, poll, encode_command, report_transport_fault}
```

### 3. Contracts

Robot YAML allows only:

```yaml
schema_version: 1
chassis: { morphology: <compiled chassis>, motor_ids: <role -> id> }
arm: { morphology: <profile name>, motor_ids: <role -> id> }
imu: { transport: uart|usb, baud_rate: <1200..4000000> }
```

It never contains frame layouts, checksums, controller gains, device paths, Zenoh settings or executable expressions.

Every external `Header` contains source ID, 16-byte non-zero boot ID, boot-local sequence, source monotonic timestamp, optional Unix correlation timestamp, optional acquisition timestamp with an explicit Unix/ROS domain, non-zero validity, optional frame ID, clock state/uncertainty and quality. Commands additionally carry resource ID, owner ID, 16-byte lease/command IDs, non-zero session epoch and deadline. Acquisition/ROS time is observation metadata and never substitutes for the local monotonic enforcement clock.

Protobuf package and key-space major version are `robot.v1`. Enums reserve zero as `<ENUM>_UNSPECIFIED`; removed field numbers are reserved and never reused. SI units appear in field names/comments. Coordinates are right-handed: x forward, y left, z up, radians for angles.

Local cross-process fast paths remain protocol-module implementations behind the
same semantic ports. Shared memory uses an explicit fixed-layout payload
contract, layout magic/version/schema identity, bounded capacity, cache-line
separated SPSC ownership, torn-read detection, crash-reclaim leases and
sequence/drop counters. `application_module` code never opens the shared-memory object. Rust
`needs_drop::<T>() == false` is not evidence that a type is safe for shared
memory; any mmap/raw-atomic `unsafe` stays private to a reviewed `protocol_module` boundary.

Classic-CAN device paths use `StandardCanId`, fixed-storage
`ClassicCanFrame`, local-monotonic `ReceivedCanFrame` and raw `CanErrorEvent`
contracts from `hardware-module-core`. Diagnostic socket/kernel/hardware
timestamps retain explicit clock-domain tags and are never compared with
`MonoTime`. Device lifecycle output is appended to caller-owned `FrameBatch`
storage; configuration and enable/disable are bounded multi-frame handshakes,
not successful-write aliases.

Damiao robot YAML IDs are expected `ESC_ID` values. The vendor boundary narrows
them to `1..=15`, discovers `MST_ID`, `TIMEOUT`, `CTRL_MODE`, `PMAX`, `VMAX` and
`TMAX` read-only while disarmed, validates the complete bus ID set, and requires
an explicit enable after verification. Runtime writes, zero-position and flash
store are maintenance-only codecs with no ordinary driver call path.

### 4. Validation & Error Matrix

| Condition | Owner | Result |
|---|---|---|
| Unknown YAML field/type | `config-model` parser | `ConfigError::Syntax` |
| Unsupported schema version | `config-model` | `UnsupportedSchemaVersion` |
| Chassis roles differ from morphology | `config-model` | `RoleMismatch` |
| Duplicate/out-of-range group motor ID | `config-model` | typed duplicate/range error |
| Non-`none` arm role mismatch | later `application_module` profile + composition root | refuse startup/arming |
| Missing wire submessage | `contract-codec` | `CodecError::MissingField` |
| Unknown/zero wire enum | `contract-codec` | `CodecError::UnknownEnum` |
| ID not 16 bytes/all zero | `message-model` | `Id128Error` |
| Zero validity/session/deadline | `message-model` constructor | `SemanticError` |
| Expired lease/command | enforcement/codec at supplied monotonic time | `SemanticError::Expired` |
| Forbidden Cargo edge | `architecture-check` | non-zero exit with all violations |
| Incompatible SHM layout/schema/endianness | `protocol_module` local IPC boundary | reject attach before exposing a port |
| Full/slow/crashed SHM endpoint | `protocol_module` queue policy | explicit latest/drop/reject result plus counters; producer never blocks indefinitely |
| Extended/RTR/malformed classic CAN input | `transport-socketcan` | discard with a saturating typed counter; never coerce into a device frame |
| BUS-OFF or partial/fatal CAN transmit | transport + bus owner | fault link, report every affected driver, inhibit normal TX, require explicit disarmed recovery |
| Missing/stale Damiao feedback | `motor-damiao` | bounded safe-stop disable path or typed fault; never synthesize numeric zero |
| Unknown/fault motor status or mismatched reply | `motor-damiao` | `Faulted`; recovery must repeat read-only configuration and explicit enable |

### 5. Good/Base/Bad Cases

- Good: `mecanum_4` with exactly four compiled roles, unique IDs and a valid UART IMU; normalization is stable and produces a recorded SHA-256.
- Base: arm morphology `none` with `{}` motor IDs.
- Bad: placing CAN bitrate in robot YAML, mapping two chassis roles to one ID,
  accepting `UNSPECIFIED` quality, sending an expired command, letting
  `application_module` code
  mmap IPC directly or treating a destructor-free Rust type as shared-memory POD.

### 6. Tests Required

- Config unit tests assert every chassis profile, deterministic parse-normalize-parse/hash, unknown fields/version, role mismatch, arm identifier/`none`, duplicate/range IDs, transport/baud and size bound.
- CLI integration tests assert exit status and byte-stable output.
- Codec tests assert header/lease/command round trips, malformed bytes, missing submessages, identity length, unknown enums and expiry.
- Architecture fixtures assert application_module-to-hardware_module, hardware_module-to-protocol_module, protocol_module-to-application_module and production-to-test-support rejection; the real graph must also pass.
- `tools/bufw lint` and `breaking` plus `proto-compat-check` are mandatory for schema changes.
- `protocol_module` local IPC tests assert incompatible-layout rejection, torn reads,
  endpoint exhaustion, slow consumers, sequence/drop accounting and `SIGKILL`
  lease reclamation; target benchmarks compare it with the pinned Horus commit.
- SocketCAN tests use a private fake backend for exact SFF filters, bounded RX,
  one-shot accepted-prefix TX, raw combined error bits, unavailable kernel-drop
  telemetry, BUS-OFF inhibition and checked classic-CAN budgeting. The optional
  ignored `vcan` test opens only a caller-provided nonphysical interface.
- Damiao tests assert manufacturer-derived golden vectors, every raw status,
  exhaustive 16/12-bit decode domains, exact register correlation, read-only
  configuration, bus collisions, feedback freshness and fail-closed lifecycle
  transitions without opening a CAN interface.

### 7. Wrong vs Correct

#### Wrong

```rust
// Generated DTO leaks into application_module code and may contain quality = 0/999.
fn control(input: proto_generated::robot::v1::Header) { /* ... */ }
```

#### Correct

```rust
// The protocol_module boundary owns fallible conversion; application_module code receives a valid semantic type.
let header = contract_codec::decode_header(bytes)?;
controller.step(TimedInput { header, value })?;
```

## Scenario: Add or change local/Zenoh middleware delivery

### 1. Scope / Trigger

Use this contract when adding a middleware key, endpoint, local channel,
network command intent, Zenoh route, ingress guard, query or liveliness path.
This is an infra/cross-layer boundary: RT-facing code must remain independent
of generated DTOs, a channel implementation and the Zenoh runtime.

### 2. Signatures

```rust
ConcreteKey::new(robot_id, domain, resource, channel) -> ConcreteKey
str::parse::<ConcreteKey>() -> Result<ConcreteKey, KeyError>
EndpointDescriptor::new(fields: EndpointFields)
    -> Result<EndpointDescriptor, EndpointError>
latest_channel<T: Copy + Send + 'static>()
    -> (LocalLatestPublisher<T>, LocalLatestSubscriber<T>)
ordered_channel<T: Send + 'static>(capacity: usize)
    -> Result<(LocalEventPublisher<T>, LocalEventSubscriber<T>), ChannelError>
ShmNamespace::create_latest<R: ShmPayload>(name, config) -> Result<(), ShmError>
ShmNamespace::create_ordered<R: ShmPayload>(name, config) -> Result<(), ShmError>
ShmNamespace::attach_{latest,event}_{publisher,subscriber}<T, C: ShmCodec<T>>(...)
    -> Result<nonclonable typed endpoint, ShmError>
ZenohBridge::spawn(config: ZenohBridgeConfig, routes: BridgeRoutes)
    -> Result<ZenohBridge, BridgeError>
ZenohBridge::query(endpoint: EndpointId, payload: Vec<u8>)
    -> Result<Vec<u8>, BridgeError>
SourceGuard::accept(header, value, received_mono_ns, policy, evidence)
    -> Result<ReceivedSample<T>, IngressError>
CommandIntent::materialize(local_lease, received_mono_ns, route_max_ttl_ns)
    -> Result<CommandEnvelope, SemanticError>
```

### 3. Contracts

- Concrete keys are exactly
  `robot/<robot_id>/v1/<domain>/<resource>/<channel>`. Concrete APIs reject
  wildcards; only named `KeySelector` constructors create selectors.
- Every endpoint declares direction, interaction class, exact Protobuf type,
  semantic QoS, nonzero payload/queue bounds, freshness or timeout where
  applicable, source ownership and fallback behavior.
- Latest-value and ordered-event ports are nonclonable SPSC capabilities.
  Steady-state producer calls never block: latest replaces an unread value;
  ordered full/disconnected errors return ownership of `T`. Dependency types
  remain private to `middleware-local`.
- Linux cross-process ports use layout-v1 `middleware-shm` behind those same
  traits. A shared representation must explicitly implement
  `ShmPayload: bytemuck::Pod` and carry a nonzero stable 16-byte schema ID plus
  schema version; matching size, Rust type name or `needs_drop == false` is not
  eligibility. A bounded `ShmCodec` keeps semantic `T` separate and may reject
  a complete representation without exposing it to application code.
- SHM creation/attach/cleanup are startup/admin operations in an absolute,
  exact-mode-0700 private namespace. Creation is `create_new`; mapped files are
  never resized. Every mapping holds a shared region lock, while init/removal
  require its exclusive lock. Publisher/subscriber authority is a distinct
  nonblocking lifetime `flock`; PID/epoch/active fields are diagnostics only.
  Cleanup is explicit and endpoint destructors never unlink.
- Layout v1 is Linux, little-endian SPSC with lock-free 8/64-bit atomics. It
  validates ready magic, layout/endian/atomic/kind/capacity, payload schema/
  version/size/alignment, offsets/stride and exact file length before exposing
  an endpoint. Producer and consumer header lines are cache separated; payload
  bytes are atomic and slots use position-tagged odd/even versions. Latest
  reads retry at most 16 times; ordered full and both disconnected producer
  outcomes return immediately. Hot methods allocate zero bytes and perform no
  file operation, sleep, mapping change or lock acquisition.
- Cross-host commands use `CommandIntent` with asserted resource/owner/lease/
  session/command IDs and a nonzero requested TTL. The receiver matches an
  existing local lease, uses local reception time, and caps TTL by both the
  route maximum and local lease expiration. Sender monotonic/Unix/ROS time
  never creates or extends authority.
- `SourceGuard` requires an expected source and explicitly authorized boot,
  rejects duplicates/reorder/retired boots, records sequence gaps, and keeps
  anti-replay state across disconnect. Liveliness changes availability only;
  it does not authorize a boot or grant a lease.
- `ZenohBridge` owns the only session, callbacks, encoding allocation, waits,
  retries and shutdown join on a named non-RT worker. Callback inboxes and
  commands are bounded and use nonblocking sends. QoS maps only to stable
  priority plus `CongestionControl::Drop`.
- Zenoh is exact-pinned with default features disabled and only
  `transport_tcp`; compression, unstable and shared-memory features stay off.
  The exact dependency-policy exceptions and their removal condition live in
  `deny.toml` and the task research record.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Wrong key root/version/segment count or wildcard concrete segment | `KeyError`; never pass to Zenoh |
| Zero payload/queue/age/timeout bound or incompatible direction/QoS | `EndpointError` at startup |
| Duplicate endpoint ID or same key/direction owner | reject route set before session open |
| Stalled latest consumer | publish newest generation and increment overwrite count |
| Full ordered queue | `EventPublishError::Full(T)` with exact value ownership |
| Dropped local consumer / stopped bridge | typed `Disconnected(T)` or `LatestPublish::Disconnected` |
| Live SHM endpoint lock | `EndpointOccupied`; PID metadata never overrides the lock |
| Wrong SHM ready/layout/schema/file length | typed startup rejection; no port is returned |
| SHM writer killed under odd slot stamp | bounded empty/race result; never decode partial bytes |
| SHM consumer killed with ordered backlog | producer remains nonblocking; proven replacement skips and counts backlog |
| Oversized Zenoh payload | reject before decode and increment `oversized` |
| Unexpected source, unavailable route or unapproved/retired boot | typed `IngressError` plus saturating counter |
| Duplicate/reordered sequence | reject; do not refresh local expiry |
| Accepted sequence gap | accept newer value and add exact missing count |
| Expired/untrusted/poor-quality sample | typed rejection; cached value is not refreshed |
| Resource/owner/lease/session mismatch | `SemanticError::AuthorityMismatch`; no executable command |
| Zero TTL or expired local lease | reject; requested TTL above route maximum is capped |
| Zero/multiple/error/late query replies | `NoReply`/`MultipleReplies`/`RemoteError`/`Timeout` |
| Full callback/command queue | drop or typed `CommandQueueFull`; no callback blocking |

### 5. Good/Base/Bad Cases

- Good: an IMU semantic value is encoded by `contract-codec` on the bridge
  thread, published on a checked state route, size-checked, decoded and exposed
  with local reception/expiry metadata.
- Base: a latest producer keeps publishing while the bridge consumer is
  stalled; generations and overwrite counters advance without a Zenoh call.
- Bad: pass a `zenoh::Session` or generated DTO to application/RT code, use an
  unbounded channel, accept a payload boot change as authorization, or execute
  a remote absolute monotonic deadline.

### 6. Tests Required

- Key/endpoint unit tests cover all domains, round trips, wildcard/version/
  bound/direction/QoS/source/duplicate failures.
- Local tests warm up then assert zero steady-state allocations, exact
  overwrite/full/disconnect counters, rejected-value ownership, stalled
  consumers and independent-thread progress.
- SHM tests additionally use real subprocesses and `SIGKILL` for lock
  reclamation, corrupt diagnostic PID metadata, a writer stopped under an odd
  stamp, slow/crashed consumers, exact abandoned-backlog accounting, live/
  same-process endpoint refusal, schema/length mismatch and explicit cleanup.
- Codec tests prove intents contain TTL but no remote executable deadline and
  reject missing/malformed/unknown/zero inputs.
- Fault tests script loss, delay/stale, duplicate, reorder, source/quality,
  boot restart, disconnect/reconnect and prior-lease rejection with exact
  counters.
- A serialized explicit-TCP loopback test covers semantic -> Protobuf -> real
  Zenoh -> semantic, command materialization, query/queryable timeout,
  liveliness put/delete and local disconnection after bridge stop.
- Run warnings-denied Clippy, architecture policy, Buf compatibility,
  cross-language generation, dependency policy and `./tools/ci.sh`.

### 7. Wrong vs Correct

#### Wrong

```rust
// RT path allocates, waits on Zenoh and trusts the sender's clock/authority.
session.put(remote_key, encode(remote_command)).wait()?;
execute(CommandEnvelope::new(remote_fields)?);
```

#### Correct

```rust
// RT path only publishes a typed local value; the non-RT bridge owns Zenoh.
local_command_port.try_publish(intent)?;
let intent = contract_codec::decode_chassis_setpoint_intent(bytes)?;
let command = intent.materialize(active_local_lease, received_mono_ns, route_max_ttl_ns)?;
validated_command_port.try_publish(command)?;
```

## Scenario: Add a native Protobuf-to-ROS mapping contract

### 1. Scope / Trigger

Use this contract when adding a ROS projection, changing a bridged semantic
message, adding a route, or changing QoS, clock, frame, action, service or DDS
island policy. The mapping is an `integration` concern; ROS IDL/CDR and
bridge-private Zenoh keys never become a second `protocol_module` wire format.

### 2. Signatures

The backend-neutral entry points are:

```rust
RouteRegistry::load(profile_root) -> Result<RouteRegistry, RegistryError>
contract_codec::decode_imu_state(bytes) -> Result<ImuState, CodecError>
project_imu(value, route, aliases, reception_ns) -> Result<RosImuProjection, ProjectionError>
project_twist_ingress(input, route, aliases, context) -> Result<ChassisSetpoint, ProjectionError>
CommandEnvelope::from_bridge_ingress(header, lease, command_id,
                                     received_mono_ns, ttl_ns, max_ttl_ns)
    -> Result<CommandEnvelope, SemanticError>
```

The checked-in profile lives at
`config/protocol/ros2-humble/routes.toml`; it is not part of
`config/robots/*.yaml`.

### 3. Contracts

- `robot.v1` Protobuf remains canonical. Wire DTOs are decoded only by
  `contract-codec` into validated semantic values before a ROS projection.
- A route declares exact native key/direction/class/type, generated descriptor
  SHA-256, pre-decode byte limit, rate/freshness/queue bounds, ROS name/type and
  interface-source SHA-256, explicit QoS, clock/frame policy, authorization and
  loop-suppression origin.
- The runtime descriptor identity is SHA-256 over
  `proto_generated::FILE_DESCRIPTOR_SET`. The Buf-produced
  `proto/golden/robot-v1.binpb` is the breaking-change baseline; the two
  descriptor encodings are not interchangeable fingerprints.
- Standard ROS messages are explicitly lossy and name a companion protocol
  metadata endpoint. IMU absent measurements map to covariance `[0] == -1`;
  present/unknown covariance maps to all zeros; known covariance is validated
  for length, finiteness, symmetry and nonnegative diagonal.
- Canonical frame IDs are slash-free. ROS namespaces are introduced only by a
  checked alias table. TF has one configured authority per child and a cycle-
  free static/dynamic graph.
- A ROS command needs an existing local lease. ROS time checks observation age
  only; local monotonic reception constructs a deadline capped by both route
  TTL and lease expiration.
- Actions register exact goal/feedback/result Protobuf type names, preserve the
  16-byte command ID as the ROS UUID, distinguish cancel acceptance from
  terminal cancellation, and declare bounded feedback/result retention.
- ROS, DDS, Zenoh and ros2dds runtime dependencies stay out of
  `protocol_module`; `ros2-mapping` remains an `integration` crate.

### 4. Validation & Error Matrix

| Condition | Owner | Required result |
|---|---|---|
| Descriptor or interface-source SHA mismatch | `RouteRegistry` | reject profile startup |
| Missing/unknown Protobuf type | `RouteRegistry` descriptor catalog | reject route |
| Duplicate route/key/ROS endpoint | `RouteRegistry` | reject duplicate ownership |
| Zero byte/rate/queue/freshness bound | `RouteRegistry` | reject unbounded route |
| Native and ros2dds prefix overlap | deployment validator | reject profile |
| Same host uses direct DDS and ros2dds | deployment validator | reject duplicate path |
| TF self edge, duplicate child or cycle | TF authority validator | reject profile |
| NaN/infinity, invalid quaternion/covariance | semantic constructor / `contract-codec` | typed error before projection |
| Missing trusted acquisition time on required route | projection | drop with `UntrustedAcquisitionTime` |
| Reception-time fallback route lacks time | projection | reject; never relabel silently |
| Stale/future ROS command stamp | ingress projection | reject observation |
| Missing/expired lease or zero TTL bound | `CommandEnvelope::from_bridge_ingress` | reject; cap nonzero requested TTL to route maximum |
| Unknown action/cancel/status enum | `contract-codec` | `CodecError::UnknownEnum` |

### 5. Good / Base / Bad Cases

- Good: a Humble IMU route matches the generated descriptor and checked-in
  `sensor_msgs/msg/Imu` source, uses bounded best-effort/volatile QoS, maps
  `imu_link` through an explicit alias and preserves acquisition time.
- Base: a static transform has reliable/transient-local QoS, a single child
  authority and an explicit reception-time-degraded fallback.
- Bad: infer ROS names from Zenoh keys, put route/QoS fields in robot YAML,
  accept `KEEP_ALL`, copy source monotonic time into a ROS stamp, mint a lease
  from a ROS publisher, load ros2dds into the native router, or emulate
  `rmw_zenoh` private keys.

### 6. Tests Required

- Run `tools/bufw breaking --against proto/golden/robot-v1.binpb` before
  replacing the golden baseline; then run Buf format/lint and C++/Python
  generation.
- Add semantic/codec good and bad vectors for each routed type: missing fields,
  unknown enums, non-finite values, bounds, covariance/quaternion/frame rules
  and both conversion directions.
- Load the checked-in profile against `proto_generated::FILE_DESCRIPTOR_SET`,
  assert every source/hash/type and pin the deterministic manifest fingerprint.
- Mutate valid fixtures to assert duplicate ownership, zero bounds, QoS/class
  mismatch, path traversal, prefix/DDS collision and TF cycle rejection.
- Assert IMU unavailable/unknown sentinels, required acquisition/drop behavior,
  frame aliases, UUID correlation, cancel lifecycle/result retention and local
  lease/deadline capping.
- Run `tools/ci.sh`; target-board/network/ROS runtime qualification is additive
  and belongs to the later adapter task.

### 7. Wrong vs Correct

#### Wrong

```rust
// Remote ROS time and publisher identity manufacture local authority.
let deadline = twist.header.stamp_ns + ttl_ns;
let lease = ResourceLease::from_ros_publisher(&publisher_gid);
publish_proto(wire::ChassisSetpoint { deadline, lease, .. });
```

#### Correct

```rust
// ROS time establishes observation age; local authority/time enforce safety.
let input = project_twist_ingress(&twist, route, aliases, TwistIngressContext {
    adapter_header,
    lease: existing_local_lease,
    command_id,
    current_ros_time_ns,
    received_monotonic_ns,
    requested_ttl_ns,
})?;
native_command_port.try_send(input)?;
```
