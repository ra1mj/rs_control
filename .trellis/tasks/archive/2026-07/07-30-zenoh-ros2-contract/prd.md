# Zenoh and ROS 2 Protocol Contract

## Goal

Make the native `protocol_module` Zenoh/Protobuf contract safely and
predictably interoperable with ROS 2 without turning ROS 2, DDS or a bridge
wire format into a second core protocol.

The result must implement the backend-neutral semantic messages, route
metadata, conversion rules, QoS, clock/frame behavior, command authority and
compatibility gates required before an optional ROS 2 adapter is implemented.

## Background

Native Zenoh samples are key expressions plus application bytes. ROS 2 topics,
services and actions additionally require generated ROS interface types, CDR
serialization, graph metadata, type hashes, QoS and request/action lifecycle
semantics. Neither `zenoh-bridge-ros2dds` nor `rmw_zenoh` automatically converts
the project's Protobuf payloads into ROS messages.

The first compatibility profile is the repository's existing RK3588 target:
Ubuntu 22.04 arm64, ROS 2 Humble and `rmw_cyclonedds_cpp`. A later ROS
distribution is a separate, pinned profile rather than an implicit upgrade.

This task depends on:

- `07-29-module-layer-naming`, so new paths use `hardware_module`,
  `protocol_module` and `application_module`;
- the parent framework's core Protobuf/message-model work in Phase D;
- the platform image decision to keep Ubuntu 22.04 as the first RK3588
  qualification target.

The protocol amendments are a prerequisite for the ROS adapter. Deploying
`zenoh-bridge-ros2dds` is optional and only needed when an existing DDS ROS
graph must cross a Zenoh network.

## Delivery Boundary

This task lands the executable contract layer only:

- additive `robot.v1` Protobuf amendments and validated Rust semantic/codec
  types required by the first mapping profile;
- a versioned, checked-in Humble route manifest and interface-source catalog;
- a backend-neutral integration crate that parses, fingerprints and validates
  routes, type identities, QoS, time/frame policy, bounds and deployment
  invariants;
- pure host tests for mappings, malformed inputs, action lifecycle, command
  authority and loop/collision prevention.

It does not link ROS 2, DDS or Zenoh libraries, start middleware processes,
generate ROS language bindings, or claim target-board/runtime qualification.
Requirements below that describe adapter behavior are contracts enforced by
types, manifest validation and test vectors in this task; their live runtime
verification belongs to the later adapter/integration task.

## Requirements

### 1. Ownership and topology

- **ZR-1:** Zenoh plus Protobuf remains the canonical inter-node protocol.
  ROS IDL/CDR, DDS discovery and bridge-private Zenoh keys are integration
  details and never become `protocol_module` contracts.
- **ZR-2:** Implement semantic conversion in an independent `ros2-adapter`
  process with one native Zenoh/Protobuf side and one ROS 2 client-library side.
  It must decode into validated domain values before constructing generated ROS
  messages; raw byte forwarding is forbidden.
- **ZR-3:** Keep the adapter outside the realtime process and outside
  `protocol_module`. It has no hardware handles, cannot run an executor on the
  1 kHz thread and cannot extend a lease or deadline.
- **ZR-4:** On one DDS island the adapter publishes directly through
  `rmw_cyclonedds_cpp`. Add standalone `zenoh-bridge-ros2dds` instances only for
  cross-island DDS/ROS graph transport. Do not load its dynamic plugin into the
  core Zenoh router.
- **ZR-5:** Do not emulate `rmw_zenoh` private keys, CDR attachments, graph
  liveliness tokens or GIDs from native applications. Do not combine
  `rmw_zenoh` and `zenoh-bridge-ros2dds` in one interoperability path because
  their wire key spaces are explicitly incompatible.
- **ZR-6:** The adapter and all optional bridges are crash-isolated. Their
  absence, restart or network failure must not affect local control safety.

### 2. Route and type registry

- **ZR-7:** Define a versioned, machine-validated route manifest outside the
  minimal robot topology YAML. Each allowlisted binding declares:
  native key, direction, interaction class, fully-qualified Protobuf type,
  Protobuf descriptor fingerprint, maximum encoded size, maximum rate,
  freshness/timeout policy, ROS name, ROS interface type, ROS distribution,
  exact interface package/source digest, optional native ROS type hash when the
  distribution provides one, QoS profile, clock/frame policy and future
  authorization class.
- **ZR-8:** Set native Zenoh sample encoding to `application/protobuf` and
  validate the manifest's type identity before decoding. Encoding metadata
  alone is not sufficient type negotiation.
- **ZR-9:** Expose a queryable route descriptor and manifest fingerprint for
  diagnostics. Fail startup on duplicate ownership, incompatible type/QoS,
  ambiguous direction or unsupported policy; never discover and expose an
  arbitrary topic automatically.
- **ZR-10:** Keep native and ros2dds bridge key prefixes disjoint. ROS names are
  explicit manifest values, not string substitutions of native Zenoh keys.
- **ZR-11:** Reject payloads above the route's byte limit before Protobuf
  allocation/decoding. Repeated fields and opaque payloads have semantic bounds.
- **ZR-12:** Routes are one-way by default. A bidirectional binding requires
  separate semantic endpoints plus origin/bridge identity and loop-suppression
  tests.

### 3. Canonical message semantics

- **ZR-13:** Keep current Protobuf v1 package identity as `robot.v1`. Additive
  changes use unused field numbers; removed values are reserved. A package
  rename or changed meaning requires a new protocol version.
- **ZR-14:** Add a source/acquisition timestamp with an explicit clock domain
  that can represent trusted Unix time or ROS time, distinct from
  `source_monotonic_ns` and `publish_unix_ns`. Preserve source monotonic time
  for local freshness and hardware deadlines.
- **ZR-15:** Add validated semantic and codec coverage for every bridged device,
  control, perception, health and FSM type before adding its route. Wire DTOs
  are never published to ROS without fallible conversion into a semantic type.
- **ZR-16:** Reject non-finite numeric values; require unit quaternions within a
  documented tolerance; validate covariance length, finiteness, symmetry and
  diagonal policy; require a nonempty frame for every frame-bearing sample.
- **ZR-17:** Represent unavailable IMU measurements independently. Define the
  exact mapping between canonical presence/quality and ROS
  `sensor_msgs/Imu` covariance conventions, including the all-zero unknown
  covariance and first-element-minus-one absent-measurement sentinels.
- **ZR-18:** Add an odometry semantic contract containing parent frame, child
  frame, pose, twist and both covariances. Define its planar-to-ROS 6x6
  covariance projection without inventing measurements on unused axes.
- **ZR-19:** Add stamped transform contracts for dynamic and static transforms,
  with parent and child frames. Pose observations are measurements and never
  implicitly gain TF authority.
- **ZR-20:** Give opaque action parameters and results registered
  fully-qualified Protobuf type identities. Do not allow a network sender to
  choose an unregistered decoder.
- **ZR-21:** Complete the canonical action lifecycle with goal
  accept/reject, executing, cancel request/accept/reject, canceling, feedback,
  terminal result payload and bounded result caching. Keep the 16-byte
  canonical command ID exactly correlated with the ROS goal UUID.
- **ZR-22:** Define lease request/grant/renew/revoke/release behavior at the
  network boundary or make the adapter an explicit client of the existing
  local authority service. A ROS publisher never manufactures an authority
  lease.

### 4. Time and freshness

- **ZR-23:** Never put `source_monotonic_ns` into
  `std_msgs/Header.stamp`. Map a source acquisition timestamp only when its
  clock relationship and uncertainty satisfy the binding policy.
- **ZR-24:** If no trusted source timestamp exists, a binding either drops the
  sample or uses a manifest-declared bridge reception-time fallback and marks
  the projection degraded. It never silently labels reception time as
  acquisition time.
- **ZR-25:** Treat ROS time as discontinuous: it may be uninitialized, pause or
  jump backward under `/clock`. ROS timestamps are observation metadata, never
  the enforcement clock for a motor deadline or lease.
- **ZR-26:** On ROS command ingress, validate the ROS timestamp/age, capture
  local monotonic reception time, obtain valid local authority and construct a
  bounded local deadline/TTL. Never compare a remote ROS/System timestamp
  directly with a local monotonic deadline.
- **ZR-27:** Maintain a bounded ingress tracker per route/source/boot to reject
  stale, prior-boot, duplicate and out-of-order commands where the interaction
  class requires ordering.

### 5. Units and coordinate frames

- **ZR-28:** Follow REP-103: SI units, right-handed frames, body axes x forward,
  y left, z up, radians and quaternion rotations. Camera optical frames use the
  `_optical` convention z forward, x right, y down.
- **ZR-29:** Follow REP-105 for a mobile base: `map -> odom -> base_link`;
  `odom` is locally continuous and may drift, while `map` is globally stable
  but may jump.
- **ZR-30:** Define one authority per TF child frame, reject self edges/cycles,
  and separate static from dynamic authority. The bridge may only publish
  transforms explicitly assigned to it.
- **ZR-31:** Canonical frame IDs remain portable slash-free IDs. A static,
  validated alias table applies ROS namespace/multi-robot prefixes only at the
  bridge boundary; leading slashes, double slashes and implicit stripping are
  rejected.

### 6. ROS interface projections

- **ZR-32:** Prefer standard ROS interfaces for ecosystem compatibility and
  document that they are lossy projections when they cannot carry canonical
  boot, sequence, validity, clock, quality or lease metadata.
- **ZR-33:** Publish companion custom metadata/status interfaces where external
  diagnostics need canonical fields. Safety decisions always use the canonical
  Protobuf envelope, never the lossy standard projection.
- **ZR-34:** The first mapping set contains:
  `ImuState -> sensor_msgs/Imu`, time-aligned motor aggregation to
  `sensor_msgs/JointState`, canonical odometry to `nav_msgs/Odometry`,
  canonical transforms to `/tf` and `/tf_static`, a guarded
  `geometry_msgs/TwistStamped -> ChassisSetpoint` ingress, one bounded
  query/service and one concrete custom action.
- **ZR-35:** Unstamped `geometry_msgs/Twist` ingress is disabled by default. If
  explicitly enabled for a legacy client, it uses reception time, a short
  configured TTL, a current bridge-owned lease and degraded provenance.
- **ZR-36:** Services are only for short, bounded and idempotent
  request/response operations. Motion lasting long enough to need feedback or
  cancellation uses an action.
- **ZR-37:** A ROS action maps the complete three-service/two-topic lifecycle.
  Cancel acceptance is not a terminal canceled result, and results are retained
  for a bounded configured timeout.

### 7. QoS and bounded execution

- **ZR-38:** Every route declares QoS explicitly; no route inherits an
  implementation default. The manifest covers reliability, durability,
  history/depth and the requested deadline, lifespan and liveliness behavior.
- **ZR-39:** Sensor/state streams use a bounded sensor-data profile: keep-last,
  best-effort and volatile unless a specific consumer contract proves
  otherwise. Native delivery remains latest/drop rather than blocking.
- **ZR-40:** Command ingress uses bounded keep-last storage, reliable/volatile
  ROS delivery, a short application validity window and local lease/deadline
  enforcement. Reliable middleware retries never block a realtime thread.
- **ZR-41:** Services use reliable and volatile QoS with bounded timeout and
  request idempotency. Transient-local service requests are forbidden because a
  restarted server could receive stale side effects.
- **ZR-42:** Actions use the target ROS distribution's generated action QoS and
  explicitly test goal, cancel, result, status and feedback compatibility.
  Feedback storage is bounded; status remains available to late joiners as
  required by the ROS action contract.
- **ZR-43:** Use the target `tf2_ros` dynamic/static QoS classes rather than
  re-creating guessed defaults. Static transforms are transient-local; dynamic
  transforms are volatile.
- **ZR-44:** A requested policy not implemented by the selected RMW/bridge is a
  startup error or explicit degraded warning according to the route. It is
  never silently reported as enforced.
- **ZR-45:** Adapter queues are statically bounded/configuration-bounded and
  expose drops, duplicate/reorder, decode failure, stale rejection, conversion
  failure, ROS QoS incompatibility and reconnect counters.

### 8. Deployment, compatibility and evidence

- **ZR-46:** Pin the ROS distribution, RMW, generated interface package,
  Zenoh libraries and any ros2dds executable as one tested compatibility
  matrix. Do not mix Humble with Iron-or-newer `rmw_zenoh` endpoints because
  their type-hash keys are incompatible.
- **ZR-47:** If ros2dds bridges two hosts, prevent a parallel direct DDS path
  between those hosts and enforce one bridge path per DDS island to avoid
  duplicate samples and routing loops.
- **ZR-48:** Preserve ROS-owned GIDs and request correlation inside the ROS
  stack. Canonical source IDs are semantic provenance and never substitute for
  ROS endpoint GIDs.
- **ZR-49:** Store the Protobuf descriptor fingerprint, route-manifest
  fingerprint, ROS interface type/hash and bridge build identity with recordings
  and diagnostics.
- **ZR-50:** Test golden semantic vectors in Rust and the adapter language, both
  conversion directions, intentional lossy projections, unknown enums,
  malformed/oversized payloads, frame/time edge cases, QoS compatibility,
  service retries, action cancellation/result caching, bridge restart,
  duplicates/reordering and loop prevention.
- **ZR-51:** The initial Humble profile may use a mature non-Rust ROS adapter
  implementation at the integration boundary. `hardware_module` and
  `protocol_module` remain Rust-only and keep their dependency boundary.
- **ZR-52:** This task does not add ROS fields, routes or QoS settings to the
  minimal robot topology YAML.

## Acceptance Criteria

- [x] The repository has a reviewed protocol/ROS design that distinguishes
      semantic conversion from DDS-over-Zenoh transport.
- [x] The first compatibility profile is pinned to RK3588 Ubuntu 22.04 arm64,
      ROS 2 Humble and `rmw_cyclonedds_cpp`; optional ros2dds is independently
      pinned and its key/path ownership is validated. Live runtime
      qualification remains in the adapter task.
- [x] The route manifest schema rejects duplicate routes, unspecified
      directions, unbounded payloads/queues and incompatible type/QoS entries.
- [x] Every declared route has a Protobuf descriptor fingerprint, ROS
      interface identity and exact checked-in interface-source digest;
      distributions that expose a native ROS type hash can record and verify it
      without weakening the other identity checks.
- [x] Header semantics distinguish source monotonic, source acquisition and
      publication/correlation time, with tests for unsynchronized clocks and
      ROS time jumps.
- [x] IMU, odometry, transform and frame alias mappings satisfy the documented
      REP-103/REP-105 and ROS interface rules.
- [x] All bridged numeric messages reject NaN/infinity, malformed covariance
      lengths and invalid quaternions before publication.
- [x] A standard ROS projection is explicitly marked lossy wherever canonical
      metadata cannot round-trip, with a companion metadata/status path where
      required.
- [x] ROS command ingress cannot create a setpoint without a current local lease
      and a locally constructed bounded monotonic deadline.
- [x] The action mapping proves UUID correlation, accept/reject, feedback,
      cancel acceptance, cancel completion, terminal result and result expiry.
- [x] Manifest tests cover sensor, command, service, action, `/tf`,
      `/tf_static` and late-joiner policy without unbounded `KEEP_ALL` storage.
- [x] The contract and deployment validator make adapter/ros2dds ownership
      optional and crash-isolated; disabling either cannot create or extend a
      local command lease/deadline.
- [x] Direct DDS plus ros2dds duplicate paths and native/ROS key collisions are
      rejected by deployment validation.
- [x] Protobuf compatibility, checked-in ROS interface source identity and Rust
      golden semantic/manifest tests pass in CI.
- [x] No ROS/DDS/Zenoh executor, allocator or blocking queue operation appears
      on the qualified 1 kHz realtime path.

## Out of Scope

- Replacing the native Protobuf protocol with ROS IDL/CDR.
- Direct native emulation of `rmw_zenoh` internals.
- Auto-bridging arbitrary ROS graph entities.
- Network authentication/encryption in the first trusted-LAN deployment;
  route authorization metadata remains reserved for later enforcement.
- Running ROS 2 controllers inside `hardware_module` or exposing driver
  handles to ROS.
- Claiming cross-distribution compatibility without a separately pinned and
  tested profile.
- Linking or launching a ROS 2, DDS, Zenoh or ros2dds runtime in this task.
- Building generated ROS language bindings or running target-board/network
  failure qualification; those are acceptance gates for the runtime adapter.
