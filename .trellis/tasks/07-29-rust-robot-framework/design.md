# Rust Robot Framework — Technical Design

## 1. Design objective

Build a Rust-owned robot foundation that can drive a real DM-J4310-2EC motor on an Orange Pi 5 Plus, then grow into a multi-process robot runtime without changing the layer boundaries or public contracts.

The core invariant is compile-time dependency direction:

```text
upper applications / algorithms / FSM
                 |
                 v
middle typed API / in-process fast path / Zenoh / Protobuf
                 |
                 v
lower transport / device protocol / device lifecycle
```

No lower crate imports a middle or upper crate. No middle crate imports an upper crate. Upper crates do not import lower crates. Executable composition roots may construct objects from all three layers, but contain wiring only and no business logic.

Runtime process boundaries are deliberately different from compile-time layer boundaries. A process may vertically compose all three layers while the source dependency graph remains one-way.

## 2. Fixed decisions

| Area | Decision |
|---|---|
| Primary board | Orange Pi 5 Plus / RK3588, active cooling, external isolated 3.3 V CAN transceiver |
| OS | Ubuntu 22.04 Server |
| Kernel | Orange Pi/Rockchip 5.10 BSP with a pinned matching PREEMPT_RT patch |
| First actuator | DM-J4310-2EC, classic CAN 1 Mbps, MIT mode |
| Lower and middle language | Rust only; unavoidable kernel/vendor FFI is wrapped and reviewed |
| Middleware | Zenoh between processes/nodes; Protocol Buffers as the external message contract |
| Realtime local path | Middle-owned, preallocated latest-value mailboxes/bounded queues; no Protobuf encoding inside the 1 kHz hot path |
| Network realtime | Ethernet inputs may affect control, but are soft realtime in v1 and must pass freshness/lease checks |
| Runtime isolation | `motion-rt`, `perception`, `robot-fsm`, `telemetry-recorder`, optional `ros2-bridge`, and `zenohd` |
| Control period | Prefer 1 kHz; explicitly qualified 500 Hz fallback; no silent runtime rate change |
| Safety | Hardware E-stop/enable, motor communication timeout, software freshness/lease enforcement |
| FSM | Compiled Rust hierarchical state machine; no v1 YAML/SCXML runtime |
| Mission composition | Typed Rust DSL task tree/graph owned by the FSM `Running` state |
| Recording | Structured diagnostics plus MCAP; continuous control/state, triggered rolling raw camera/point-cloud buffers |
| ROS 2 | Optional standalone bridge; no ROS 2 dependency in the core |
| Network security | Trusted-LAN v1; mTLS/ACL are deferred, but contracts do not prevent later addition |
| Simulation | Deferred; v1 validates against real DM-J4310-2EC hardware |

## 3. Workspace and crate boundaries

Recommended repository shape:

```text
Cargo.toml                         # workspace, resolver, shared dependency pins
rust-toolchain.toml                # pinned stable toolchain
deny.toml                          # license/advisory/source policy

crates/
  lower/
    lower-core/                    # Transport, Device, lifecycle, time, lower errors
    transport-socketcan/           # Linux SocketCAN implementation
    transport-serial/              # UART/USB-serial transport
    camera-v4l2/                   # USB/MIPI capture boundary; later milestone
    motor-damiao/                  # DM protocol and static motor profiles
    imu-drivers/                   # concrete IMU protocols; selected later
    lidar-drivers/                 # concrete lidar protocols; selected later

  middle/
    message-model/                 # upper-facing typed messages and validation
    observability-contract/        # externally visible health/fault vocabulary
    proto-generated/               # generated Prost code; never hand-edited
    middleware-api/                # ports used by upper crates
    middleware-local/              # preallocated local snapshots/queues
    middleware-zenoh/              # Zenoh pub/sub/query/liveliness adapter
    contract-codec/                # lower <-> message model <-> Protobuf mapping

  upper/
    control-core/                  # controller lifecycle and generic control traits
    chassis-model/                 # body/wheel types, constraints, morphology registry
    chassis-mecanum/
    chassis-omni/
    chassis-swerve/
    state-estimation-2d/           # wheel + IMU + optional external pose correction
    trajectory-se2/                # timestamped SE(2) tracking
    arm-control/                   # realtime arm coordination; morphology plugins later
    fsm-core/                      # safety/lifecycle HSM
    mission-dsl/                   # typed task nodes and validated IR
    mission-runtime/               # single-thread deterministic executor

  integration/
    config-model/                  # deployment-only YAML AST; no layer imports it
    mcap-recorder/                 # Protobuf channel recorder and rolling buffers
    ros2-mapping/                  # explicit Protobuf <-> ROS 2 mapping tables
    test-support/                  # dev-dependency fixtures; absent from production graph

apps/
  motion-rt/                       # composition root for motion runtime
  perception/                      # composition root for camera + inference boundary
  robot-fsm/                       # HSM + mission runner service
  telemetry-recorder/              # MCAP and external telemetry service
  ros2-bridge/                     # optional bridge wrapper/mapping process
  robotctl/                        # config validation, status, safe manual commands

proto/
  robot/v1/common.proto
  robot/v1/device.proto
  robot/v1/control.proto
  robot/v1/health.proto
  robot/v1/fsm.proto
  robot/v1/perception.proto

config/
  robots/                          # minimal robot topology YAML
  zenoh/                           # deployment-only Zenoh config
  systemd/                         # generated/packaged service units

platform/orangepi5plus/
  kernel/                          # pinned source/patch manifests and config fragments
  device-tree/                     # CAN overlay notes or patches
  scripts/                         # idempotent diagnostic/bring-up helpers
  docs/                            # wiring, cooling, IRQ/CPU map, benchmark manifest
```

### 3.1 Contract ownership

- Lower owns physical units, raw device status, transport failures, device lifecycle, and protocol encoding.
- Middle owns all upper-facing message types, freshness validation, command envelopes, leases, serialization, key expressions, and local/Zenoh delivery semantics.
- Upper owns robot-level algorithms, resource arbitration policy, mission semantics, and FSM transitions.
- Protobuf is generated from `proto/` and must not become the lower driver API.
- MCAP records the already-serialized Protobuf payload and the matching descriptor set; the recorder does not redefine messages.
- ROS 2 mapping consumes the middle contract and never becomes a second source of truth.

### 3.2 Enforced dependency classes

| Crate class | Allowed internal dependencies |
|---|---|
| `lower` | other `lower` crates only |
| `middle` | `middle` and `lower` |
| `upper` | `upper` and `middle`, never `lower` |
| `integration/config-model` | no production layer crate; it is converted by composition roots into layer-owned startup settings |
| other `integration` services | `integration` and `middle`; a documented adapter may consume an upper output only through a middle port |
| `apps` | any required layer for construction only; architecture tests reject business algorithms in these crates |
| `test-support` | development dependencies only and absent from release binaries |

This prevents “support” or monitoring crates from becoming a hidden reverse-dependency channel. Lower crates expose lower-owned health/error values; middle maps those values into the externally visible observability contract. The full robot YAML is parsed only during startup by a composition root, which passes each layer a narrowed immutable configuration rather than the whole document.

## 4. Lower layer

### 4.1 Transport boundary

Conceptual interfaces:

```rust
trait FrameTransport {
    type Frame;
    type Error;

    fn receive(&mut self, out: &mut [Self::Frame]) -> Result<usize, Self::Error>;
    fn transmit(&mut self, frames: &[Self::Frame]) -> Result<usize, Self::Error>;
    fn health(&self) -> TransportHealth;
}

trait DeviceDriver {
    type Command;
    type Sample;
    type Fault;

    fn configure(&mut self) -> Result<(), DeviceError<Self::Fault>>;
    fn enable(&mut self) -> Result<(), DeviceError<Self::Fault>>;
    fn disable(&mut self) -> Result<(), DeviceError<Self::Fault>>;
    fn handle_input(&mut self, now: MonoTime) -> Result<Option<Self::Sample>, DeviceError<Self::Fault>>;
    fn encode_command(&mut self, command: &Self::Command, out: &mut FrameBatch) -> Result<(), DeviceError<Self::Fault>>;
}
```

These are design shapes, not frozen Rust signatures. The implementation must avoid async traits and allocation in device I/O hot paths.

### 4.2 SocketCAN

The SocketCAN adapter owns:

- non-blocking raw CAN sockets;
- kernel receive timestamps where supported;
- CAN error-frame subscription;
- RX overflow, error-passive, bus-off, restart and link-state counters;
- CAN filters installed per configured device set;
- preallocated RX/TX batches;
- a bus-budget calculator using actual classic-CAN frame cost and protocol rates.

The application does not receive raw file descriptors. Bringing the link up/down and changing bitrate is performed by a privileged, idempotent platform helper or systemd unit; `motion-rt` does not need broad `CAP_NET_ADMIN` during normal operation.

### 4.3 DM-J4310-2EC driver

The first profile is a static Rust constant containing the official position, velocity, torque, stiffness, damping and temperature limits. Commands outside the profile are rejected or explicitly saturated according to the field contract; saturation is never implicit.

Driver lifecycle:

```text
Unconfigured -> Disabled -> Enabling -> Enabled -> Faulted
                      ^          |          |
                      |          v          v
                      +------ Disabling <- SafeStop
```

Required operations:

- validate unique ESC_ID and MST_ID before arming;
- encode/decode MIT frames;
- enable, disable, zero-command and explicit fault reset;
- decode position, velocity, torque, MOS temperature, rotor temperature and error state;
- configure/read back the motor CAN timeout where protocol support is confirmed;
- reject feedback from the wrong ID/profile;
- treat missing or implausible feedback as stale, not as zero;
- expose a monotonically increasing sample sequence and source timestamp.

Writing persistent motor registers is a maintenance operation and is never performed automatically during normal startup. The system first reads/verifies critical settings and requires an explicit maintenance command to change them.

### 4.4 Other devices

- IMU transport supports UART and USB-serial behind the same lower interface. A concrete IMU protocol is selected in a later hardware task.
- Camera capture converges on a buffer-handle abstraction. V4L2/DMABUF is preferred when available; remote messages carry metadata or compressed frames, not local raw-frame copies in the realtime path.
- Lidar and extra sensors extend lower drivers without changing middle or upper dependencies.

## 5. Minimal robot YAML

The user-facing robot YAML selects morphology and binds logical roles to IDs. It is not a protocol DSL and does not contain controller tuning, frame layouts, arbitrary expressions, Zenoh configuration, or systemd settings.

Illustrative schema:

```yaml
schema_version: 1

chassis:
  morphology: mecanum_4       # mecanum_4 | omni_3 | omni_4 | swerve_4
  motor_ids:
    front_left: 1
    front_right: 2
    rear_left: 3
    rear_right: 4

arm:
  morphology: none            # later: named compiled morphology profile
  motor_ids: {}

imu:
  transport: uart             # uart | usb
  baud_rate: 921600
```

Rules:

- unknown fields are rejected;
- duplicate/out-of-range motor IDs are rejected;
- required roles are derived from the selected compiled morphology;
- the document has an explicit schema version;
- the parsed configuration becomes immutable before hardware enable;
- changes require disarming and a controlled process restart;
- CAN interface, device path, Zenoh endpoints and deployment settings live outside this robot YAML.

The logical-role binding pattern follows the useful part of `rm-controls/rm_control`, while intentionally omitting its controller and realtime tuning fields from the robot topology document.

## 6. Middle layer

### 6.1 Message model

Every externally visible sample or command carries a common header concept:

| Field | Meaning |
|---|---|
| `source_id` | stable publisher/component identity |
| `boot_id` | changes on each publisher restart |
| `sequence` | monotonic within one boot ID |
| `source_monotonic_ns` | timestamp at the physical/algorithm source |
| `publish_unix_ns` | optional wall-clock correlation timestamp |
| `valid_for_ns` | command/sample validity window |
| `frame_id` | coordinate frame where relevant |
| `clock_status` | local/synchronized/unsynchronized and uncertainty |
| `quality` | valid/degraded/stale/calibrating/faulted |

Consumers use source monotonic time only when the source is local or has a validated clock relationship. Network consumers always calculate age using reception time plus declared clock quality; unknown synchronization never masquerades as precision.

Units are SI and coordinates follow a right-handed robot convention compatible with common ROS usage: `x` forward, `y` left, `z` up, radians for angles.

### 6.2 Protobuf organization and compatibility

- Use packages `robot.v1.<domain>`.
- Use `prost`/`prost-build` for Rust generation and standard generators for C++/Python.
- Commit `.proto` sources and a versioned descriptor-set artifact; do not hand-edit generated Rust.
- Every enum has an `UNSPECIFIED = 0` value.
- Removed fields and enum numbers are reserved and never reused.
- Do not change field wire types, reinterpret existing fields, or make presence assumptions without a version transition.
- `buf lint` and `buf breaking --against <main-or-golden>` run in CI.
- Breaking semantics create a new package version or a parallel key space; deployment supports a bounded rolling compatibility window.

### 6.3 Zenoh key space

Canonical shape:

```text
robot/<robot_id>/v1/<domain>/<resource>/<channel>
```

Examples:

```text
robot/r1/v1/device/motor/front_left/state
robot/r1/v1/device/imu/base/state
robot/r1/v1/control/chassis/setpoint
robot/r1/v1/control/chassis/ack
robot/r1/v1/control/chassis/lease
robot/r1/v1/health/motion-rt/status
robot/r1/v1/fsm/system/state
robot/r1/v1/fsm/mission/event
robot/r1/v1/perception/tracker/target
```

The key carries routing identity; the payload carries typed semantic identity. Wildcards are used by monitoring and recording, not by realtime command publishers.

### 6.4 Interaction classes

| Class | Zenoh primitive | Delivery semantics | Backpressure |
|---|---|---|---|
| Streaming setpoint | pub/sub | latest valid value; loss tolerated within lease | drop old/latest wins |
| High-rate state | pub/sub | sequence-detectable best effort | drop/downsample |
| Discrete action | pub/sub command + explicit ack/event | idempotent, retryable, correlated | bounded retry |
| Snapshot/config/diagnostic | query/queryable | request/response with timeout | outside RT loop |
| Presence | liveliness | component/ability availability | event-driven |
| Bulk image/point cloud | pub/sub or shared memory | lossy/triggered according to mode | drop first |

Streaming setpoints never block an RT producer. Discrete actions do not report success until a matching acknowledgement or terminal event arrives.

### 6.5 In-process fast path

The public upper-facing API is independent of its delivery backend. In one process, middle selects a local backend:

- latest-value double buffer/mailbox for sensor state and setpoints;
- bounded SPSC ring for ordered discrete events;
- preallocated storage and explicit overflow counters;
- one writer per channel or a defined arbitration point;
- acquire/release memory ordering documented and tested;
- no `Mutex`, async executor wakeup, logging format, serialization or allocation in the 1 kHz path.

Selected samples are mirrored by a non-RT bridge thread to Zenoh/Protobuf. Candidate implementations must be benchmarked on the target board; the design does not lock a queue crate before measurement.

The local backend will use the source of `softmata/horus` at a pinned commit as a comparison point, specifically its cache-line-separated head/tail fields, publisher-subscriber SPSC matrix, per-slot versioned seqlock and `flock`-backed crashed-endpoint reclamation. These are patterns to benchmark, not an imported runtime dependency. The project explicitly rejects automatic POD detection based on `needs_drop`: shared-memory payloads require a reviewed fixed-layout `Pod`-equivalent contract or opaque byte/descriptor transport, with layout magic/version/schema identity and an isolated private `unsafe` boundary. Every latest/drop/reject path reports sequence gaps and counters.

### 6.6 Network inputs

Ethernet Zenoh nodes may publish commands, sensor samples and algorithm results. V1 classifies ordinary Ethernet as soft realtime:

- `motion-rt` reads only the latest validated snapshot and never waits for a network sample;
- duplicate, out-of-order, expired, prior-boot and prior-lease messages are rejected;
- each input type declares maximum age, loss tolerance, fallback source and safe action;
- link latency, jitter, loss and clock uncertainty are monitored separately;
- PTP, hardware timestamps and TSN are future capability profiles, already represented by clock/link-quality fields.

## 7. Upper control layer

### 7.1 Realtime scheduling model

`motion-rt` uses OS threads rather than an async executor for its realtime domain. Relative priority is established by measurement, with this intent:

```text
critical hardware IRQ / E-stop observation
          > CAN I/O and safety enforcement
          > 1 kHz motion control
          > local-to-Zenoh bridge
          > normal async/network work
          > telemetry formatting/compression/disk
```

Realtime setup includes:

- `SCHED_FIFO` for qualified RT threads;
- CPU affinity based on measured RK3588 core topology;
- IRQ affinity coordinated with thread placement;
- `mlockall`/prefaulted stacks and bounded memory;
- absolute monotonic wakeups to avoid period drift;
- performance governor and documented thermal policy;
- no blocking I/O, heap allocation, DNS, reconnect, compression or disk logging in RT threads.

Affinity APIs distinguish an exact single Linux CPU, an exact CPU mask and a candidate set. Setting a candidate list to the first core that happens to succeed is not valid mask semantics. Qualified startup reads the effective thread/IRQ affinities back and fails closed when PREEMPT_RT, FIFO priority, memory locking or required placement is unavailable. General IRQs stay off the control-only CPU, while the CAN IRQ remains on its designated I/O CPU; the system never blindly moves every IRQ away from all realtime roles.

The base tick is 1 ms. If the board cannot pass the full stress gate, the process starts only with an explicitly selected and separately qualified 2 ms profile. A running process never silently changes period.

### 7.2 CAN budget

The bus scheduler calculates planned load from bitrate, classic-CAN framing overhead, command grouping, motor count, feedback frequency and maintenance traffic. The default planned maximum is 70%. A configuration over budget is rejected or must select a lower validated rate before arming.

RX remains event-driven. Control calculations and command emission have independent periodic schedules. “1 kHz runtime” does not mean every motor must transmit and report at 1 kHz.

### 7.3 Chassis modules

Unified upper-facing types:

- `BodyTwist2 { vx, vy, yaw_rate }`;
- `BodyPose2 { x, y, yaw }`;
- `WheelState` / `SteeringState`;
- timestamped `Se2Trajectory` with velocity/acceleration constraints;
- `ChassisLimits` and explicit saturation result.

Morphology plugins implement forward/inverse kinematics and feasibility projection:

- four-wheel mecanum;
- three/four-wheel omni;
- four-module swerve with steering optimization and rate limits.

Pipeline:

```text
wheel feedback + IMU + optional external pose correction
        -> time alignment / quality gate
        -> 2D estimator
        -> pose + velocity
        -> trajectory / body velocity / heading controller
        -> morphology feasibility projection
        -> per-motor setpoints
        -> middle command port
```

SLAM, map management and global planning are external upper applications. They may send pose corrections, trajectories or goals through middle contracts.

### 7.4 Arm and vision

- Rust arm realtime coordination can run as isolated threads in `motion-rt` and only uses middle ports.
- Non-Rust planners or vision algorithms run in separate processes and use Zenoh/Protobuf.
- Camera capture and inference are isolated in `perception`; inference faults cannot terminate motion control.
- Vision publishes observations/targets, not raw motor-driver commands. A control module with a valid resource lease converts targets into robot commands.

## 8. Resource authority and command safety

There is no last-writer-wins arbitration.

FSM owns policy and grants one active lease per exclusive resource. `motion-rt` owns enforcement. Each command contains:

```text
resource_id, owner_id, lease_id, session_epoch,
sequence, issued_at, deadline, command_id
```

Priority:

```text
hardware E-stop / driver safety
  > local manual takeover
  > FSM fault/degraded handling
  > current automatic mission
  > remote test control
```

Grant or takeover is two-phase: revoke/expire the old authority, confirm the resource safe, then activate the new lease. Lease expiry, FSM loss, epoch change or invalid command causes a predefined safe action. The motor's independent CAN timeout remains the last software-independent communication guard.

## 9. FSM and automatic project orchestration

### 9.1 Lifecycle HSM

Top-level model:

```text
Boot
  -> Standby
  -> Armed
  -> Running
       -> Degraded
       -> Fault
  -> EStop
  -> Shutdown
```

Exact transitions are an explicit table. `EStop` can preempt every state except completed shutdown. Leaving `Running` cancels all mission work before releasing resources. Restart always begins at `Boot`, publishes motion inhibit, reconstructs health from live observations and stops at `Standby` until a new explicit arm command.

`statig` is a candidate because it supports hierarchical event-driven state machines without requiring heap allocation. A spike must prove graph visibility, deterministic event ordering, synchronous cancellation boundaries, resource ceilings and testability. Otherwise, implement the same transition table directly with enums and exhaustive matches.

### 9.2 Mission DSL and IR

The `Running` state owns a mission runtime. The typed Rust DSL constructs a validated immutable IR from nodes:

- `Action<TInput, TOutput>`;
- `Sequence`;
- `Selector`;
- `ParallelAll` / `ParallelAny`;
- `Timeout`;
- `Retry` with bounded policy;
- `Loop` with explicit termination;
- `Guard`;
- `CancelScope`;
- named reusable subflow.

Each node has a stable ID, declared resources, timeout/cancel behavior and a terminal result: `Succeeded`, `Failed(code)`, `TimedOut`, or `Cancelled(reason)`.

Mission validation rejects:

- unknown actions;
- unreachable nodes;
- unbounded retry/loop without an externally cancellable scope;
- parallel branches requesting the same exclusive resource;
- actions without a timeout or cancellation contract;
- paths that can retain a lease after terminal completion.

Leaf actions send a middle-layer command and await a correlated acknowledgement/event. They never call a lower driver. The executor is a single-thread serialized state reducer; external work runs elsewhere and returns events.

Future visual/YAML tooling must compile to this same IR and run the same validator. It does not get a separate interpreter or safety model.

### 9.3 FSM validity

Required verification:

- transition-table coverage for every state/event pair;
- reachability and safe-exit analysis;
- property tests over duplicate, lost, reordered, stale and timeout events;
- deterministic replay from recorded event sequences;
- crash/restart and stale-checkpoint tests;
- fault injection for lost components, action timeouts and E-stop races;
- generated Mermaid/DOT state graph checked into build artifacts;
- no automatic resumption of a prior active mission.

## 10. Observability, faults and recording

### 10.1 Health and fault contract

Every component exposes:

- lifecycle: `Starting`, `Ready`, `Active`, `Degraded`, `Faulted`, `Stopping`;
- heartbeat/liveliness;
- last successful sample/command time;
- counters and gauges relevant to its layer;
- typed faults with `code`, `severity`, `latched`, `recoverability`, `first_seen`, `last_seen`, `count`, `source`, `related_command_id`, and `recommended_action`.

Severity is independent of log level. Safety decisions consume typed faults, not formatted log strings.

### 10.2 Structured diagnostics

Rust crates emit `tracing` spans/events. RT threads write only compact preallocated diagnostic records into bounded buffers. Non-RT workers format/export them.

OpenTelemetry is an optional adapter for external metrics/traces/logs, not the internal API or a startup dependency.

Core metrics include:

- RT wakeup jitter, execution time, deadline misses and queue occupancy;
- CAN utilization, RX overflow, error frames, bus-off and recovery;
- per-device sample age, feedback gaps, command rejection and temperature;
- Zenoh publish/receive age, reconnects, drops and queue pressure;
- lease grants, revocations, rejects and expirations;
- FSM transitions, action latency, retries, timeouts and cancellations;
- recorder drops, disk throughput, quota and rolling-buffer coverage.

### 10.3 MCAP

`telemetry-recorder` subscribes without backpressuring publishers and writes MCAP chunks containing:

- original Protobuf bytes;
- the corresponding descriptor-set schema;
- canonical topic;
- publish/source and record timestamps;
- sequence and gap metadata;
- build, config hash, board, kernel and clock metadata.

Control/state/FSM/health streams record continuously. Raw image/point-cloud data use bounded rolling storage; a fault or manual command freezes configurable pre/post windows. Training mode explicitly enables continuous bulk recording.

On queue or disk pressure, priority order is: preserve faults/FSM/commands, then control state/IMU/odometry, then downsampled perception, then raw bulk data. Gaps are written as explicit metadata.

## 11. Processes and supervision

```text
systemd
  +-- zenohd                 # explicit local/network routing process
  +-- motion-rt              # CAN/IMU + RT chassis/arm control + non-RT Zenoh bridge
  +-- perception             # camera capture + inference boundary
  +-- robot-fsm              # lifecycle HSM + mission runtime + lease policy
  +-- telemetry-recorder     # MCAP + external telemetry adapters
  +-- ros2-bridge (optional) # ROS 2 mapping and standalone ros2dds bridge
  +-- robotctl (on demand)   # operator/maintenance client
```

systemd owns ordering, OS credentials, capabilities, restart rate limits and service watchdogs. FSM consumes component health and orchestrates robot state; it does not spawn or restart general processes.

Recommended privilege shape:

- a small privileged platform unit configures CAN link and IRQs;
- `motion-rt` receives only required realtime/memory-lock/device permissions;
- recorder has write access only to its data directory;
- optional bridges run without motion-device access;
- no production service runs as unrestricted root unless a measured kernel limitation forces a documented exception.

## 12. ROS 2 bridge

The optional bridge maps selected contracts explicitly:

- sensor state topic;
- chassis command topic;
- snapshot/service query;
- long-running action with feedback/cancel;
- coordinate frames, SI units and QoS.

Use a standalone `zenoh-bridge-ros2dds` process rather than dynamically loading it into the core router. Pin Zenoh/Rust/bridge compatibility. ROS 2 failure cannot remove local safety enforcement or extend an expired lease.

## 13. Failure and recovery matrix

| Failure | Immediate local response | FSM/system response | Record |
|---|---|---|---|
| Physical E-stop | hardware removes enable/power | enter latched `EStop` | all available pre/post data |
| Motor command timeout | motor independently stops/disables | resource fault, revoke lease | command/feedback gap |
| CAN bus-off | stop affected resource; no blind restart while armed | `Degraded` or `Fault` | error frames and counters |
| IMU stale | estimator marks invalid; controller uses declared fallback or stops | revoke affected mission capability | age/quality transition |
| Network command stale | reject command; retain prior value only until its lease expires | report source degradation | reject reason |
| Zenoh/router loss | local fast loop continues only with valid local lease/data | network-dependent missions cancel/degrade | disconnect interval |
| Perception crash | motion remains isolated | perception capability degraded; related actions fail/cancel | watchdog and mission event |
| Recorder slow/full | drop by priority; never block producer | alert only unless policy requires stop | explicit data gaps |
| FSM crash | leases expire and motion stops | systemd restarts FSM into `Boot` | restart boundary/boot ID |
| `motion-rt` deadline miss | count and evaluate burst threshold; safe-stop if threshold exceeded | `Fault`/degraded policy | jitter and execution trace |
| Thermal throttling | realtime qualification invalid; reduce non-RT load or stop | fault if deadline risk persists | temperature/frequency |

## 14. Compatibility and evolution

- New device protocols add lower crates and contract mappings; they do not change upper dependencies.
- New chassis morphologies implement a registered upper plugin/profile and declare required YAML roles.
- New Protobuf fields are additive and compatibility-checked; breaking semantics use a new version.
- RDK X5 adds a platform directory and a new performance qualification record, not `cfg` branches scattered through algorithms.
- Network security later adds mTLS/ACL deployment policy and identity provisioning without changing command payload semantics.
- Deterministic Ethernet later supplies a validated clock/link profile without changing consumers' freshness API.
- Simulation later implements the same lower/middle interfaces and uses the same acceptance suites.

## 15. Rollout and rollback

Each milestone produces a separately runnable vertical slice. A new slice is enabled only after its hardware, contract, timing and fault gates pass.

- Keep the motor disabled by default; enable only inside fixture-based tests with explicit operator action.
- Persist kernel, firmware, motor register and config hashes with benchmark results.
- Package application and kernel changes separately so application rollback does not silently change the qualified kernel.
- Schema changes retain a last-known-compatible descriptor set and recorded fixture.
- If 1 kHz qualification fails, select the explicit 500 Hz profile and rerun the full gate; do not tune thresholds to hide misses.
- If a new middleware/recorder/bridge version changes RT measurements, rollback that component and retain the evidence bundle.

## 16. Research references

- PREEMPT_RT theory: https://docs.kernel.org/core-api/real-time/theory.html
- Orange Pi 5 Plus CAN and BSP documentation: https://www.orangepi.org/orangepiwiki/index.php/Orange_Pi_5_Plus
- DM-J4310-2EC product/manual source: https://www.dmbot.cn/index.php?c=show&id=125
- Zenoh deployment: https://zenoh.io/docs/getting-started/deployment/
- Zenoh QoS API: https://docs.rs/zenoh/latest/zenoh/qos/
- Protobuf compatibility checks: https://buf.build/docs/breaking/
- Prost generation: https://docs.rs/prost-build/latest/prost_build/
- V4L2 DMABUF: https://docs.kernel.org/userspace-api/media/v4l/dmabuf.html
- MCAP specification: https://mcap.dev/spec
- Rust HSM candidate: https://github.com/mdeloof/statig
- ROS 2 Zenoh bridge: https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds
- Horus source comparison (affinity, SHM and queues): https://github.com/softmata/horus
