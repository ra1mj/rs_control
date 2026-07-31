# Cross-language algorithm and ROS 2 contract — Technical design

## 1. Outcome and boundaries

This task adds one independent Python example and two host-testable integration
contracts:

```text
Rust local odometry producer
        |
        | bounded local port
        v
middleware-zenoh -- robot/demo/v1/navigation/odometry/state
        |
        | real TCP Zenoh + robot.v1 Protobuf
        v
Python heading-hold process
        |
        | robot.v1.ChassisSetpointIntent
        v
Rust source/boot/sequence guard
        |
        | existing local lease + local monotonic receipt
        v
bounded CommandEnvelope
```

The Python process has no driver access and cannot construct an executable
local deadline. Process death therefore removes a source of future intents; it
does not remove local safety enforcement.

The ROS deliverable remains a pure integration contract. It validates runtime
evidence and failure transitions without linking ROS 2, DDS or ros2dds into the
core workspace.

## 2. Repository shape

```text
examples/python-zenoh-algorithm/
  pyproject.toml
  uv.lock
  README.md
  heading_hold.py

crates/integration/cross-language-contract/
  Cargo.toml
  src/lib.rs
  tests/python_zenoh.rs

crates/integration/ros2-mapping/
  src/lib.rs
  src/runtime.rs
  tests/profile.rs
  tests/runtime_failure.rs
```

Generated Python and C++ Protobuf sources live only in temporary directories.
The integration test uses the same vendored `protoc` as the Rust workspace and
sets `PYTHONPATH` to the generated tree.

## 3. Python algorithm contract

### 3.1 Inputs and configuration

The process accepts explicit arguments for:

- Zenoh TCP endpoint;
- input and output canonical keys;
- expected robot/controller identity;
- asserted resource, owner, lease ID and session epoch;
- requested TTL, proportional heading gain and maximum yaw rate;
- optional deterministic boot ID for qualification only.

Normal startup generates a random 16-byte boot ID. The process declares the
subscriber and publisher before printing a machine-readable readiness line.
It handles Zenoh callbacks on the non-realtime Python side.

### 3.2 Transformation

For each `robot.v1.OdometryState`:

1. reject malformed Protobuf, missing required nested values, non-finite
   position/orientation/twist, invalid quaternion magnitude, wrong covariance
   lengths or nonpositive input validity;
2. recover planar yaw from the unit quaternion;
3. compute
   `yaw_rate_radps = clamp(-heading_gain * yaw_rad, ±maximum_yaw_rate_radps)`;
4. publish a `ChassisSetpointIntent` with zero linear velocity and the bounded
   yaw rate in SI units;
5. create a fresh algorithm header with the process boot ID, increasing
   sequence, Python monotonic time, bounded validity and explicit quality.

The intent contains a requested TTL and asserted authority tuple only. It never
contains a local executable deadline.

### 3.3 Dependency and import boundary

`pyproject.toml` exact-pins `eclipse-zenoh` to the workspace Zenoh compatibility
family and a Protobuf runtime compatible with vendored protoc 31.1. `uv.lock`
records resolved artifacts. A static test parses the Python AST and rejects
imports outside the standard library allowlist, `zenoh`, and generated
`robot.v1.*_pb2` modules.

## 4. Cross-process qualification

The integration crate is classified as `integration`. Its test:

1. generates Python Protobuf modules into a temporary directory;
2. reserves a loopback port and starts the Rust Zenoh router;
3. starts the locked Python process in client mode and waits for readiness;
4. publishes a semantic odometry vector through the bounded local port;
5. receives the Python intent, decodes it through `contract-codec`, applies
   `SourceGuard`, and materializes it against a pre-existing local lease;
6. checks zero linear target, bounded yaw correction and a deadline capped by
   route TTL and lease expiry;
7. force-kills the process while continuing local publications and checks
   nonblocking producer progress;
8. proves the last command is invalid at its exclusive deadline and that no
   new intent arrived;
9. restarts with a different boot ID, explicitly authorizes that boot and
   proves a new intent can be accepted.

All waits have short explicit timeouts. Child processes are killed and reaped
by a drop guard, including assertion-failure paths.

## 5. ROS 2 compatibility profile

The profile gains explicit `zenoh_version` and `ros2dds_mode` fields. Supported
v1 values are:

| Field | Value |
|---|---|
| ROS distribution | Humble |
| RMW | `rmw_cyclonedds_cpp` |
| Rust/Python Zenoh | `1.9.0` |
| ros2dds | `1.9.0` |
| ros2dds deployment | `standalone` |

Validation rejects empty or mismatched versions and plugin deployment. This
does not claim that arbitrary future patch versions are compatible; a new
matrix requires a reviewed profile update.

## 6. Runtime evidence and failure contract

`ros2-mapping::runtime` defines pure, serializable observations rather than a
ROS client:

- adapter lifecycle epoch and alive state;
- profile ID, descriptor fingerprint and manifest fingerprint;
- Zenoh and ros2dds build versions plus standalone mode;
- native Zenoh connectivity;
- ROS graph/DDS availability;
- optional ros2dds availability for the configured remote DDS island.

`RouteRegistry::evaluate_runtime` returns a report for the current epoch.
Identity mismatches are startup errors. A changed epoch invalidates old
evidence. Route admission is computed conservatively:

| Condition | Egress sensor/state | Command/action ingress | Service/query |
|---|---|---|---|
| all required paths ready | available | eligible for local validation | available |
| adapter absent/restarted | unavailable | closed | unavailable |
| native Zenoh unavailable | unavailable | closed | unavailable |
| local DDS/ROS graph failed | unavailable | closed | unavailable |
| required ros2dds failed | unavailable | closed | unavailable |

“Eligible” is deliberately not “authorized”: command/action conversion still
requires the existing local lease, source guard and local deadline path.
Runtime evidence has no lease-grant API and never carries an executable command
expiry.

## 7. Explicit route verification

Profile tests select exact route IDs and assert:

- IMU sensor: SI fields, `imu_link` alias, best-effort/volatile keep-last QoS;
- chassis command: `TwistStamped`, body frame, reliable/volatile depth one,
  short ROS age and local deadline construction;
- health service: reliable/volatile, bounded timeout, idempotent request;
- trajectory action: registered goal/feedback/result types, bounded feedback
  and result cache, status durability and distinct cancel/terminal states.

Pure projection and lifecycle tests remain the executable proof for frame,
clock, lease, stale command and cancellation semantics.

## 8. Rollback and non-claims

The example and integration test can be removed without changing the canonical
protocol. The runtime module and two additive manifest fields are isolated to
`ros2-mapping`; rollback removes them together.

This task does not run a camera, Orange Pi, motor, ROS graph, DDS daemon or
ros2dds binary. The host TCP process test is real Zenoh evidence, while ROS
results are contract/state-machine evidence only.
