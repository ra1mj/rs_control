# Research: ROS 2 bridge contracts for `protocol_module`

- Query: What official ROS 2 rules must a future `protocol_module` ↔ ROS 2 bridge obey for units, coordinate frames, QoS, time, headers, TF, services/actions, cancellation, feedback/results, type descriptions/hashes, and interface evolution?
- Scope: mixed (project contracts plus ROS official REPs, design documents, documentation, and source)
- Date: 2026-07-30

## Findings

### 1. Files found in this repository

- `.trellis/spec/backend/directory-structure.md:3-29` — defines the current hardware_module/protocol_module/application_module dependency classes; an external ROS bridge belongs behind `protocol_module` semantic ports rather than exposing ROS DTOs to `application_module`.
- `.trellis/spec/backend/robot-contracts.md:33-61` — makes `robot.v1` the Protobuf/key-space major version, requires SI/right-handed coordinates, and centralizes validation at the protocol boundary.
- `.trellis/spec/backend/error-handling.md:3-29` — requires typed boundary errors and rejects missing/stale data rather than manufacturing zeros.
- `.trellis/spec/backend/quality-guidelines.md:25-40` — forbids unbounded queues, hidden allocation, or blocking work on declared real-time paths.
- `proto/robot/v1/common.proto:31-50` — the project `Header` carries source/boot identity, sequence, source-monotonic time, optional Unix correlation time, validity, optional frame, clock relationship, and quality.
- `proto/robot/v1/common.proto:53-93` — project vectors/quaternions/planar pose and twist already use SI units, right-handed body axes, and radians.
- `proto/robot/v1/device.proto:27-42` — `ImuState` is close to `sensor_msgs/msg/Imu`, but its covariance fields are variable-length Protobuf arrays and therefore require exact-length validation before ROS conversion.
- `proto/robot/v1/control.proto:7-17` — project action states include accepted/running/succeeded/failed/cancelled/timed-out/rejected but no explicit canceling state.
- `proto/robot/v1/control.proto:33-48` — `CommandEnvelope.command_id` is a nonzero 16-byte idempotency/correlation ID and deadlines are in a local monotonic domain.
- `proto/robot/v1/control.proto:59-80` — `ActionCommand` uses a versioned name plus opaque Protobuf bytes and `ActionAck` has status/result detail, but there is no action feedback contract or cancellation request contract yet.
- `proto/robot/v1/perception.proto:7-28` — perception positions, velocities, and poses are explicitly expressed in `Header.frame_id`.
- `crates/protocol_module/message-model/src/lib.rs:12-58` — all current IDs share an ASCII alphabet that excludes `/`; this accepts canonical names such as `base_link` but cannot represent namespaced TF frames such as `robot_1/base_link`.
- `crates/protocol_module/message-model/src/lib.rs:193-313` — the semantic `Header` preserves project monotonic/correlation semantics and validates nonzero validity.
- `.trellis/tasks/07-29-socketcan-damiao/design.md:15-19` — Linux/file-descriptor types must stop at `hardware_module`; the future protocol adapter receives only validated samples and health summaries.

### 2. Normative source baseline and status

The durable citations below use immutable GitHub commits where possible. The classic REPs are active:

- [REP-103, active source at `11ca24a`](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst) — units and coordinate conventions.
- [REP-105, active source at `11ca24a`](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst) — mobile-platform frames and frame authorities.
- [REP-145, draft source at `11ca24a`](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0145.rst) — useful IMU conventions, but its status is Draft rather than Active.

Type-description terminology needs a status warning. The proposed REP-2011 remains an open draft PR and is absent from the accepted `ros-infrastructure/rep` master tree as of this research date: [official PR #358](https://github.com/ros-infrastructure/rep/pull/358). Some ROS source comments also refer to “REP-2016”, but no accepted `rep-2016.rst` exists in that tree. The shipped Iron-and-later APIs and source listed in section 10 are therefore the executable authority; neither a future automatic conversion system nor full draft REP behavior should be assumed.

### 3. REP-103 units and coordinate rules

Official facts:

- ROS standardizes on SI base and derived units; angle is radians, force newtons, and temperature degrees Celsius ([REP-103 lines 40-83](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L40-L83)).
- All coordinate systems are right-handed ([lines 89-99](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L89-L99)).
- A body frame uses x forward, y left, z up ([lines 102-110](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L102-L110)).
- A camera `_optical` frame uses z forward, x right, y down; a `_ned` secondary frame uses x north, y east, z down ([lines 124-140](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L124-L140)).
- Quaternions are preferred; fixed-axis roll/pitch/yaw are rotations about X/Y/Z. Positive yaw is counter-clockwise and geographic yaw zero is east, not compass north ([lines 145-175](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L145-L175)).
- Covariance is row-major, with angular dimensions ordered about fixed x/y/z axes ([lines 178-199](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0103.rst#L178-L199)).

Required bridge policy:

1. Perform unit and axis conversion at the typed bridge boundary. Never relabel a left-handed, NED, camera-native, degree, RPM, or g-valued sample as ROS-conforming data.
2. Preserve the project’s existing x-forward/y-left/z-up and radian semantics where they already match ROS.
3. Publish a separate, correctly transformed camera optical frame; do not put optical-axis data under the body camera link merely by changing `frame_id`.
4. Validate every covariance length and ordering. In particular, each `ImuState` covariance must contain exactly nine finite values before mapping to the fixed `float64[9]` ROS arrays.
5. `sensor_msgs/msg/Imu` says all-zero covariance means unknown, while `covariance[0] == -1` means the associated estimate is unavailable ([official message source](https://github.com/ros2/common_interfaces/blob/0a91a3f88fcaa24364da2392017c174a8fd69b6d/sensor_msgs/msg/Imu.msg#L1-L24)). Preserve this distinction; missing orientation must not become an identity quaternion with zero covariance.
6. REP-145 further says IMU `frame_id` identifies the sensor frame (normally `imu_link`), mounting relative to `base_link` belongs in TF, acceleration is m/s², angular rate is rad/s, and NED data needs an explicit convention/transform ([REP-145 lines 24-89](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0145.rst#L24-L89)). Treat this as recommended interoperability guidance because REP-145 is Draft.

### 4. REP-105 frame graph and authority

Official facts:

- `base_link` is rigidly attached to the mobile base ([REP-105 lines 39-47](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L39-L47)).
- `odom` is world-fixed and continuous but may drift without bound; it is the short-term local reference ([lines 49-65](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L49-L65)).
- `map` is world-fixed and should not drift significantly, but localization corrections may cause discrete jumps; it is unsuitable as the direct local-control reference ([lines 67-83](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L67-L83)).
- The minimal tree is `earth -> map -> odom -> base_link`, and every frame has exactly one parent ([lines 140-168](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L140-L168)).
- The odometry authority publishes `odom -> base_link`; localization consumes that and publishes `map -> odom`, not a competing `map -> base_link` edge ([lines 204-220](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L204-L220)).

Required bridge policy:

1. The chassis estimator/odometry owner is the sole authority for `odom -> base_link`. The localization owner is the sole authority for `map -> odom`. The ROS bridge must not duplicate a transform already published by a native ROS estimator.
2. Local chassis tracking and closed-loop control use the continuous `odom`/body relationship, not jump-prone `map` corrections directly.
3. Rigid mount relationships such as `base_link -> imu_link`, camera body link -> camera optical frame, and lidar link are static TF. Estimated robot motion is dynamic TF.
4. Multi-robot deployment must disambiguate frames when data leaves a robot. REP-105 permits canonical on-robot IDs and rewrite/remap at forwarding boundaries ([lines 179-200](https://github.com/ros-infrastructure/rep/blob/11ca24a41f31480dfb9562ba99f2a5b93d3ebda5/rep-0105.rst#L179-L200)). The project’s current `AsciiId` rejects every `/`, so a decision is needed before supporting common namespaced forms such as `robot_1/base_link`.

### 5. `Header`, `frame_id`, and TF wire semantics

Official facts:

- ROS 2 `std_msgs/msg/Header` contains only `stamp` and `frame_id`; it is for timestamped data in a coordinate frame ([source](https://github.com/ros2/common_interfaces/blob/0a91a3f88fcaa24364da2392017c174a8fd69b6d/std_msgs/msg/Header.msg#L1-L9)). It cannot carry the project’s boot ID, sequence, validity, quality, or clock uncertainty.
- `geometry_msgs/msg/TransformStamped` uses `header.frame_id` as parent/reference, `child_frame_id` as child, and the transform describes the child from the parent at `header.stamp` ([source](https://github.com/ros2/common_interfaces/blob/0a91a3f88fcaa24364da2392017c174a8fd69b6d/geometry_msgs/msg/TransformStamped.msg#L1-L19)).
- `nav_msgs/msg/Odometry` puts pose in `header.frame_id`, while twist is expressed in `child_frame_id` ([source](https://github.com/ros2/common_interfaces/blob/0a91a3f88fcaa24364da2392017c174a8fd69b6d/nav_msgs/msg/Odometry.msg#L1-L15)).
- tf2 rejects empty frame IDs and IDs beginning with `/` ([`BufferCore` source](https://github.com/ros2/geometry2/blob/672e53d7b47f57bde17cd6fc16ff5136c9a98edb/tf2/src/buffer_core.cpp#L107-L145)).
- Standard broadcasters publish dynamic transforms on `/tf` and static transforms on `/tf_static` ([dynamic source](https://github.com/ros2/geometry2/blob/672e53d7b47f57bde17cd6fc16ff5136c9a98edb/tf2_ros/include/tf2_ros/transform_broadcaster.hpp#L71-L90), [static source](https://github.com/ros2/geometry2/blob/672e53d7b47f57bde17cd6fc16ff5136c9a98edb/tf2_ros/include/tf2_ros/static_transform_broadcaster.hpp#L70-L89)). Static listener and broadcaster QoS are transient-local; current defaults use depth 100 for listener and 1 for broadcaster ([source](https://github.com/ros2/geometry2/blob/672e53d7b47f57bde17cd6fc16ff5136c9a98edb/tf2_ros/src/qos.cpp#L37-L57)).

Required bridge policy:

1. `frame_id` names a coordinate frame, not a device, node, topic, or source. Use the project `source_id` for provenance and a ROS frame only for geometry.
2. Do not strip project metadata silently. Either publish a companion typed metadata/diagnostic message keyed by `(source_id, boot_id, sequence)`, or define a versioned custom ROS message when consumers require validity/quality/clock uncertainty.
3. Do not construct TF from a single project `Header.frame_id`; TF additionally requires an explicit child frame and transform. A dedicated semantic transform contract is required.
4. Publish through `tf2_ros` broadcasters and use a `tf2_ros::Buffer` for lookup. Do not create a second transform graph or hand-roll `/tf` serialization.
5. Static TF is only for a truly time-invariant relationship. Dynamic transforms use the sample’s ROS timestamp. A bridge must not block a real-time control thread waiting for a TF lookup.

### 6. ROS time versus monotonic/steady time

Official facts:

- ROS defines `SystemTime`, `SteadyTime`, and `ROSTime`. ROS time follows system time until a ROS time source is active; with `use_sim_time`, it follows `/clock` and may pause or jump backward/forward ([official clock design](https://design.ros2.org/articles/clock_and_time.html)).
- Hardware timeouts are a canonical use of steady time, but steady/system timestamps should be isolated internally and translated to ROS time for external ROS interfaces ([clock design, “Steady Time”](https://design.ros2.org/articles/clock_and_time.html#steady-time)).
- ROS time can be uninitialized at zero and supports time-jump callbacks; consumers must not assume it always increases ([clock design, “Default Time Source”](https://design.ros2.org/articles/clock_and_time.html#default-time-source)).
- `builtin_interfaces/msg/Time` is signed `int32 sec` plus normalized `uint32 nanosec` in `[0, 1e9)` ([source](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/builtin_interfaces/msg/Time.msg#L1-L10)).

Required bridge policy:

1. Keep all project freshness checks, command deadlines, leases, retry timers, and hardware watchdog logic in their monotonic/steady domains. Never convert a safety deadline to ROS time.
2. Use the node’s ROS clock for public ROS stamps and TF so simulation/bag playback works. Register time-jump handling for caches, derivative/integral state, and time-ordered buffers that depend on ROS time.
3. Never copy `source_monotonic_ns` into `Header.stamp`. A monotonic epoch is process/boot-local and has no ROS-time meaning.
4. The current `publish_unix_ns` is documented as optional publication-time correlation, not physical acquisition time. It is therefore not a faithful source for a sensor `Header.stamp`. A future bridge needs an explicit clock-correlation input that maps the source acquisition monotonic timestamp to ROS time with measured uncertainty.
5. If no trustworthy acquisition-time correlation exists, the adapter must use an explicit degraded policy (for example, bridge-receipt ROS time plus companion provenance) or refuse the time-sensitive mapping. It must not silently claim receipt/publication time is measurement time.
6. Reject zero/uninitialized ROS time when the target interface requires a valid acquisition time, reject values outside `int32 sec`, normalize nanoseconds, and never compare timestamps from different clock domains.

### 7. QoS contracts

Official profiles in current ROS source:

| Use | History/depth | Reliability | Durability | Official source |
|---|---|---|---|---|
| Default pub/sub | keep-last 10 | reliable | volatile | [`rmw_qos_profile_default`](https://github.com/ros2/rmw/blob/8a08db2fb7c8d5986fa5ec449bb422f77e257ceb/rmw/include/rmw/qos_profiles.h#L51-L62) |
| Sensor data | keep-last 5 | best-effort | volatile | [`rmw_qos_profile_sensor_data`](https://github.com/ros2/rmw/blob/8a08db2fb7c8d5986fa5ec449bb422f77e257ceb/rmw/include/rmw/qos_profiles.h#L25-L36) |
| Services | keep-last 10 | reliable | volatile | [`rmw_qos_profile_services_default`](https://github.com/ros2/rmw/blob/8a08db2fb7c8d5986fa5ec449bb422f77e257ceb/rmw/include/rmw/qos_profiles.h#L64-L75) |
| Action goal/cancel/result services | services default | reliable | volatile | [`rcl_action` defaults](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl_action/include/rcl_action/action_server.h#L233-L244) |
| Action feedback | default pub/sub | reliable | volatile | [same action defaults](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl_action/include/rcl_action/action_server.h#L233-L244) |
| Action status | keep-last 1 | reliable | transient-local | [`rcl_action_qos_profile_status_default`](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl_action/include/rcl_action/default_qos.h#L26-L37) |

Compatibility is request-versus-offer: reliable publisher → best-effort subscriber is compatible, but best-effort publisher → reliable subscriber is not; transient-local publisher → volatile subscriber is compatible for new samples, while volatile publisher → transient-local subscriber is not. All compatibility-affecting policies must match sufficiently or no data flows ([official QoS documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities)). Services intentionally use volatile durability so a restarted server does not receive stale requests ([official QoS documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-profiles)).

Required bridge policy:

1. Pin a named QoS contract per mapped endpoint; do not use one global profile.
2. Use sensor-data QoS as the default for high-rate IMU/camera/lidar/motor-state telemetry where latest data is more valuable than retransmitting stale samples. A specific estimator may override this only with a measured reason.
3. For latest-value motion commands, use an explicit bounded volatile profile; project recommendation is keep-last 1 and reliable unless target-network evidence justifies another choice. Keep the project monotonic deadline as the final safety check because reliable QoS is not a latency guarantee.
4. Use service defaults for short request/reply operations and the ROS action defaults for actions. Do not make a motion command or service transient-local.
5. Use transient-local only for genuinely retained state/configuration and `/tf_static`; both publisher and late-joining subscriber need compatible durability.
6. Do not use `system_default` for safety-relevant endpoints because behavior is RMW-dependent. Do not use `best_available` there either: the RMW source explicitly warns that discovery races make its selected policy nondeterministic and it does not update after endpoint creation ([source](https://github.com/ros2/rmw/blob/8a08db2fb7c8d5986fa5ec449bb422f77e257ceb/rmw/include/rmw/qos_profiles.h#L116-L140)).
7. At startup, inspect endpoint QoS/type information, fail or degrade explicitly on incompatibility, and expose incompatible-QoS/liveliness/deadline events to monitoring. Keep ROS publication/retry work off the hard real-time control thread.

### 8. Services versus actions

Official ROS guidance is: topics for continuous streams, services for short request/reply work, and actions for long-running work with feedback/cancellation ([official interface guidance](https://docs.ros.org/en/ros2_documentation/rolling/Concepts/Basic/Interfaces-Topics-Services-Actions.html)). The action design specifies goal, result, and feedback as three distinct typed sections and implements an action with three services plus two topics ([design lines 117-140](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L117-L140), [lines 221-225](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L221-L225)).

Required bridge policy:

1. Use a service only for bounded, quick operations such as state queries or small calculations. Use an action for chassis navigation, arm motion, calibration, or an FSM mission step that can run long, report progress, or be canceled.
2. Preserve idempotency for mutating operations. The project’s 16-byte `command_id` is suitable for mapping an incoming ROS action goal UUID to an internal command ID; a retry must not execute the same logical command twice.
3. Do not expose `ActionCommand.parameters` as an untyped generic ROS byte blob for core APIs. Register each `action_name` against a concrete versioned ROS `.action` goal/result/feedback type and a concrete Protobuf type.

### 9. Action goal, cancellation, feedback, and result semantics

Official facts:

- Only accepted goals enter the state machine. Active states are ACCEPTED, EXECUTING, CANCELING; terminal states are SUCCEEDED, ABORTED, CANCELED ([action design lines 174-207](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L174-L207), [official status message](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/action_msgs/msg/GoalStatus.msg#L1-L32)).
- Accepting a cancel request transitions a goal to CANCELING, not directly to CANCELED. The server may perform safe cleanup before reporting terminal cancellation ([action design lines 181-206](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L181-L206), [example note](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L325-L330)).
- Cancel selection uses the four UUID/timestamp combinations: all, all at/before time, one UUID, or one UUID plus all at/before time. The response distinguishes accepted, rejected, unknown ID, and already terminal and returns the goals now canceling ([official `CancelGoal.srv`](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/action_msgs/srv/CancelGoal.srv#L1-L46)).
- Goal feedback is user-defined and may be published during execution; goal result includes terminal status plus typed result. A server caches results for a configurable timeout so clients/tools can retrieve them ([action design lines 258-309](https://github.com/ros2/design/blob/93a415bf928751d37fce6f83215c521658e20c93/articles/actions.md#L258-L309)). Current `rcl_action` server default result retention is ten seconds ([source](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl_action/include/rcl_action/action_server.h#L233-L244)).

Required project mapping:

| Project status | ROS action representation |
|---|---|
| `ACCEPTED` | accepted goal / `STATUS_ACCEPTED` |
| `RUNNING` | `STATUS_EXECUTING` |
| `SUCCEEDED` | `STATUS_SUCCEEDED` plus typed result |
| `FAILED` | `STATUS_ABORTED` plus stable project result code |
| `CANCELLED` | `STATUS_CANCELED`, only after safe cleanup completes |
| `TIMED_OUT` | `STATUS_ABORTED` plus a typed timeout result code; ROS has no timeout terminal state |
| `REJECTED` | rejected SendGoal response; a rejected goal never enters `GoalStatus` |
| `UNSPECIFIED` | invalid boundary input; never publish as a real state |

The current project contract cannot yet implement a faithful generic ROS action bridge because it lacks an explicit CANCELING state, a cancel request/response contract, and a typed feedback stream. These must be added to the semantic protocol before claiming action compatibility. `ActionAck(CANCELLED)` must not be emitted merely because a cancel request was accepted.

### 10. Type description, type hash, and interface evolution

Shipped official behavior:

- `rosidl_generator_type_description` builds a fully expanded description for messages, services, and actions and hashes its stable serialized representation with SHA-256 ([official README](https://github.com/ros2/rosidl/blob/c16445bdf2ecf68c71a79b70c08b330b4fbfe3da/rosidl_generator_type_description/README.md#L1-L14)).
- The C type hash is `{version, 32-byte value}`; its canonical string format is `RIHS{version}_{value}` ([source](https://github.com/ros2/rosidl/blob/c16445bdf2ecf68c71a79b70c08b330b4fbfe3da/rosidl_runtime_c/include/rosidl_runtime_c/type_hash.h#L25-L65)). Current generation uses version 1 and SHA-256 ([runtime source](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl/src/rcl/type_hash.c#L245-L270)); `rosgraph_msgs/msg/TypeHash` likewise defines version 1 and 32 bytes ([message source](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/rosgraph_msgs/msg/TypeHash.msg#L1-L4)).
- The hashable JSON includes all types and fields recursively but excludes default values, comments, and original plaintext source ([`rcl/type_hash.h`](https://github.com/ros2/rcl/blob/15aeab9f4a2d93fdc45f3d84338453046c18ec22/rcl/include/rcl/type_hash.h#L29-L46)).
- `TypeDescription` contains the root type plus recursive referenced descriptions ([source](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/type_description_interfaces/msg/TypeDescription.msg#L1-L6)).
- `GetTypeDescription` requests an exact type name and RIHS string, optionally returns original sources, and reports explicit failure ([service source](https://github.com/ros2/rcl_interfaces/blob/ce3f0f268ab5fb50d690be9a967acb7162773048/type_description_interfaces/srv/GetTypeDescription.srv#L1-L29)).

Required bridge/version policy:

1. Maintain a generated mapping manifest for every bridge endpoint containing at least: Protobuf full name (`robot.v1...`), Protobuf descriptor-set digest, ROS fully qualified type name, ROS RIHS version/hash, semantic mapping version, frame/unit contract, and QoS profile.
2. Verify the ROS type name and hash at startup/discovery. An unknown hash version, unset hash, name/hash mismatch, or unexpected recursive type description is a typed incompatibility, not a warning to ignore.
3. A ROS type hash establishes structural identity, not semantic equivalence. Because comments/defaults are excluded, semantic units/frame/default-policy changes can occur without changing the hash. Keep the bridge’s explicit semantic mapping version and unit/frame tests.
4. Do not assume a Protobuf wire-compatible change is ROS-interface compatible. Protobuf field-number evolution and ROS RIHS identity are different contracts.
5. Do not assume ROS automatically converts different revisions. REP-2011’s broader transfer-function design is still draft; shipped type descriptions/hashes provide detection and introspection, not a general automatic compatibility layer.
6. For a breaking field or semantic change, create a new versioned ROS type/action/service name or interface package, provide an explicit converter, and dual-publish/dual-serve only for a bounded migration window. Do not mutate a deployed interface in place and rely on its unchanged topic name.
7. Preserve removed Protobuf field numbers as the project already requires. Independently preserve old ROS interface definitions needed for bags and deployed consumers.
8. `GetTypeDescription` and downloaded source text are introspection inputs, not authorization to dynamically execute or deserialize an arbitrary type inside a safety process. Dynamic tooling belongs outside the hard real-time/control boundary.

### 11. Recommended bridge boundary and startup gate

The minimum safe data flow is:

```text
validated protocol_module semantic value
  -> typed ROS mapping (units/frame/time validation)
  -> explicit endpoint QoS and ROS type/hash check
  -> ROS publisher/service/action or tf2 broadcaster
```

For ROS input the checks run in reverse, with authority/lease/deadline validation remaining inside the project boundary. Generated ROS DTOs must not leak into `application_module`.

Before advertising “ready”, the bridge should prove:

1. every configured Protobuf ↔ ROS mapping exists and both schema identities match the manifest;
2. every geometric mapping has a nonempty legal frame and correct REP-103 units/axes;
3. TF has exactly one intended authority for each child and the REP-105 base chain is coherent;
4. acquisition time has a trustworthy ROS-time conversion or the endpoint’s explicit degraded policy is active;
5. requested/offered QoS is compatible and bounded;
6. action mappings support goal rejection, CANCELING, cooperative cancellation, feedback, terminal result, and result retention;
7. no ROS executor, allocation, retry, TF wait, type-description lookup, or discovery path runs on the hardware/control real-time thread;
8. failures and degraded mappings surface as typed monitoring events with endpoint, type hash, frame, clock state, and QoS detail.

## External references

- REP repository snapshot: `ros-infrastructure/rep` `11ca24a41f31480dfb9562ba99f2a5b93d3ebda5`.
- ROS 2 design snapshot: `ros2/design` `93a415bf928751d37fce6f83215c521658e20c93`.
- ROS common interfaces snapshot: `ros2/common_interfaces` `0a91a3f88fcaa24364da2392017c174a8fd69b6d`.
- ROS core interfaces snapshot: `ros2/rcl_interfaces` `ce3f0f268ab5fb50d690be9a967acb7162773048`.
- ROS middleware interface snapshot: `ros2/rmw` `8a08db2fb7c8d5986fa5ec449bb422f77e257ceb`.
- ROS client core/action snapshot: `ros2/rcl` `15aeab9f4a2d93fdc45f3d84338453046c18ec22`.
- ROS IDL snapshot: `ros2/rosidl` `c16445bdf2ecf68c71a79b70c08b330b4fbfe3da`.
- tf2 snapshot: `ros2/geometry2` `672e53d7b47f57bde17cd6fc16ff5136c9a98edb`.
- Current official QoS overview: [ROS 2 QoS settings](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html).
- Current official interface selection overview: [topics, services, and actions](https://docs.ros.org/en/ros2_documentation/rolling/Concepts/Basic/Interfaces-Topics-Services-Actions.html).
- Official time design: [Clock and Time](https://design.ros2.org/articles/clock_and_time.html).
- Official action design: [Actions](https://design.ros2.org/articles/actions.html).

## Related specs

- `.trellis/spec/backend/directory-structure.md`
- `.trellis/spec/backend/robot-contracts.md`
- `.trellis/spec/backend/error-handling.md`
- `.trellis/spec/backend/quality-guidelines.md`
- `.trellis/spec/backend/realtime-platform.md`
- `.trellis/spec/guides/cross-layer-thinking-guide.md`

## Caveats / Not Found

- No accepted `rep-2011.rst` or `rep-2016.rst` was present in the official REP master snapshot. REP-2011 PR #358 remains a draft/open proposal even though a useful subset is shipped in ROS 2. This research deliberately treats the implemented APIs/source as authoritative and does not promise draft conversion behavior.
- REP-145 is Draft, not Active. Its IMU conventions are included as official interoperability guidance but are labeled accordingly.
- The repository does not yet contain a ROS bridge, ROS interface package, semantic transform message, clock-correlation service, cancellation protocol, or action feedback protocol. Exact topic/action names and concrete interface definitions therefore remain future design work.
- Rolling source was inspected to capture current behavior, but all citations are immutable commit links. Implementation must pin a supported ROS distribution and re-check its branch because defaults and APIs can differ by distribution.
- A ROS `Header` cannot losslessly represent the project `Header`; a companion/custom metadata contract is required if ROS consumers need boot identity, source sequence, validity, quality, or clock uncertainty.
