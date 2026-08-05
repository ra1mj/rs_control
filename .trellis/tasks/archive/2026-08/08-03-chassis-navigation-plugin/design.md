# Chassis navigation plugin integration — technical design

## 1. Scope and selected architecture

This task integrates `chassis_nav` as a crash-isolated upper-layer client of
`rs_control`. Nav2 remains a ROS 2 process group. It never enters the realtime
motion process and never receives a hardware handle.

The first delivery is a motor-disabled host loop. It exercises the complete
mission/action/command/feedback/fault path, but terminates at a fake sink. It
cannot approve physical motion, a production map, AArch64 timing or a complete
robot resource budget.

This remains one integration task rather than a parent with three new child
tasks. The shared manifest, semantic contract, ROS adapter, Nav2 launch profile
and end-to-end harness form one compatibility boundary and must be reviewed
together before any part can claim success. Implementation will still use
separate, rollback-safe commits for the contract, each repository's runtime
changes, and qualification evidence.

The selected co-located transport is the existing fixed-capacity
`middleware-shm` implementation. A local navigation integration does not need
`zenohd` or `zenoh-bridge-ros2dds`; omitting both saves a process, discovery
traffic, threads and duplicate serialization. The ROS adapter joins the local
DDS graph directly through exact-pinned `rclrs` and is the only new DDS
participant outside Nav2.

## 2. Process and authority boundaries

```text
systemd / test orchestrator
  |
  +-- chassis-nav-minimal
  |     map_server
  |     planner_server
  |     controller_server --> /rs_control/navigation/controller_proposal
  |     bt_navigator       <-> /navigate_to_pose
  |     lifecycle_manager
  |
  +-- rs-navigation-bridge           Rust, rclrs, non-RT
  |     ROS action/topic side <-> bounded semantic mapping
  |     ordered/latest shared-memory endpoints
  |
  +-- rs-navigation-e2e              Rust, motor-disabled host harness
        lifecycle + mission + synthetic lease owner
        local lease revalidation + final limiter + mecanum projection
        fake sink + deterministic odometry/TF loopback
```

The bridge cannot issue, renew or extend a lease. The E2E harness is a separate
authority/control owner and is the only process allowed to create the
synthetic test lease. In a production deployment, the real FSM/authority and
`motion-rt` replace the harness while the bridge contract remains unchanged.

Killing Nav2 or the bridge leaves the Rust harness alive. Proposal silence,
bridge boot change, IPC loss or lease expiry invalidates the cached command and
causes a local zero-output decision within the declared freshness bound.

## 3. Repository ownership

### `rs_control`

- `message-model` owns backend-neutral map identity, navigation goal,
  feedback/result, bridge authority state and proposal semantics.
- `navigation.proto` owns the external `robot.v1` wire messages.
- `contract-codec` is the only Protobuf conversion boundary.
- A new protocol-layer navigation IPC crate owns fixed-layout
  `middleware-shm` representations and codecs. It does not expose ROS types.
- `ros2-mapping` validates the navigation routes, interface snapshots,
  manifest identity, direction, QoS, freshness and frame ownership.
- A motor-disabled `app` composition root wires existing lifecycle, mission,
  lease, limiter and mecanum crates to a fake sink and deterministic loopback.
  It has no dependency on any hardware crate.
- The ROS adapter is an isolated nested Cargo project under the ROS platform
  boundary. Its checked source and lockfile are retained, while a build script
  verifies installed Humble Rust message sources and renders absolute paths
  only into a temporary build tree.

### `chassis_nav`

- A separate `rs_control` launch/profile starts only the four required Nav2
  lifecycle nodes plus the lifecycle manager.
- The controller publishes its raw `geometry_msgs/msg/Twist` proposal to the
  manifest-owned internal topic.
- The integrated profile does not start `nav2_velocity_smoother`, Gazebo, RViz,
  the scenario runner, online mapping or a device driver.
- Static-map and synthetic localization inputs remain explicitly
  `simulation_only` and `approved_for_production: false`.
- Contract tests validate exact node set, topic ownership, interface identity,
  map/BT/config digests and the absence of a final ROS command publisher.

The existing standalone `minimal_navigation.launch.py` remains available for
component testing. The integration profile is separate so removing its
velocity smoother does not silently change already collected standalone
evidence.

## 4. Shared plugin manifest

Both repositories retain byte-identical copies of one versioned YAML manifest.
Each repository validates its local assets against the copy, and a
cross-repository check compares the complete SHA-256 before an integration run.

The manifest contains:

- schema, plugin and compatibility profile IDs;
- ROS distribution, RMW selection and exact `rclrs`/message-package versions;
- Protobuf descriptor SHA-256 and semantic payload type names;
- Nav2 action, proposal, odometry and TF endpoints;
- fixed shared-memory namespace/channel names, layout IDs, queue capacities and
  freshness limits;
- canonical/ROS frame aliases and single-writer TF authorities;
- map identity plus map YAML/image digests;
- parameter overlay and behavior-tree digests;
- final limiter owner `rs_control`;
- `motor_output: forbidden` and `can_transmit: forbidden` for this profile.

Every executable receives the manifest path and expected digest explicitly.
Unknown keys, a digest mismatch, a wrong map/config/BT artifact, a different
ROS interface source or a production-output flag fail before navigation
eligibility.

## 5. Semantic and wire contracts

Additive `robot.v1` navigation messages carry:

- `NavigationMapIdentity`: stable ID/version, 32-byte artifact digest and map
  frame;
- `NavigateToPoseGoal`: exact map identity, goal pose in SI units and an
  allowlisted behavior-tree identity;
- `NavigateToPoseFeedback`: current pose, elapsed/estimated time, recovery
  count and remaining distance;
- `NavigateToPoseResult`: completed map identity and final bounded feedback
  evidence;
- `NavigationAuthorityState`: source boot/sequence, the current unexpired
  chassis lease or an explicit unavailable state, and the active command ID;
- `NavigationHealth`: process epoch, manifest identity, readiness gates,
  counters and typed fault codes.

The existing generic `ActionGoalIntent`, feedback, cancellation and terminal
result envelopes remain the action-lifecycle carrier. The action name is
`navigate_to_pose_v1`; registered payload type names must match the route
allowlist exactly.

## 6. Local IPC profile

Every shared-memory representation is fixed-layout, little-endian and versioned
through `ShmPayload`. Semantic values are encoded/decoded only through
`contract-codec`; raw Rust layout is never treated as the wire contract.

| Channel | Mode | Capacity | Policy |
|---|---:|---:|---|
| authority state | ordered | 8 | exact source/boot/sequence; lease expiry is local monotonic |
| action goal/cancel | ordered | 8 | exact command correlation; no overwrite |
| action feedback/result | ordered | 8 | bounded feedback; terminal result retained by owner |
| controller proposal | latest | 3 physical / 1 logical | 100 ms maximum reception lifetime |
| odometry | latest | 3 physical / 1 logical | 100 ms maximum age; no zero synthesis |
| dynamic TF batch | latest | 3 physical / 1 logical | one declared writer per child edge |
| bridge health | latest | 3 physical / 1 logical | process epoch and manifest identity required |

The latest-value channels expose only the newest logical value. Their physical
capacity is three because `middleware-shm` layout v1 uses a three-slot seqlock
window to avoid reading a slot under overwrite; this is not a depth-three
application queue.

Attach rejects layout/schema/endianness mismatch before exposing an endpoint.
Restart creates a new boot/process epoch, retires old source state and prevents
old slots from becoming executable input.

## 7. ROS adapter

The implementation uses exact `rclrs = 0.7.0` with ROS Humble. A host probe
already proved that the installed `nav2_msgs 1.1.20` and
`geometry_msgs 4.9.1` Rust bindings compile offline and expose
`NavigateToPose` action support.

The adapter uses one basic executor and bounded callbacks:

- action client: `/navigate_to_pose`;
- subscription: raw `geometry_msgs/msg/Twist`, keep-last depth 1;
- publishers: odometry and declared TF routes only;
- lifecycle/action-server readiness checks outside the command callback;
- no Python runtime, plugin loading, logging in high-rate callbacks or
  unbounded callback queue.

`Twist` has no ROS stamp or authority. The adapter accepts it only while one
exact navigation action is active and its matching authority snapshot is
current. It creates a new adapter header/sequence and a bounded
`ChassisSetpointIntent`; the receiving Rust control owner independently matches
the asserted lease and constructs the executable local deadline.

`rclrs 0.7` creates the ROS goal UUID internally. The adapter therefore keeps
one bounded active-goal binding from the canonical 16-byte command ID to that
exact ROS UUID, and verifies both sides for feedback, cancellation and result
correlation. Goal acceptance, executing feedback, cancel acceptance and the
terminal result are mapped without treating publication or cancel acceptance
as completion.

The semantic goal carries an allowlisted behavior-tree identity rather than a
filesystem path. After checking that identity, the adapter leaves the ROS goal
field empty so Nav2 uses the exact default BT file already digest-bound by the
validated launch profile.

Generated ROS Rust sources are not committed. The isolated build verifies an
exact source lock for all required installed packages, renders a temporary
manifest, and runs Cargo offline/locked. No `/opt/ros` path enters the root
workspace manifest or lockfile.

## 8. Rust command and fake-output loop

The motor-disabled harness processes each proposal in this order:

1. reject foreign source/boot, duplicate/reordered sequence or expired cache;
2. materialize the intent against the current exact local chassis lease;
3. authorize the resulting envelope through `LeaseEnforcementCache`;
4. apply `chassis-model::constrain_body_command`—the same allocation-free
   primitive used by `trajectory-se2`—as the sole final speed/acceleration
   limiter;
5. project the constrained body command through `chassis-mecanum`;
6. send only to a bounded fake sink;
7. integrate the achieved simulated body twist and publish semantic odometry
   and TF back through shared memory.

The harness crate graph excludes `hardware-module-core`, `motor-damiao` and
`transport-socketcan`. It has no interface name, file descriptor or trait
object through which CAN could be opened. `motor_output: forbidden` is checked
again at startup; this flag cannot be changed at runtime.

When no usable command exists, the next control tick emits a typed local stop
decision and the fake sink records zero. Missing feedback is not represented as
numeric zero; the loopback state becomes unavailable if its own update path
fails.

## 9. Readiness and fail-closed behavior

Navigation is eligible only when all of the following match:

- expected plugin-manifest digest;
- exact synthetic map identity and artifact digests;
- current adapter process epoch and source boot;
- action server and required Nav2 lifecycle nodes ready;
- fresh odometry plus complete `map -> odom -> base_link` ownership;
- current chassis lease owned by the mission;
- motor-disabled output gate active.

| Failure | Bridge behavior | Rust control behavior |
|---|---|---|
| Missing/wrong/expired lease | reject/cancel goal; publish no proposal | reject intent and stop locally |
| Stale/duplicate proposal | count and discard | no command refresh; bounded stop |
| Nav2/controller exit | action fails; health unavailable | proposal expires; process remains alive |
| Adapter exit/restart | systemd may restart with new epoch | retire old boot; proposal expires |
| IPC producer/consumer loss | typed health fault | no cached-command refresh; stop |
| Localization/TF loss | cancel active goal | stop; do not synthesize pose |
| Cancel accepted | remain canceling | keep authority until terminal or lease loss |
| Cancel terminal/rejected | exact terminal result/rejection | release only after correlated terminal evidence |
| E2E harness exit | adapter loses authority and cancels | fake sink disappears; no hardware exists |

systemd owns process start/restart and rate limiting. Checked example units and
a motor-disabled target define ordering, restart limits, process hardening and
one cgroup/accounting boundary. The Rust FSM consumes health and authority
state; it does not spawn Nav2 or the bridge.

## 10. Low-resource runtime profile

The integrated Nav2 graph contains map server, Smac 2D planner, MPPI controller,
BT navigator and lifecycle manager only. It reuses the four-node fail-closed
behavior tree, disables MPPI visualization, uses keep-last queues, incremental
costmaps and the lowest validated controller/planner/update rates.

Resource evidence is collected from one cgroup containing Nav2, the adapter,
the motor-disabled Rust harness and the selected localization/SLAM profile.
The report records per-process and aggregate CPU time, cgroup current/peak
memory, RSS/PSS where available, threads, context switches, page faults,
deadline misses, proposal rate/drop/rejection counters and stop latency.

The existing Nav2-only host observation is retained as component evidence. It
must not be presented as the whole-robot budget. The first integrated run is
explicitly measurement-only because no commissioned host budget or target
hardware is available; measurements are reported without claiming target
suitability.

## 11. Verification strategy

Verification proceeds from pure contracts to processes:

1. semantic constructors and Protobuf good/base/bad round trips;
2. manifest/interface/source-lock mismatch rejection;
3. SHM layout, stale/replay, crash-reclaim and bounded-channel tests;
4. fake-sink graph proof and final limiter/mecanum tests;
5. isolated rclrs adapter offline build and action/proposal mapping tests;
6. chassis-nav static contract tests for exact process/topic ownership;
7. nominal goal success and explicit cancellation through the complete loop;
8. wrong lease, expiry, duplicate/reordered proposal and prior-boot replay;
9. SIGTERM/SIGKILL of controller, Nav2 group, adapter and IPC peer;
10. concurrent host resource report with truthful evidence labels;
11. both repositories' full host CI and cross-repository manifest check.

No test opens SocketCAN, enables a motor, writes a physical device or marks a
host result as AArch64/realtime/production qualification.

## 12. Rollback and coexistence

All additions are isolated behind a new plugin manifest, launch profile,
adapter build and motor-disabled app. Existing ROS routes and the standalone
Nav2 profile remain unchanged until the new loop passes. Rollback disables the
new systemd target and removes the new profile; no robot YAML or hardware
binding is rewritten.

The current root Cargo and architecture manifests are clean, but Trellis task
metadata/bootstrap files are unrelated work. The external `chassis_nav`
worktree contains extensive pre-existing modified and untracked files.
Implementation patches must inspect each overlapping file, make minimal
additive edits and never restore, regenerate or overwrite unrelated changes.

## 13. Verified implementation inputs

The planning audit established the following local facts rather than assuming
them from package names:

- `middleware-shm` uses Linux `mmap` and already tests real cross-process
  ordered delivery, killed-peer detection, backlog skipping and endpoint
  reclaim.
- cached `rclrs = 0.7.0` source includes Humble `rcl_action` bindings,
  `Node::create_action_client`, goal UUIDs, feedback, cancellation and result
  clients. Installed `nav2_msgs` implements `rosidl_runtime_rs::Action` for
  `NavigateToPose`.
- the pinned `rclrs`, `geometry_msgs` and `nav2_msgs` dependency probe builds
  locally with `cargo check --offline --locked` after sourcing ROS Humble. This
  proves dependency admission; a real Nav2 action round trip remains an
  implementation acceptance test.
- the existing `chassis_nav` minimal launch is headless and shuts down on a
  critical process exit, but intentionally includes `nav2_velocity_smoother`.
  Therefore the rs_control profile must be a separate launch and must not
  rewrite existing standalone evidence.
- the existing `/robot_1/cmd_vel_stamped` route is ROS-to-Rust ingress for an
  authorized stamped command. Nav2 emits an unstamped private controller
  proposal, so it cannot be connected directly to that route or treated as an
  executable command; the isolated adapter and local revalidation boundary are
  required.
