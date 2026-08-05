# Motor-disabled chassis navigation plugin

The `chassis_nav` integration is a crash-isolated ROS 2/Nav2 client of
`rs_control`. Nav2 produces a controller proposal; it does not own a lease,
final motion constraints, mecanum projection, motor output or CAN. The checked
host profile ends at a bounded fake sink and cannot enable hardware.

## Runtime boundary

The profile consists of three supervised services in one accounting slice:

1. `rs-navigation-e2e` creates the synthetic test lease, owns the sole final
   limiter, projects the command through the mecanum model, and publishes
   deterministic odometry/TF loopback.
2. The weak-host deployment validates the `chassis_nav_bringup` contract and
   starts only map server, controller server, planner server, BT navigator and
   lifecycle manager as native processes. It excludes the persistent Python
   `ros2 launch` parent, Gazebo,
   RViz, scenario runners, online mapping, Zenoh bridges and the Nav2 velocity
   smoother.
3. `rs-navigation-bridge` is the only added DDS participant. It maps the Nav2
   action and raw `Twist` proposal to fixed-capacity local IPC. It cannot mint
   or extend authority.

All three processes require the same exact plugin-manifest SHA-256. The launch
also verifies the map manifest, occupancy map/image, three parameter layers,
two behavior trees and ROS interface snapshots before creating any Nav2 node.
The current manifest is deliberately `simulation_only`,
`approved_for_production: false`, `motor_output: forbidden` and
`can_transmit: forbidden`.

## Command and failure behavior

The Rust owner rejects a proposal unless its source boot, sequence, reception
age, command ID, owner, lease ID and session epoch match current local state.
It then applies the sole final speed/acceleration constraint and mecanum
projection. A stale proposal, lost bridge/IPC peer or expired authority causes
zero output within the 100 ms proposal lifetime; no ROS-side process can
refresh that deadline after it exits.

`rclrs 0.7` generates the ROS action UUID. The bridge retains one bounded
canonical-command-ID to ROS-UUID binding and checks it for feedback, cancel and
terminal result correlation. Cancel acceptance is not treated as completion.
A semantic behavior-tree ID is checked by the bridge and mapped to Nav2's
digest-bound launch default; it is never misused as an unchecked file path.
A new bridge or harness start always receives a new nonzero process epoch, so
old shared-memory publisher state is retired. The bridge can publish TF before
Nav2 activation, but holds its one pending goal until systemd marks the action
and all four managed lifecycle nodes ready through a supervisor-owned file.

The checked systemd target binds all three services: critical exit tears down
the integration target, while the Rust control path still reaches its own
bounded local stop. Restart rate limits and hardening are encoded in the unit
files; the Rust FSM is not a process supervisor.

## Host validation and remaining physical gates

The reproducible host checks are:

```bash
cargo test -p navigation-plugin-config -p navigation-ipc -p rs-navigation-e2e
source /opt/ros/humble/setup.bash
platform/ros2-humble/navigation-bridge/build.sh --verify-only
platform/ros2-humble/navigation-deployment/verify.sh
platform/ros2-humble/navigation-deployment/tests/motor-disabled-e2e.sh \
  success /absolute/bridge /absolute/harness
```

The E2E runner also supports `cancel`, `fault` and `nav2-fault`. The latter two
terminate the bridge or fake Nav2 server and require typed `PeerUnavailable`
or `ProposalExpired` stops while the Rust process stays alive. The real-Nav2
resource smoke accepts `success` or `localization-fault`; the fault mode stops
odometry/TF after the first proposal and requires a failed action followed by
a proposal expiry no later than the 100 ms TTL plus one 5 ms observation
period. Resource evidence is non-overwriting, binds both executable digests,
and contains raw `/proc`/cgroup observations plus `SHA256SUMS`. Without systemd
privileges, explicit launch PIDs can be supplied and descendants are included.

The checked 2026-08-04 host run and its portable raw observations are retained
in [`evidence/navigation-plugin-host-2026-08-04.md`](../evidence/navigation-plugin-host-2026-08-04.md).

These checks qualify software contracts on the host only. A production map,
physical localization/obstacle sensors, traction, calibration, AArch64 timing,
realtime scheduling, production SLAM load and whole-robot resource suitability
remain pending until the corresponding hardware and commissioned budget are
available.
