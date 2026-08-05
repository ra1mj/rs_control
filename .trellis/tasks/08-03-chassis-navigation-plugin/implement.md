# Chassis navigation plugin integration — implementation plan

## 1. Planning and dependency admission

- [x] Record the motor-disabled end-to-end delivery choice in the PRD.
- [x] Confirm the parent process, ROS isolation, lease and Nav2 boundaries.
- [x] Inspect existing action, lease, limiter, mecanum, ROS mapping and local
      shared-memory contracts for reuse.
- [x] Reject `r2r` for this host because its binding-generation dependency is
      absent and would add avoidable build machinery.
- [x] Prove `rclrs = 0.7.0`, `nav2_msgs 1.1.20` and
      `geometry_msgs 4.9.1` compile offline against the installed Humble Rust
      message sources.
- [x] Review and approve this PRD/design/implementation plan before starting
      code changes.
- [x] Label the first concurrent host run measurement-only because there is no
      commissioned target budget or target hardware.

Rollback: planning artifacts only; no runtime or root-manifest changes.

## 2. Shared manifest and interface snapshots

- [x] Define the versioned navigation plugin YAML schema and canonical fixture.
- [x] Add byte-identical manifest copies to both repositories.
- [x] Snapshot `NavigateToPose.action` and raw `Twist.msg` sources with exact
      Humble versions/hashes.
- [x] Bind map YAML/image, Nav2 overlay and behavior-tree digests.
- [x] Add local and cross-repository schema/hash checks.
- [x] Reject unknown keys, unsafe output flags, wrong frames/topics, duplicate
      writers, invalid capacities/freshness and mismatched asset digests.

Validation:

```bash
# exact commands will be added with the validator
rs-control navigation-plugin validate <manifest>
chassis-nav navigation-plugin validate <manifest>
```

## 3. Navigation semantic and Protobuf contract

- [x] Add bounded map identity, NavigateToPose goal/feedback/result,
      authority-state and health semantic types to `message-model`.
- [x] Add additive `robot.v1` messages to `navigation.proto`.
- [x] Add every mapping only through `contract-codec`.
- [x] Reuse generic action goal/cancel/feedback/result envelopes and exact
      command-ID correlation.
- [x] Test zero/all-zero identities, wrong map/frame/action/payload types,
      missing fields, bounds, NaN/Inf, unknown enums and terminal status rules.
- [x] Run Buf format/lint/breaking and update descriptor/route identities only
      after compatibility succeeds.

Validation:

```bash
cargo test -p message-model -p contract-codec
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
```

## 4. Fixed-capacity navigation IPC

- [x] Add and classify a protocol-layer navigation IPC crate.
- [x] Implement fixed-layout codecs for authority, action lifecycle, proposal,
      odometry, TF and health values without treating Rust memory as the wire
      schema.
- [x] Use ordered channels for authority/actions and latest depth-one channels
      for proposal/odometry/TF/health.
- [x] Enforce manifest layout IDs, payload bounds, source boot, sequence,
      freshness and local monotonic lease rules.
- [x] Add attach mismatch, full queue, slow peer, torn value, restart, stale
      boot and crash-reclaim tests.
- [x] Measure zero hot allocations for proposal and odometry exchange.

Validation:

```bash
cargo test -p navigation-ipc -p middleware-shm
cargo clippy -p navigation-ipc --all-targets --all-features -- -D warnings
```

## 5. Motor-disabled Rust harness

- [x] Add and allowlist the `rs-navigation-e2e` composition root.
- [x] Wire lifecycle/mission dispatch and a synthetic test authority owner.
- [x] Receive proposal intents, independently materialize and authorize them,
      then apply the common Rust body-command limiter and mecanum projection.
- [x] Implement a bounded fake sink and deterministic SE(2) loopback that emits
      semantic odometry/TF through IPC.
- [x] Make motor-disabled mode compile-time/startup immutable.
- [x] Add a dependency-graph gate proving the binary contains no hardware,
      SocketCAN or motor-driver crate.
- [x] Add nominal, cancel, timeout, wrong-owner, expiry, stale/reordered,
      prior-boot and IPC-loss tests.

Validation:

```bash
cargo test -p rs-navigation-e2e
cargo tree -p rs-navigation-e2e
cargo run --locked -p architecture-check -- check
```

## 6. Isolated Rust ROS adapter

- [x] Add the nested, non-workspace ROS adapter source, `Cargo.toml.in`, exact
      lockfile and ROS source lock.
- [x] Add a safe build script that verifies the sourced Humble environment and
      installed Rust message trees before rendering a temporary manifest.
- [x] Build only offline/locked; never commit generated ROS Rust sources or an
      absolute `/opt/ros` path.
- [x] Implement one basic executor, NavigateToPose action mapping, raw Twist
      subscription, odometry/TF publication and IPC endpoints.
- [x] Bind ROS UUID to the canonical command ID and preserve complete
      acceptance/feedback/cancel/result semantics.
- [x] Gate raw proposals by active action, current authority, adapter boot and
      manifest identity; do not mint leases.
- [x] Expose bounded counters and typed readiness/fault snapshots without
      high-rate logging.
- [x] Add source-scan/build checks proving no hardware/application dependency
      enters the adapter.

Validation:

```bash
source /opt/ros/humble/setup.bash
platform/ros2-humble/navigation-bridge/build.sh --verify-only
platform/ros2-humble/navigation-bridge/build.sh --output <absent-file>
```

## 7. `chassis_nav` integration profile

- [x] Add a dedicated `rs_control_navigation.launch.py` and tuning overlay.
- [x] Start only map server, controller server, planner server, BT navigator and
      lifecycle manager.
- [x] Remove velocity smoother from this profile and remap controller output to
      the manifest-owned raw proposal topic.
- [x] Default inactive; require explicit manifest, expected digest, map,
      parameter and BT paths.
- [x] Validate exact files/digests/topics/frames and shut down the group when a
      critical process exits.
- [x] Keep Gazebo, RViz, scenario runner, online mapping and hardware drivers
      absent.
- [x] Add static tests for exact node/plugin/topic/publisher ownership and
      simulation-only map status.

Validation:

```bash
colcon test --packages-select chassis_nav_bringup chassis_nav_mecanum_navigation
colcon test-result --verbose
```

## 8. Deployment and supervision profile

- [x] Add checked example systemd services for the Nav2 process group, isolated
      adapter and motor-disabled Rust harness plus one integration target.
- [x] Encode startup ordering, bounded restart/backoff, hardening and explicit
      shutdown propagation without granting the Rust FSM process-spawn rights.
- [x] Place the complete integration slice in one accounting/cgroup boundary
      while retaining per-process metrics.
- [x] Validate units with `systemd-analyze verify` after replacing executable
      paths with inert host-test fixtures.
- [x] Prove a restarted adapter receives a new epoch and cannot reuse old SHM
      backlog or command state.

Rollback: disable only the integration target; existing platform and Nav2
standalone units remain unchanged.

## 9. Nominal and cancellation end-to-end loop

- [x] Start the motor-disabled Rust harness, isolated bridge and Nav2 profile in
      one owned process group/cgroup.
- [x] Validate matching runtime manifest/map/interface identities before
      activation.
- [x] Drive one accepted NavigateToPose goal to success using Rust-generated
      odometry/TF feedback.
- [x] Drive a second goal through request, cancel acceptance and terminal
      cancellation.
- [x] Prove proposals reach the fake sink only under the exact active lease and
      after the sole Rust final limiter.
- [x] Record command, wheel, odometry, action and authority correlations without
      logging every control tick.

## 10. Fault injection

- [x] Reject missing, wrong-owner, wrong-session and expired leases.
- [x] Reject malformed, duplicate, reordered, stale and prior-boot proposals.
- [x] Exercise the fail-together Nav2 process group and Rust adapter as real
      processes; cover producer/consumer SIGKILL, reclaim and restart for each
      IPC role in the shared-memory process tests.
- [x] Remove odometry/TF readiness and verify correlated terminal failure plus
      bounded stop.
- [x] Verify a new adapter epoch cannot replay the old goal or command cache.
- [x] Measure stop latency against the 100 ms proposal bound and retain typed
      fault/counter evidence.
- [x] Prove the Rust control harness remains alive for every Nav2/adapter fault.

## 11. Concurrent resource evidence

- [x] Add a non-overwriting host evidence collector with raw observations and
      SHA-256 manifest.
- [x] Run Nav2, Rust adapter, motor-disabled harness and the selected
      localization/SLAM workload concurrently.
- [x] Record per-process RSS/PSS, CPU, threads, context switches, page faults,
      deadlines and stop latency; keep cgroup current/peak explicitly
      unavailable when the unprivileged host cannot provide an owned cgroup.
- [x] Compare the integrated graph with the existing Nav2-only component
      observation without summing RSS as unique physical memory.
- [x] Report the run as host software evidence; leave AArch64, production SLAM
      and whole-robot qualification pending.

## 12. Architecture, CI and handoff

- [x] Preserve all pre-existing changes in both worktrees while adding root
      workspace members, dependencies and architecture classifications.
- [x] Run targeted formatting, warnings-denied Clippy, unit/integration tests,
      architecture policy, schema/interface checks and both repositories' full
      host CI.
- [x] Run the cross-repository manifest/digest check from clean build outputs.
- [x] Document deployment ordering, fault behavior, resource evidence and the
      exact boundary between host validation and physical qualification.
- [x] Keep this task active if production map, AArch64 timing, real sensors or
      physical motion remain required by a later delivery.

Validation:

```bash
./tools/ci.sh
# chassis_nav repository's applicable colcon/contract gate
```

## Host software completion note

The complete motor-disabled software scope passed on x86_64 with real Nav2,
including success, cancellation, Nav2 exit, bridge exit, localization loss and
IPC process-crash coverage. The task deliberately remains active only for the
later production map/sensor, AArch64 timing, privileged cgroup and physical
motion qualification gates; none can be inferred from this host evidence.
