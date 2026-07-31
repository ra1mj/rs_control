# Cross-language algorithm and ROS 2 contract

## Goal

Complete the host-testable software slice of parent Phase I2/I3: run an
independent Python algorithm over the canonical Zenoh/Protobuf contract, prove
that killing and restarting it cannot prolong motion authority, and turn the
existing ROS 2 mapping profile into an explicit runtime readiness/failure
contract.

## Requirements

- **CLR-01:** The example algorithm runs as a standalone Python process. It is
  not embedded in or dynamically loaded by `motion-rt`, an FSM, a driver or any
  Rust library.
- **CLR-02:** The Python process depends only on the generated `robot.v1`
  Protobuf modules, the pinned Zenoh Python binding and the Python standard
  library. It must not import repository Rust bindings, ROS 2, a hardware SDK
  or an application-specific transport shim.
- **CLR-03:** C++ and Python code generation continues to originate from the
  canonical `proto/robot/v1` files. Generated client artifacts are disposable
  build outputs, not a second checked-in schema.
- **CLR-04:** The example consumes canonical odometry, calculates a bounded
  heading-hold command intent in SI units and publishes only a
  `robot.v1.ChassisSetpointIntent`. A local process must still validate source,
  boot, sequence, active lease and local monotonic deadline before the intent
  becomes executable.
- **CLR-05:** Process identity includes a per-start boot ID and monotonic
  sequence. The test may inject deterministic identities, but the normal
  executable generates a new boot ID at every start.
- **CLR-06:** Dependencies are exact-version locked. The Python Zenoh version,
  Rust Zenoh version and optional standalone `zenoh-bridge-ros2dds` version
  must belong to one explicit compatibility family.
- **CLR-07:** A real TCP loopback test starts the Rust Zenoh side and Python
  process, transmits generated Protobuf payloads in both directions, validates
  the result with the Rust semantic codec and local command-authority logic,
  then force-kills and restarts the Python process.
- **CLR-08:** While the Python process is absent, the local producer remains
  nonblocking, no command deadline or lease is renewed, and the last accepted
  command becomes unusable at its exclusive local deadline. Restart requires a
  newly authorized boot identity.
- **CLR-09:** The checked-in ROS 2 profile explicitly selects a standalone
  ros2dds executable and pins compatible Zenoh/ros2dds versions. Dynamic plugin
  mode is rejected by the profile validator.
- **CLR-10:** Runtime evidence is matched to the checked-in profile,
  Protobuf descriptor fingerprint, route-manifest fingerprint and bridge build
  identity before ROS ingress is considered ready. A restart clears readiness
  until identity is revalidated.
- **CLR-11:** Adapter absence, ROS graph/DDS failure, or optional ros2dds
  failure closes command/action ingress and makes affected observation/query
  paths unavailable or degraded. No failure state can mint a lease, extend an
  existing command deadline or reuse stale readiness.
- **CLR-12:** Host tests explicitly cover one sensor topic, one guarded
  command, one bounded service/query and one cancellable action, including SI
  units, frames, QoS, cancellation, lease and stale-command behavior.
- **CLR-13:** Camera capture, inference hardware, Orange Pi work, physical
  motion and live ROS/DDS/target-board qualification remain out of scope. Host
  evidence must not be reported as hardware or live-deployment qualification.

## Acceptance Criteria

- [x] C++ and Python Protobuf generation passes from the canonical schemas,
      and the Python algorithm imports only generated Protobuf, Zenoh and the
      standard library.
- [x] An exact Python dependency lock pins `eclipse-zenoh` and a compatible
      Protobuf runtime; the Rust and ros2dds profile pins remain compatible.
- [x] A real Rust-to-Python TCP test consumes odometry and returns a
      semantically valid, bounded chassis command intent.
- [x] The returned intent becomes a command only through an already-active
      local lease and a locally constructed deadline.
- [x] Force-kill/restart under ongoing publications proves producer progress,
      exclusive deadline expiry, changed boot identity and successful
      post-authorization recovery.
- [x] The ROS 2 manifest rejects non-standalone ros2dds mode or an incompatible
      Zenoh/ros2dds version pair.
- [x] Runtime readiness rejects stale/mismatched profile, descriptor, manifest
      and bridge identities, and must be re-established after restart.
- [x] Tests cover sensor, command, service/query and action policies plus
      frame/SI/QoS/cancel/lease/staleness behavior.
- [x] Adapter, DDS and ros2dds failure-matrix tests close remote ingress and
      prove local lease/deadline enforcement is unchanged.
- [x] Targeted checks and the repository quality gate pass without using
      camera, Orange Pi or physical hardware.

## Constraints and dependencies

- Depends on completed tasks `07-30-zenoh-protobuf-middleware` and
  `07-30-zenoh-ros2-contract`.
- The canonical wire package remains `robot.v1`; this task does not add or
  reinterpret Protobuf fields.
- `middleware-zenoh` and `contract-codec` remain the Rust transport and
  validation owners. The Python example is deliberately outside those crates.
- Live ROS 2 and DDS execution may be added later as an environment-qualified
  gate; its absence does not weaken the host-level fail-closed contract and is
  not disguised as live evidence.
