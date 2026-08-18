# Host motion runtime and telemetry recording

## Goal

Complete the framework parent's remaining host-verifiable runtime and recording
work with a bounded motion composition root and a separate MCAP telemetry
recorder. Failure behavior must be testable without opening a CAN interface,
enabling a motor, claiming realtime qualification, or weakening architecture,
lease, and lifecycle contracts.

## Confirmed facts

- control-core owns allocation-free lifecycle, mission and local lease
  enforcement. middleware-local provides bounded SPSC latest and ordered ports
  with explicit overflow and disconnect results.
- message-model owns typed ComponentHealth and Fault. proto-generated exports
  the canonical robot-v1 Protobuf descriptor set.
- btctl and the motor-disabled navigation harness provide read-only bounded
  diagnostic snapshots, but no general motion composition root or persistent
  recorder exists.
- There is no motion-rt app, telemetry-recorder crate, MCAP dependency, or
  replay boundary in this workspace.
- The candidate dependency is crates.io mcap version 0.25.0. Its exact source,
  feature graph and license must pass the project dependency policy before use.
- This environment is Linux x86_64 host only. Orange Pi/RK3588 hardware,
  physical CAN topology, enabled motors, a realtime kernel, and target timing
  evidence are unavailable.

## Requirements

- HMR-01: Add an apps/motion-rt wiring-only composition root. Its host test
  mode uses synthetic I/O, periodic control, non-RT drain, and fake-sink roles.
  It adds no business policy and never bypasses control-core lease enforcement.
- HMR-02: After construction, the control worker uses fixed-capacity,
  preallocated communication and performs no filesystem, network, formatting,
  logging, allocator-growth, blocking receive, or device-I/O call. Host tests
  inject ticks and make no scheduling qualification claim.
- HMR-03: Only the current exact lease, epoch, sequence, and deadline can
  produce a fake-sink Apply decision. Expiry, stale input, owner mismatch,
  worker/bridge loss, shutdown, and diagnostic/recorder failure produce a
  bounded SafeStop and cannot refresh authority.
- HMR-04: Expose copyable fixed-capacity runtime diagnostics: tick/deadline
  counters, channel outcomes, availability, health/fault evidence, and
  safe-stop reason. Formatting and tracing export stay on the non-RT drain.
- HMR-05: Add a telemetry-recorder integration crate and optional app. It
  consumes bounded records and writes MCAP only outside the control path. Each
  record preserves original Protobuf bytes, topic, descriptor-set schema,
  sequence, source/publish/record timestamps, and typed metadata.
- HMR-06: Recording has byte/item bounds, priority classes, explicit gap
  records, chunking, rotation, quota, create-new output, and interrupted-output
  recovery. Full/failed storage is evidence, never producer backpressure.
- HMR-07: Replay validates MCAP/schema/channel metadata and yields ordered,
  bounded test inputs only. It rejects malformed/incompatible data and cannot
  construct a device or output command.
- HMR-08: Host tests cover thread separation, warm allocations, stale/expired
  command rejection, loss/failure, ordered shutdown, recorder recovery,
  quota/slow-storage degradation, and deterministic replay using only
  synthetic data and a fake sink.
- HMR-09: Register each new crate atomically in Cargo.toml and architecture.toml.
  motion-rt is an app composition root. The recorder remains integration-class
  and depends only on integration/protocol crates. No unsafe, reverse edge,
  generated-code edit, or broad device access is allowed.

## Acceptance criteria

- [ ] The host motion runtime proves separate synthetic I/O, control, drain,
      and fake-sink roles with bounded startup and deterministic join.
- [ ] Warm control ticks and diagnostic publication allocate nothing and avoid
      blocking/I/O/logging; stalled drain or recorder consumers cannot block
      the control producer.
- [ ] Exact lease/epoch/deadline validation reaches a fake sink only for a
      current command; every loss, expiry, shutdown, and failure path safely
      stops without creating authority.
- [ ] Runtime health/fault and diagnostic values are typed/fixed-capacity and
      observable on a non-RT renderer without parsing log text.
- [ ] MCAP output preserves original Protobuf bytes, descriptor schema,
      canonical channel metadata, sequence, and dual timestamps with explicit
      priority/gap behavior.
- [ ] Interrupted, quota-limited, and slow-storage cases preserve valid sealed
      output or recover it deterministically without control-path delay.
- [ ] Replay validates records and reproduces semantic order in tests without
      constructing a hardware command or output path.
- [ ] Warnings-denied Clippy, architecture, dependency/source policy, targeted
      tests, and the full host quality gate pass.

## Scope boundaries

- No SocketCAN open/transmit, motor enable, physical safe-stop test, camera,
  live ROS, Orange Pi/RK3588 profile selection, target benchmark, or realtime
  scheduling claim.
- No global tracing subscriber, network exporter, unbounded queue, allocation,
  or disk I/O is added to a control worker.
- This task records already-encoded protocol evidence. It adds no Protobuf
  schema and never turns the recorder into a command authority.
