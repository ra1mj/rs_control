# Core workspace and contracts

## Goal

Deliver Phase A of the approved robot framework: a compiling Rust workspace with enforceable layer boundaries, allocation-conscious hardware_module interfaces, a minimal validated robot YAML, and the canonical Protocol Buffers v1 contract baseline.

## Dependencies

- Parent task: `07-29-rust-robot-framework`.
- Normative inputs: the approved parent `prd.md`, `design.md`, and Phase A of `implement.md`.
- No hardware or earlier child task is required. Hardware-changing behavior is explicitly outside this child.

## Requirements

- C1: Pin the Rust toolchain and centralize workspace dependency versions, lints, formatting and release profiles.
- C2: Establish `hardware_module`, `protocol_module`, `application_module`, `integration`, and `apps` crate classes with the exact allowed dependency directions from the parent design.
- C3: Add a machine-readable architecture policy and a deterministic checker over Cargo metadata that rejects reverse, skipped-layer and test-support production dependencies.
- C4: Keep application crates as named composition roots; this child may add only `robotctl`, whose behavior is limited to configuration inspection.
- C5: Define synchronous hardware-module time, lifecycle, quality, transport, frame-batch, device-driver and typed-error contracts without async traits or hidden allocation requirements.
- C6: Make the v1 robot YAML contain only schema version metadata plus chassis morphology/motor IDs, arm morphology/motor IDs, and IMU transport/baud rate.
- C7: Reject unknown YAML fields, unsupported schema versions, invalid chassis morphologies/arm profile identifiers/transports/baud rates, missing or extra compiled chassis roles, invalid `none` arm bindings, duplicate IDs and out-of-range IDs. Exact non-`none` arm roles are checked later against the selected compiled application-module profile before arming.
- C8: Produce deterministic normalized YAML and a SHA-256 configuration hash from the validated model.
- C9: Provide `robotctl config validate` and `robotctl config print-normalized`; neither command may access hardware.
- C10: Define `robot.v1` Protobuf schemas for common headers, device state, control/lease/action acknowledgements, health/faults, FSM/mission events and perception observations.
- C11: Generate Rust types with Prost from one schema owner, emit a descriptor set, enforce basic Protobuf evolution rules through Buf, and smoke-test standard C++/Python binding generation without adding non-Rust business code.
- C12: Define protocol_module-owned semantic Rust types and validation for source identity, boot identity, sequence, timestamps, validity, clock status, quality and command leases.
- C13: Document SI units and `x` forward, `y` left, `z` up coordinate conventions beside the schemas.
- C14: Do not add Zenoh runtime wiring, SocketCAN, motor code, realtime threads, chassis algorithms, FSM execution, ROS 2 or hardware setup in this child.

## Acceptance Criteria

- [x] `cargo fmt --all -- --check`, workspace Clippy with warnings denied, and all workspace tests pass.
- [x] The architecture checker passes the real graph and its fixture tests prove rejection of `application_module -> hardware_module`, `hardware_module -> protocol_module`, `protocol_module -> application_module`, and production use of `test-support`.
- [x] hardware_module contracts compile and their tests demonstrate fixed-capacity frame batches, explicit lifecycle transitions and typed error propagation.
- [x] Valid example YAML for every supported chassis morphology parses, validates, normalizes and hashes deterministically.
- [x] Negative YAML tests cover unknown fields, unsupported version, chassis role mismatch, invalid arm profile identifiers/`none` bindings, duplicate/out-of-range IDs, invalid transport and invalid baud rate.
- [x] `robotctl config validate config/robots/example.yaml` succeeds and `print-normalized` is stable across repeated runs.
- [x] All `.proto` files pass `buf lint`; Rust code and a descriptor set are generated, while pinned `protoc` successfully emits standard C++ and Python bindings into disposable test directories.
- [x] Protobuf round-trip tests preserve identity, sequence, time, quality and lease fields; malformed or semantically invalid inputs are rejected at the protocol_module boundary.
- [x] `cargo metadata` proves no production layer imports `config-model`; its only current consumer is the allowlisted `robotctl` composition root, and later runtime roots must translate the full document into narrower layer-owned settings.

## Out of Scope

- Target-board, PREEMPT_RT, SocketCAN and DM-J4310-2EC implementation.
- Zenoh transport and realtime in-process channel implementations.
- Committing or compiling C++/Python bindings and implementing non-Rust business modules; this child performs generation compatibility smoke tests only.
- Hardware access or torque-producing tests.
