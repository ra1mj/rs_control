# Rust Robot Framework — Implementation Plan

## 1. Execution strategy

This task is the parent architecture/integration task. Implementation should be split into independently verifiable child tasks after the user approves these planning artifacts. Do not start the parent as a monolithic implementation task.

Proposed child task map:

| Order | Child | Owns | Depends on |
|---|---|---|---|
| 1 | `core-workspace-contracts` | Cargo workspace, layer APIs, YAML schema, Protobuf rules, dependency gate | approved parent design |
| 2 | `orangepi-rt-platform` | Ubuntu/BSP/PREEMPT_RT image, CAN overlay, IRQ/CPU/thermal qualification | approved parent design |
| 3 | `damiao-socketcan-driver` | SocketCAN lower adapter and DM-J4310-2EC driver | 1; hardware setup from 2 |
| 4 | `zenoh-protobuf-middleware` | middle API, local fast path, codec, Zenoh key/QoS semantics | 1 |
| 5 | `single-motor-vertical-slice` | `motion-rt`, safe enable/control/feedback, leases, timing | 2, 3, 4 |
| 6 | `observability-recording` | typed health/faults, tracing drain, metrics, MCAP/rolling buffer | 4, 5 |
| 7 | `fsm-mission-orchestration` | lifecycle HSM, authority leases, mission DSL/runtime, replay tests | 4, 5, 6 |
| 8 | `chassis-control` | three morphologies, estimator, velocity/heading/SE(2) tracking | 5, 7 |
| 9 | `perception-ros2-bridge` | USB/MIPI boundary, isolated perception process, optional ROS 2 bridge | 4, 6; platform media support from 2 |
| 10 | `system-integration` | full acceptance, deployment packages, failure matrix, docs | all required children |

Dependencies above must be copied into each child `prd.md`/`implement.md`; tree position alone is not a dependency declaration.

## 2. Global gates

Every child must satisfy applicable gates before the next dependent child starts:

1. Layer dependency direction remains `upper -> middle -> lower`.
2. No production code in `motion-rt` uses unbounded queues or blocks an RT thread.
3. All external payloads have a single decoder/validator owner.
4. All hardware-changing tests start disabled and require an explicit fixture/operator opt-in.
5. Every change that affects control timing reruns the target-board stress profile.
6. Every Protobuf change runs lint, breaking-change and cross-language tests.
7. Every failure path produces a typed fault and a deterministic safe action.
8. No milestone is accepted only from desktop/x86 tests when ARM hardware is in scope.

Baseline repository checks once the workspace exists:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo deny check
buf lint
buf breaking --against <pinned-main-or-golden-schema>
```

Use project-owned wrappers for target-board tests so they record board revision, kernel hash, patch hash, device tree, governor, temperature, CPU/IRQ affinity and application build ID.

## 3. Phase A — Core workspace and contracts

### A1. Scaffold the workspace

- [ ] Add pinned Rust toolchain, workspace dependency policy and reproducible build metadata.
- [ ] Create lower, middle, upper, integration and app directories from `design.md`.
- [ ] Define allowed internal dependency edges in a machine-readable policy.
- [ ] Add a CI/script check based on `cargo metadata` that rejects reverse or skipped-layer edges.
- [ ] Configure formatting, Clippy, tests, dependency advisories/licenses and minimal supported Rust policy.

Validation:

- [ ] Deliberately injected `upper -> lower`, `lower -> middle`, `middle -> upper` and production-layer -> `test-support` dependencies are rejected by the architecture test.
- [ ] Composition-root exceptions are limited to named `apps/*` crates and checked to contain wiring only.

Rollback: remove only newly scaffolded crates; do not weaken the dependency policy to make a bad edge compile.

### A2. Define lower contracts

- [ ] Define transport frame/batch, device lifecycle, monotonic time, sample quality and typed lower errors.
- [ ] Keep interfaces synchronous/allocation-free for realtime implementations.
- [ ] Document ownership, thread-safety and blocking guarantees for every trait method.
- [ ] Add compile-fail or trait-contract tests where useful.

### A3. Define the minimal YAML

- [ ] Implement the exact v1 topology schema: chassis morphology/IDs, arm morphology/IDs, IMU transport/baud.
- [ ] Reject unknown keys, duplicate IDs, missing morphology roles and invalid baud values.
- [ ] Hash the normalized configuration and expose the hash in runtime metadata.
- [ ] Add `robotctl config validate` and `robotctl config print-normalized`.
- [ ] Keep deployment and protocol details out of this YAML.
- [ ] Parse the full YAML only in a composition root, then pass lower/middle/upper narrowly typed immutable startup settings; no production layer imports `config-model`.

Validation:

```bash
cargo test -p config-model
cargo run -p robotctl -- config validate config/robots/example.yaml
```

### A4. Define Protobuf and middle contracts

- [ ] Create `robot.v1` schemas and the common header.
- [ ] Define device state, control setpoint/action/ack, health/fault, lease, FSM and perception messages.
- [ ] Generate Rust via Prost and C++/Python fixtures with pinned `protoc`/Buf versions.
- [ ] Commit descriptor-set/golden compatibility input.
- [ ] Add round-trip and malformed-input tests.
- [ ] Add SI unit and coordinate-frame documentation next to schemas.

Validation:

```bash
buf format --diff --exit-code
buf lint
buf breaking --against proto/golden/robot-v1.binpb
cargo test -p proto-generated -p contract-codec -p message-model
```

Rollback: publish additive corrections within v1; if semantics were already released incompatibly, create `v2` rather than reinterpreting v1 fields.

## 4. Phase B — Orange Pi 5 Plus realtime platform

### B1. Reproducible image and kernel

- [ ] Pin Orange Pi/Rockchip 5.10 BSP source commit and U-Boot/device-tree sources.
- [ ] Pin a matching PREEMPT_RT patch and document all conflict resolutions.
- [ ] Store kernel config fragments for full preemption, high-resolution timers, SocketCAN, CAN error reporting, cgroups/systemd and required media/USB drivers.
- [ ] Build installable kernel packages/image from a clean Ubuntu 22.04 build host.
- [ ] Record source hashes and artifact checksums.

Validation:

- [ ] Clean rebuild produces the expected artifact hashes or a documented reproducible-difference report.
- [ ] Board boots, reports the intended RT configuration and retains network/USB/storage access.

Rollback: retain a known bootable vendor kernel entry and serial-console recovery path. Kernel rollback must not overwrite user data.

### B2. Board and CAN bring-up

- [ ] Fix Orange Pi board revision, power supply, active cooler, isolated CAN transceiver, termination and wiring diagram.
- [ ] Enable one CAN controller using the pinned device-tree overlay.
- [ ] Configure classic CAN 1 Mbps and error-frame monitoring using an idempotent systemd helper.
- [ ] Verify loopback/analyzer traffic before connecting the motor.
- [ ] Record CAN IRQ number and CPU topology/capacity.

Safety gate:

- [ ] Verify physical E-stop/enable path before any torque-producing test.
- [ ] Mount the motor and use a current-limited supply.
- [ ] Initial commands are zero torque/low gain and require an operator confirmation flag.

### B3. Realtime qualification harness

- [ ] Pin `rt-tests` and create a wrapper that records complete environment metadata.
- [ ] Measure baseline and stressed wakeup latency under CPU, memory, network, storage and NPU/GPU load.
- [ ] Tune CPU governor, RT runtime settings, CPU affinity, IRQ affinity, memory locking and thermal policy.
- [ ] Define numerical pass/fail outputs for the 1 kHz and 500 Hz profiles.
- [ ] Run long-duration thermal soak; a throttled result is a failure, not a qualified result.

Validation target:

- [ ] 1 kHz: maximum measured wakeup jitter <= 100 us and zero application deadline misses during the agreed soak window.
- [ ] If failed, explicitly qualify 500 Hz with the same stress matrix and retain the failure evidence.

Rollback: restore the last qualified affinity/governor/kernel bundle as a unit.

## 5. Phase C — SocketCAN and DM-J4310-2EC

### C1. SocketCAN adapter

- [ ] Implement non-blocking RX/TX, filters, timestamp extraction and error frames.
- [ ] Preallocate frame batches and expose queue/overflow/error metrics.
- [ ] Implement bus-off detection with safe, explicit recovery rules.
- [ ] Implement the classic-CAN budget calculator and 70% default gate.
- [ ] Unit-test frame/filter/error parsing using captured fixtures without hardware.

### C2. DM protocol codec

- [ ] Encode/decode MIT mode from official documentation/captured reference frames.
- [ ] Implement static DM-J4310-2EC limits and explicit saturation/rejection.
- [ ] Decode status, position, velocity, torque, MOS/rotor temperature and error codes.
- [ ] Implement enable, disable, zero command and fault reset.
- [ ] Implement read/verify of ESC_ID, MST_ID, mode and timeout; keep persistent writes maintenance-only.
- [ ] Use exhaustive boundary/property tests for float-to-integer mappings.

Validation:

```bash
cargo test -p transport-socketcan -p motor-damiao
```

- [ ] Compare encoded/decoded frames with official tooling or a known-good CAN capture.
- [ ] Fuzz decoders with arbitrary eight-byte frames; no panic or out-of-bounds behavior.

### C3. Safe hardware driver lifecycle

- [ ] Start disabled and verify IDs/profile before enable.
- [ ] Enforce feedback freshness and motor communication timeout.
- [ ] Require explicit arm/enable authorization.
- [ ] Confirm Ctrl-C/process exit, lease expiry and process kill lead to motor timeout stop.
- [ ] Test temperature/fault/bus-off reporting without automatic unsafe re-enable.

Rollback: disable motor, remove torque, preserve CAN capture and restore known motor parameters; never bulk-write flash registers during rollback.

## 6. Phase D — Middle local path and Zenoh

### D1. Local fast path

- [ ] Implement interchangeable latest-value and ordered-event channel interfaces.
- [ ] Prototype at least two candidate mechanisms if benchmark evidence is not decisive.
- [ ] Preallocate and prefault all RT-visible buffers.
- [ ] Define overflow/latest-wins behavior and counters per channel.
- [ ] Prove a blocked/crashed consumer cannot stop the producer.

Target-board benchmark:

- [ ] one-way latency distribution and maximum;
- [ ] jitter under contention;
- [ ] CPU cycles/load;
- [ ] allocation count (zero after arm);
- [ ] overflow and consumer-stall behavior.

### D2. Codec and Zenoh adapter

- [ ] Implement the canonical key builder/parser in one crate.
- [ ] Map lower samples to message model and Protobuf once at the middle boundary.
- [ ] Implement stream, action/ack, query/queryable and liveliness interaction classes.
- [ ] Assign semantic QoS classes without allowing Zenoh calls on RT threads.
- [ ] Validate source/boot/sequence/time/validity/quality for all network input.
- [ ] Mirror selected local samples from a non-RT thread.

### D3. Network fault tests

- [ ] Inject delay, jitter, loss, duplicate, reorder, disconnect, reconnect and publisher restart.
- [ ] Verify expired/prior-boot/prior-lease commands are rejected.
- [ ] Verify the 1 kHz local thread never waits on the Zenoh executor.
- [ ] Verify network sensor fallback/stop policies are deterministic.

## 7. Phase E — Single-motor vertical slice

- [ ] Build `motion-rt` as a wiring-only composition root.
- [ ] Create separate CAN I/O, 1 kHz control, Zenoh bridge and diagnostic-drain threads.
- [ ] Apply realtime policy, affinity, stack prefault and memory lock after all startup allocation.
- [ ] Implement one safe position/velocity target path using a valid resource lease.
- [ ] Publish motor state, command ack, health and timing metrics.
- [ ] Add systemd units with least privilege and watchdog reporting.

Vertical acceptance sequence:

```text
Boot disabled
-> validate config/kernel/CAN/motor IDs/mode/timeout
-> grant local test lease
-> enable
-> command bounded low-energy motion
-> observe feedback via local path and Zenoh
-> expire/revoke lease
-> verify safe stop and disable
-> replay MCAP evidence
```

- [ ] Repeat with Zenoh disconnected, FSM killed, recorder blocked, CPU/NPU/network/disk stress and CAN faults.
- [ ] Pass target-board 1 kHz gate or explicitly select the qualified 500 Hz profile.

Rollback: deploy the last known motor-disabled build and revoke all control leases.

## 8. Phase F — Observability and recording

### F1. Typed health/faults

- [ ] Implement shared lifecycle and fault messages without creating a reverse dependency.
- [ ] Add per-layer health adapters and freshness metrics.
- [ ] Ensure safety logic consumes typed codes, never log text.

### F2. Non-blocking diagnostics

- [ ] Instrument Rust code with structured `tracing` spans/events.
- [ ] Route RT diagnostics through bounded compact records.
- [ ] Export formatting/OpenTelemetry only from non-RT workers.
- [ ] Test exporter outage and log storms.

### F3. MCAP recorder

- [ ] Write Protobuf bytes, descriptor sets, channel metadata, sequence and dual timestamps.
- [ ] Add chunking, index, rotation, quota and crash recovery.
- [ ] Implement data priority/drop policy and explicit gap records.
- [ ] Implement rolling bulk-data buffers and fault/manual freeze triggers.
- [ ] Build deterministic replay into middle test inputs without claiming a virtual device implementation.

Validation:

- [ ] Kill recorder mid-write and recover all valid completed data.
- [ ] Fill disk/slow writes do not add RT deadline misses.
- [ ] Replayed FSM/control events reproduce the same projections.

## 9. Phase G — FSM, leases and mission DSL

### G1. HSM library spike

- [ ] Prototype required HSM transitions in `statig` and a minimal direct-enum reference.
- [ ] Compare allocation, binary size, transition visibility, graph generation, async leakage, cancellation and test ergonomics.
- [ ] Record the selection decision; do not carry both implementations into production.

### G2. Lifecycle and restart safety

- [ ] Implement explicit transition table and exhaustive event handling.
- [ ] Implement `Boot` motion inhibit and live health reconstruction.
- [ ] Require fresh explicit arm after every restart.
- [ ] Make E-stop globally preemptive and latched until explicit physical/software reset sequence.
- [ ] Generate state graph and transition report in CI.

### G3. Resource leases

- [ ] Implement grant, renew, release, expire and preempt events.
- [ ] Implement `motion-rt` enforcement cache and session epochs.
- [ ] Test all command sources concurrently; no last-writer-wins behavior.
- [ ] Test FSM loss and stale authority after restart.

### G4. Mission DSL/runtime

- [ ] Implement typed nodes and immutable validated IR.
- [ ] Implement deterministic single-thread event reducer.
- [ ] Implement correlated action acknowledgements and cancellation scopes.
- [ ] Validate resource conflicts, unreachable nodes, unbounded retries/loops and missing timeouts.
- [ ] Create one representative automatic project using sequence, branch, parallel, timeout, retry and cancellation.

Validation:

- [ ] Transition-table tests cover every state/event combination.
- [ ] Property tests cover event duplication, loss, reorder, timeout and restart.
- [ ] Event-log replay is deterministic.
- [ ] Kill/restart during every active state; never auto-resume motion.

## 10. Phase H — Chassis control

### H1. Shared models

- [ ] Define SI/body-frame types, constraints and explicit feasibility/saturation results.
- [ ] Implement common fixture vectors and invariants for all morphologies.

### H2. Kinematics

- [ ] Implement mecanum-4 forward/inverse kinematics.
- [ ] Implement omni-3/omni-4 forward/inverse kinematics.
- [ ] Implement swerve-4 forward/inverse kinematics, steering optimization and rate limits.
- [ ] Validate singular/near-singular, saturation, reversal and zero-speed behavior.

### H3. State estimation and control

- [ ] Time-align wheel/IMU/external-pose inputs and propagate quality.
- [ ] Implement 2D wheel+IMU estimator with external correction port.
- [ ] Implement body velocity and heading control.
- [ ] Implement timestamped SE(2) trajectory tracking.
- [ ] Define loss/degradation behavior for wheel, IMU and external pose inputs.

Hardware gate:

- [ ] Start with lifted/unloaded chassis and bounded outputs.
- [ ] Validate each morphology on its actual configured hardware before ground motion.
- [ ] Repeat timing/safety tests with the full motor count and measured CAN load.

## 11. Phase I — Perception and ROS 2

### I1. Perception boundary

- [ ] Implement USB camera path first where it reduces BSP risk; then qualify Orange Pi MIPI path.
- [ ] Use V4L2/DMABUF handles where supported and measure copies.
- [ ] Isolate inference from capture and motion processes.
- [ ] Publish typed observation/target messages with clock/quality metadata.

### I2. Cross-language example

- [ ] Generate C++ and Python Protobuf clients.
- [ ] Build one isolated example algorithm process using only Zenoh/Protobuf.
- [ ] Kill/restart it under load; motion remains safe.

### I3. ROS 2 bridge

- [ ] Pin a compatible standalone `zenoh-bridge-ros2dds` release.
- [ ] Implement explicit mapping for one sensor topic, one command, one service/query and one action.
- [ ] Validate SI units, coordinate frames, QoS, cancellation, lease and stale-command behavior.
- [ ] Verify bridge/DDS failure cannot bypass local safety.

## 12. Phase J — Full integration and release gate

- [ ] Run all repository checks and target-board suites from a clean checkout/image.
- [ ] Execute the complete failure/recovery matrix in `design.md`.
- [ ] Run thermal and concurrent CPU/NPU/network/storage soak.
- [ ] Validate process watchdogs, restart rate limiting and startup ordering.
- [ ] Validate MCAP decode/replay with the released descriptor set.
- [ ] Verify minimal YAML examples for all three chassis morphologies and an arm profile.
- [ ] Produce wiring, operator safety, bring-up, configuration, deployment, monitoring and rollback documentation.
- [ ] Capture software bill of materials, toolchain, dependency locks, kernel/firmware/config hashes and benchmark evidence.
- [ ] Perform final cross-layer review: source sample -> lower -> middle -> upper -> command -> lower -> acknowledgement -> record/replay.

Final acceptance is against the parent PRD, not merely the last child task.

## 13. Deferred work

- RDK X5 platform qualification.
- Generic lower TCP/UDP device transports.
- PTP/hardware timestamp/TSN deterministic Ethernet profile.
- Production mTLS, identity provisioning and Zenoh ACLs.
- Virtual devices, `vcan` simulation loop and Gazebo integration.
- Runtime visual/YAML mission editor targeting the validated mission IR.
- Additional concrete motor, IMU, lidar and camera protocols.
