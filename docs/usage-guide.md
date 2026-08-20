# rs_control Usage Guide

[中文版本](usage-guide.zh-CN.md)

This guide is the operational entry point for `rs_control`. It covers supported
developer and operator workflows, then directs target-specific work to the
authoritative runbooks. It does not add a motor-enablement procedure or claim
that a host result qualifies an Orange Pi, a CAN link, a motor, or a realtime
profile.

## 1. Scope and Safety Classes

Read the class of a workflow before running its commands.

| Class | What you may do | What it proves |
|---|---|---|
| Host-safe/read-only | Build, test, validate configuration, run the arm simulator, render behavior trees, inspect existing diagnostic IPC, or inspect platform plans. | Software and contract behavior on the current host only. |
| Target-only, motor-disarmed | Build/install the pinned kernel, read board state, prepare a candidate profile, configure an analyzer-only CAN link, and collect qualification evidence. | Only the evidence bundle and its reviewed verdict can qualify the exact target configuration. |
| Physical safety gate | Use a real motor, motion power, a physical stop path, or a safety fixture. | Nothing in this repository currently closes this gate by itself. Follow the owning acceptance procedure. |

The checked-in Orange Pi profile is a **candidate**. `PROFILE_QUALIFIED=0` and
`QUALIFICATION_EVIDENCE_SHA256=unqualified` mean that it must not be used by a
production preparation service. A failed 1 kHz run is retained as evidence; it
does not silently select 500 Hz.

## 2. Repository Orientation and Prerequisites

All commands below assume a Linux development environment and a clean source
checkout. Start at the repository root so that relative paths resolve as
documented:

```bash
cd "$(git rev-parse --show-toplevel)"
```

The workspace uses the committed `rust-toolchain.toml` and `Cargo.lock`. Install
Rust through the project's normal `rustup` policy before invoking Cargo. The
complete host gate also uses `uv` for the checked Python example. Docker is
needed only for the optional reproducible Orange Pi kernel build. No host quick
start command requires `sudo`, changes `/sys` or `/proc`, configures a network
link, or sends CAN traffic.

The dependency direction is intentionally one-way:

```text
application_module algorithms, control, FSM
                  |
                  v
protocol_module    messages, local ports, Zenoh, Protobuf
                  |
                  v
hardware_module    transports and device protocols
```

Composition roots in `apps/` may wire approved layers together. Application
logic must not bypass the protocol boundary to open a device backend.

## 3. Host Quick Start

### 3.1 Validate a Robot Configuration

`robotctl` is configuration-only. It does not open a device, configure a CAN
link, or start motion.

```bash
cargo run --locked -p robotctl -- \
  config validate config/robots/example.yaml

cargo run --locked -p robotctl -- \
  config print-normalized config/robots/example.yaml
```

The first command prints a stable SHA-256 identity. The second prints the
canonical YAML form. Validation failures are intentional: correct the input
instead of editing generated output or weakening the schema.

### 3.2 Check Layer Boundaries

```bash
cargo run --locked -p architecture-check -- check
```

This verifies the Cargo dependency policy. It is a host-safe static check and
does not demonstrate scheduler, I/O, CAN, or motor behavior.

### 3.3 Verify the Arm Simulation

```bash
tools/run-arm-simulation.sh verify
```

This re-runs the deterministic articulated-arm simulation and byte-compares the
checked-in evidence report. To generate a separate report, provide an absent
path; an existing report is verified instead of overwritten:

```bash
tools/run-arm-simulation.sh qualify /absolute/path/new-arm-report.json
```

The simulation exercises prepared kinematics, dynamics, gravity feed-forward
and bounded payload identification. It opens no hardware or ROS endpoint and
does not qualify a physical arm, controller gains, realtime timing, or payload
limits. See [Arm Payload Identification and Adaptive Gravity
Compensation](arm-payload-identification.md) for its acceptance and rollback
rules.

### 3.4 Render the Behavior-Tree Example

```bash
cargo run --locked -p btctl -- example
```

This constructs the checked behavior-tree example and replays deterministic
startup events. It does not open IPC, ROS, an actuator, or a hardware boundary.
The runtime is event-driven: its embedding owner supplies deadlines and I/O
results; the tree itself does not form a control loop or grant authority.

### 3.5 Run the Full Host Gate

```bash
tools/ci.sh
```

This is the repository's complete host-side gate. It runs platform fixture
tests, formatting, Clippy with warnings denied, workspace tests, architecture
and Protobuf compatibility checks, simulation evidence verification, Buf
checks, dependency policy checks, and every checked-in robot YAML validation.

It intentionally does **not** fetch/build/install a kernel, write target
kernel state, configure a real network link, transmit CAN frames, enable a
motor, or establish ARM/realtime qualification.

## 4. Robot YAML

Robot YAML selects topology only. A minimal mecanum example is:

```yaml
schema_version: 1
chassis:
  morphology: mecanum_4
  motor_ids:
    front_left: 1
    front_right: 2
    rear_left: 3
    rear_right: 4
arm:
  morphology: none
  motor_ids: {}
imu:
  transport: uart
  baud_rate: 921600
```

Supported checked examples are in `config/robots/`: `example.yaml`,
`omni_3.yaml`, `omni_4.yaml`, and `swerve_4.yaml`. The parser rejects unknown
fields, invalid morphology roles, duplicate/out-of-range IDs, unsupported
schema versions, invalid transport, and invalid baud rates.

Do not put CAN frame layout, byte order, checksums, gains, device paths, Zenoh
settings, realtime tuning, or executable expressions in this file. Those
belong to reviewed code and platform configuration, not robot topology.

## 5. Behavior-Tree and Navigation Observation

### 5.1 Observe Existing Navigation IPC

The motor-disabled navigation composition publishes latest diagnostic snapshots
for behavior-tree participation and transport counters. `btctl` only observes
them; it cannot obtain a lease, refresh a command deadline, or create output.

```bash
cargo run --locked -p btctl -- inspect \
  --ipc-root /absolute/path/to/navigation-ipc

cargo run --locked -p btctl -- observe \
  --ipc-root /absolute/path/to/navigation-ipc \
  --plain --samples 1
```

`--ipc-root` must be an existing absolute directory. Missing, stale, malformed,
or incompatible snapshots are reported as unavailable, never as a healthy
zero-value state. Add `--interval-ms 100 --samples 20` for a bounded,
machine-readable observation stream. See [Project-native behavior
tree](behavior-tree.md) for all observer options and runtime semantics.

### 5.2 Run the Motor-Disabled Navigation Self-Test

The checked navigation manifest explicitly declares `motor_output: forbidden`,
`can_transmit: forbidden`, `simulation_only: true`, and
`approved_for_production: false`. Derive its required digest from the exact
file rather than hand-copying it:

```bash
nav_manifest=config/navigation/rs_control_navigation_plugin.yaml
nav_digest=$(sha256sum "$nav_manifest" | awk '{print $1}')

cargo run --locked -p rs-navigation-e2e -- \
  --manifest "$nav_manifest" \
  --expected-sha256 "$nav_digest" \
  self-test
```

The self-test runs one synthetic authority/proposal/stop path against a fake
sink. Its expected output includes `motor_output=forbidden`. It is not a ROS 2
deployment or a physical navigation test.

For the isolated bridge, Nav2 profile, service units, host evidence collection,
and fault modes, follow [Motor-disabled chassis navigation
plugin](navigation-plugin.md) and
[the deployment runbook](../platform/ros2-humble/navigation-deployment/README.md).
Those procedures retain the motor-disabled boundary.

## 6. Motion Runtime and Telemetry Recording

`apps/motion-rt` currently provides a host-only composition library and an
empty executable entry point. There is no production `motion-rt` command-line
operation to document or run. Do not infer an operator workflow from the host
systemd example.

`HostMotionRuntime` uses fixed-capacity local channels, injected control ticks,
a fake motor-disabled sink, typed safe-stop decisions, and a non-realtime
diagnostic drain. Loss of an expected bridge, control worker, recorder, or
diagnostic consumer leaves only the fail-closed fake output path in host tests.
The example service at `apps/motion-rt/systemd/motion-rt-host.service` pins
`RS_CONTROL_MOTOR_MODE=disabled`; it is a least-privilege host example, not a
target deployment unit.

The `telemetry-recorder` crate is also a library boundary, not a CLI. Integrate
it only from a non-realtime owner:

1. Create a bounded `TelemetryRecord` from original encoded Protobuf bytes.
   Topics are printable ASCII and at most 96 bytes; payloads are at most
   4,096 bytes; source sequence is nonzero.
2. Call `TelemetryIngress::try_push` from the producer. It has separate fixed
   capacity for `Critical`, `Control`, and `Bulk` records and reports drops as
   gaps instead of waiting for a consumer.
3. On a non-realtime drain worker, open `TelemetryRecorder` with an existing
   absolute output directory and immutable byte/quota/chunk limits; call
   `drain_into`, then `seal` at the recording boundary.
4. Replay only a sealed `.mcap` through `ReplayPolicy` and `replay_file` with
   the compiled descriptor digest, an explicit topic allowlist, and an input
   bound. Replay returns validated inputs and cannot construct output commands.

The recorder uses create-new staging output, rotating bounded MCAP files,
manifest sealing, recovery of complete staging, and explicit degradation on
storage or quota pressure. It preserves original payload bytes and records gap
metadata. A recorder failure must never block a realtime producer or create
motion authority.

## 7. Orange Pi/RK3588 Target Handoff

### 7.1 Safe Host-Side Platform Checks

These commands validate checked-in inputs and fixture behavior. They do not
mutate a board or create qualification evidence:

```bash
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/kernel/build-kernel.sh --print-plan
platform/orangepi5plus/scripts/rt-affinity.sh plan
platform/orangepi5plus/scripts/can-link.sh plan
platform/orangepi5plus/tests/host-test.sh
platform/orangepi5plus/tests/middleware-host-test.sh
platform/orangepi5plus/scripts/build-horus-adapter.sh --verify-only
```

### 7.2 Target-Only Preconditions

Before any privileged target action, prepare the real Orange Pi 5 Plus with a
recorded board revision, stable power, active cooling, serial-console recovery,
retained vendor-kernel boot path, isolated 3.3 V CAN transceiver, correct
two-endpoint termination, and an analyzer or non-motor loopback peer. Remove
motion power and keep motors disconnected for platform bring-up.

The board's TX/RX header pins are logic signals, not CANH/CANL. Do not wire them
directly to a CAN bus. The candidate `0-5 / 6 / 7` CPU role map is not an
approval to pin a different host or to use a "first core that works" fallback.

The target flow is deliberately staged:

1. [Build and install the pinned kernel](../platform/orangepi5plus/docs/build-and-install.md)
   with serial recovery retained.
2. [Bring up the board and analyzer-only CAN link](../platform/orangepi5plus/docs/bring-up.md).
   `rt-affinity.sh apply` and disruptive CAN operations require explicit
   `--confirm-disarmed`; production preparation additionally requires
   `--require-qualified`.
3. Run the [realtime qualification procedure](../platform/orangepi5plus/docs/realtime-qualification.md)
   with a unique absent output directory and filled hardware manifest.
4. Independently review and retain raw output, nested manifests, and the
   top-level SHA-256 before any profile promotion. Use the
   [rollback procedure](../platform/orangepi5plus/docs/rollback.md) after a
   tuning, CAN, or boot failure.

A complete concurrent result needs the exact target identity, pinned source and
build IDs, CPU/IRQ/governor readback, memory lock policy, CPU/memory/storage,
real Ethernet, accelerator, thermal/frequency observations, and at least an
eight-hour duration. A 1 kHz pass requires maximum wakeup jitter at most
100 microseconds and no application deadline misses during the agreed soak.
Missing inputs or a short research run are `incomplete`, not passing evidence.

Middleware selection has the same standard. It compares local and SHM paths
with the pinned external Horus adapter across latest/ordered semantics, payload
sizes, and normal/slow/stopped/killed peers. `qualify-middleware.sh` refuses a
wrong architecture, board, clean-source identity, locking/FIFO/perf preflight,
or pre-existing output. A valid sealed `fail` or `incomplete` bundle may verify
its internal integrity, but only `qualification_status=pass` is eligible for a
profile decision.

## 8. Physical Gates, Failure Response, and Related Documentation

No command in this guide enables torque, transmits a motor frame, proves
hardware watchdog behavior, or verifies a physical stop. Real SocketCAN and
DM-J4310-2EC work remains a separately owned, restrained procedure with a
safety fixture, current limits, explicit authorization, and immutable target
evidence.

On a target tuning, CAN, thermal, or boot failure: keep motion physically
disarmed, preserve the evidence directory, do not reuse a pass digest for a
changed board/kernel/device tree/profile/build, and follow the target
[rollback and evidence retention](../platform/orangepi5plus/docs/rollback.md)
procedure. Do not turn `fail` or `incomplete` output into a candidate profile.

Use these documents for the detailed procedure owned by each area:

| Topic | Authoritative document |
|---|---|
| Platform overview and safe host checks | [Orange Pi 5 Plus / RK3588 platform baseline](../platform/orangepi5plus/README.md) |
| Kernel build and guarded installation | [Pinned kernel build and guarded installation](../platform/orangepi5plus/docs/build-and-install.md) |
| Board, overlay, and CAN bring-up | [Orange Pi 5 Plus bring-up](../platform/orangepi5plus/docs/bring-up.md) |
| Timing and middleware evidence | [Realtime qualification](../platform/orangepi5plus/docs/realtime-qualification.md) |
| Target failure recovery | [Rollback and evidence retention](../platform/orangepi5plus/docs/rollback.md) |
| Behavior-tree semantics | [Project-native behavior tree](behavior-tree.md) |
| Motor-disabled navigation integration | [Motor-disabled chassis navigation plugin](navigation-plugin.md) |
| Arm payload identification | [Arm Payload Identification and Adaptive Gravity Compensation](arm-payload-identification.md) |

For an exact CLI synopsis, use the checked executable or script help rather
than guessing flags:

```bash
cargo run --locked -p robotctl -- --help
cargo run --locked -p btctl -- --help
cargo run --locked -p rs-arm-simulation -- --help
cargo run --locked -p rs-navigation-e2e -- --help
platform/orangepi5plus/scripts/rt-affinity.sh plan --help
platform/orangepi5plus/scripts/can-link.sh plan --help
```
