# rs_control

Rust-owned robot control framework with compile-time layer boundaries:

```text
application_module algorithms / FSM
                  |
                  v
protocol_module ports / Zenoh / Protobuf
                  |
                  v
hardware_module transports / device protocols
```

The implementation contains the Phase A contracts, a host-verified Orange Pi 5
Plus platform candidate, a bounded Linux SocketCAN adapter, and a pure
DM-J4310-2EC MIT codec/read-only discovery/lifecycle driver. Default tests use
fake I/O and never transmit motor frames. The platform and motor path are **not
hardware-qualified yet** and do not establish physical disable, watchdog,
firmware compatibility, or realtime timing evidence.

## Configuration

The robot YAML intentionally contains only:

- chassis morphology and logical motor IDs;
- arm morphology and logical motor IDs;
- IMU transport and baud rate.

Protocol frames, controller tuning, device paths, Zenoh deployment and realtime scheduling do not belong in this file.

```bash
cargo run -p robotctl -- config validate config/robots/example.yaml
cargo run -p robotctl -- config print-normalized config/robots/example.yaml
```

## Contract and architecture checks

```bash
cargo run -p architecture-check -- check
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
tools/ci.sh
```

`tools/bufw` pins and verifies the Buf CLI for Linux x86_64 and aarch64. Prost uses a vendored `protoc`, so Rust schema generation does not depend on a host installation.

The motor-disabled `chassis_nav` integration, isolated ROS 2 bridge and host
qualification boundary are documented in
[docs/navigation-plugin.md](docs/navigation-plugin.md).

## Arm simulation and adaptive gravity compensation

The repository includes a deterministic articulated-arm host simulation that
exercises prepared FK/IK, RNEA/ABA dynamics, nominal gravity compensation and
bounded payload identification. Run it from any working directory with:

```bash
tools/run-arm-simulation.sh qualify [output-path]
tools/run-arm-simulation.sh verify
tools/run-arm-simulation.sh test
```

`qualify` performs the fixed-step simulation and writes a canonical report only
when the destination does not exist (the optional path defaults to
`evidence/arm-adaptive-gravity-simulation.json`). If the destination already
exists, it performs the same read-only byte comparison as `verify` instead of
overwriting evidence. `verify` always checks the checked-in report. `test` runs
the focused algorithm, control and simulation suites.

This simulation is host-only. It opens no hardware or ROS endpoint, produces
no motor frame, and does not qualify physical-arm accuracy, controller
stability, realtime timing or safe payload limits. The complete calibration,
activation, gating and rollback process is in
[docs/arm-payload-identification.md](docs/arm-payload-identification.md).

The first hardware target is Orange Pi 5 Plus/RK3588 with Ubuntu 22.04 and the
pinned Rockchip 5.10 PREEMPT_RT baseline. See the
[platform runbooks](platform/orangepi5plus/README.md) for the guarded build,
bring-up, eight-hour qualification and rollback flow. The target-board evidence
gate and the restrained-motor/analyzer DM-J4310-2EC qualification remain
separately gated tasks.
