# rs_control

Rust-owned robot control framework with compile-time layer boundaries:

```text
upper algorithms / FSM
          |
          v
middle ports / Zenoh / Protobuf
          |
          v
lower transports / device protocols
```

The implementation contains the Phase A contracts plus a host-verified Orange Pi
5 Plus platform candidate. The platform code pins the PREEMPT_RT kernel inputs,
models exact CPU/IRQ roles, prepares classic SocketCAN without transmitting and
produces immutable timing evidence. It is **not hardware-qualified yet** and does
not enable motors or produce torque.

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

The first hardware target is Orange Pi 5 Plus/RK3588 with Ubuntu 22.04 and the
pinned Rockchip 5.10 PREEMPT_RT baseline. See the
[platform runbooks](platform/orangepi5plus/README.md) for the guarded build,
bring-up, eight-hour qualification and rollback flow. The target-board evidence
gate and the later DM-J4310-2EC driver remain separately gated tasks.
