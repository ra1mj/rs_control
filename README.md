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

The current implementation is Phase A only. It contains core contracts, strict robot topology configuration, the Protocol Buffers v1 baseline and an executable dependency-policy gate. It does **not** yet open CAN devices, enable motors or produce torque.

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

The first hardware target remains Orange Pi 5 Plus/RK3588 with a separately qualified Ubuntu 22.04 + Rockchip 5.10 PREEMPT_RT kernel. Hardware instructions and the DM-J4310-2EC driver arrive in later, separately gated tasks.
