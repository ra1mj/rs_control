# Cross-Layer Robot Data Flow

## Start with the executable path

For a value that crosses layers, write down the full path before editing:

```text
hardware sample
  -> hardware_module validated transport/device type
  -> protocol_module semantic message
  -> local port and/or contract-codec Protobuf bytes
  -> middleware ingress validation
  -> application_module algorithm
  -> leased/deadlined semantic command
  -> protocol_module port
  -> hardware_module driver
```

Composition roots may construct both ends, but do not own the validation,
safety, codec, or algorithm behavior between them.

## Boundary ownership

| Boundary | Owner | Required questions |
|---|---|---|
| robot YAML -> startup values | `config-model`, then the composition root | Are unknown keys rejected? Is only narrow immutable data passed onward? |
| raw CAN -> device sample/fault | transport plus device driver | Which clock proves freshness? Are raw error details preserved? |
| semantic Rust -> Protobuf | `contract-codec` | Are missing/unknown values rejected and units/frame documented? |
| local port -> concrete backend | `middleware-api` plus one backend | Are latest/full/disconnected, ownership return, counters, and allocation bounds preserved? |
| Zenoh input -> accepted sample/command | `middleware-zenoh` ingress | Are source, boot, sequence, time, validity, quality, lease, and deadline checked? |
| health/fault -> operator/recorder | semantic health types plus non-RT drain | Can software decide safely without parsing text? Is raw evidence retained? |

Source references:

- `crates/protocol_module/contract-codec/src/mapping.rs`
- `crates/protocol_module/middleware-zenoh/src/ingress.rs`
- `crates/hardware_module/motor-damiao/src/driver.rs`
- `crates/integration/config-model/src/lib.rs`
- `crates/integration/ros2-mapping/src/projection.rs`

## Contract-change checklist

Before implementation:

- [ ] Identify the single semantic owner and every representation of the value.
- [ ] Name the clock domain, units, coordinate frame, identity, sequence, and
      freshness rules that apply.
- [ ] Decide where invalid input is rejected and which typed error/fault
      reaches the caller.
- [ ] Trace disconnect, timeout, overflow, process death, and restart behavior.
- [ ] Confirm the change does not add allocation, formatting, blocking, file
      I/O, or a network call to an RT-visible operation.

After implementation:

- [ ] Add accepted round-trip and rejected malformed/unknown/stale cases.
- [ ] Test restart/epoch behavior when identity or ordering matters.
- [ ] Verify all consumers use the canonical semantic type/decoder.
- [ ] After a Protobuf descriptor change, search every checked-in descriptor
      SHA/fingerprint consumer and update it only after the old-golden breaking
      gate passes.
- [ ] Run the architecture check and full host gate.
- [ ] Keep target-only acceptance open until immutable physical evidence exists.

## Common failures

- A generated Protobuf DTO reaches `application_module`, so zero/unknown enum
  values become another layer's problem.
- A diagnostic realtime or hardware timestamp is compared directly with
  `MonoTime`.
- Robot YAML gains device paths, CAN layouts, gains, or deployment policy.
- An application imports `transport-socketcan`, `middleware-shm`, or
  `middleware-zenoh` instead of consuming a protocol-module port.
- A network reconnect refreshes an old command, boot ID, lease, or deadline.
- A host benchmark is described as an RK3588/realtime qualification result.

The relevant executable contracts are in
`../backend/robot-contracts.md`, and dependency direction is enforced by
`architecture.toml`.
