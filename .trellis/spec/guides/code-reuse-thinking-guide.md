# Code Reuse and Contract Ownership

## Search before adding

Use `rg` before introducing a constant, helper, validation rule, error
variant, or conversion. Search both the name and the semantic value:

```bash
rg -n "MAX_CLASSIC_CAN_DATA_LEN|0x07ff" crates
rg -n "decode_.*header|Header::new" crates/protocol_module
rg -n "ChannelCounters|try_publish|try_receive" crates
```

The goal is not minimum line count. The goal is one owner for every safety or
wire rule, with adapters reusing that owner.

## Existing ownership patterns

| Concern | Canonical owner | Reuse pattern |
|---|---|---|
| CAN frame/ID/time/lifecycle contracts | `crates/hardware_module/hardware-module-core/src/lib.rs` | transports and drivers consume validated fixed-storage types |
| DM MIT wire/register rules | `crates/hardware_module/motor-damiao/src/{codec,register}.rs` | the driver calls pure codecs; it does not duplicate bit layout |
| Semantic robot messages | `crates/protocol_module/message-model/` | constructors validate once; consumers receive valid values |
| Protobuf conversion | `crates/protocol_module/contract-codec/` | generated DTOs stop at this boundary |
| Middleware semantics | `crates/protocol_module/middleware-api/` | local, SHM, and Zenoh code implement/adapt the same outcomes and counters |
| Robot topology YAML | `crates/integration/config-model/src/lib.rs` | composition roots parse once and pass narrow startup settings |
| Crate dependency policy | `architecture.toml` | `architecture-check` enforces the single declared graph |

Prefer extending these owners over creating a convenience copy in an
application, adapter, or composition root.

## When to extract

Extract shared code when at least one is true:

- two implementations must have identical safety or compatibility behavior;
- a repeated parser/codec could otherwise accept different inputs;
- a constant is part of a wire, timing, capacity, or hardware contract;
- backend-neutral code can live in the existing API/semantic owner without
  importing a concrete runtime.

Keep code local when it is one backend's mechanism or when extraction would
create a forbidden dependency. Similar-looking code in hardware and protocol
layers is not reusable if sharing it would move domain logic into a generic
utility crate.

## Forbidden shortcuts

- Do not create a catch-all `common` crate to bypass `architecture.toml`.
- Do not decode Protobuf or reinterpret generated enum integers outside
  `contract-codec`.
- Do not duplicate CAN bit layouts or motor scaling in the driver and codec.
- Do not let an application match concrete local/SHM/Zenoh error types when
  `middleware-api` already owns the semantic outcome.
- Do not infer shared-memory payload safety from `needs_drop`, size, or a Rust
  type name; `middleware-shm` owns the explicit layout contract.
- Do not share a formatted error string when callers need a typed fault,
  counter, or lifecycle transition.

## Change checklist

- [ ] Search found the current owner and all consumers.
- [ ] The helper lives in the lowest valid owning layer, not the lowest
      technically reachable crate.
- [ ] Existing callers use the shared owner after the change.
- [ ] Good/base/bad cases exercise the common implementation.
- [ ] `cargo run --locked -p architecture-check -- check` still accepts the
      graph.
- [ ] `tools/ci.sh` passes.
