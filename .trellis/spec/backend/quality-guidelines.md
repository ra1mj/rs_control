# Rust Quality Guidelines

## Required Host Gate

Run:

```bash
tools/ci.sh
```

It verifies formatting, Clippy with warnings denied, all workspace tests, the real dependency graph, C++/Python Protobuf generation, Buf formatting/lint/breaking checks, dependency advisories/licenses/sources and every checked-in robot YAML example.

Individual commands use the pinned `rust-toolchain.toml`, committed `Cargo.lock`, `tools/bufw` checksum pin and vendored `protoc`.

## Forbidden Patterns

- `unsafe` Rust unless the architecture task explicitly adds and reviews a contained FFI boundary; the workspace currently forbids it.
- `dbg!`, `todo!` and `unimplemented!` in production code.
- Wildcard-version path dependencies; internal versions and paths belong in `[workspace.dependencies]`.
- A production dependency on `test-support`.
- Lower/middle/upper dependency edges not allowed by `architecture.toml`.
- Hand-editing generated Protobuf Rust or bypassing `contract-codec` with local DTO parsing.
- Hardware-changing tests that run without explicit fixture/operator opt-in.

## Testing Requirements

- New validation rule: one accepted base/good case and one rejected edge/bad case.
- New wire field: semantic round trip plus malformed, missing and unknown-enum cases where applicable.
- New dependency class/edge: architecture-policy positive case and rejection fixture.
- New CLI behavior: process-level integration test with exit status and stable stdout/stderr assertions.
- Realtime/hardware behavior: host unit tests are necessary but never sufficient; target-board stress and safety gates remain mandatory.

## Review Checklist

- Data is validated exactly once at its owning entry boundary.
- Semantic objects cannot be mutated into an invalid state after construction.
- Normalized configuration and schema outputs are deterministic.
- No new unbounded queue, hidden allocation or blocking operation enters a declared RT path.
- All new crates are classified and no application becomes a business-logic backdoor.
