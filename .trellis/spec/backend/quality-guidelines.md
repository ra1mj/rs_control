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
- Dependency edges among `hardware_module`, `protocol_module` and
  `application_module` that are not allowed by `architecture.toml`.
- Hand-editing generated Protobuf Rust or bypassing `contract-codec` with local DTO parsing.
- Hardware-changing tests that run without explicit fixture/operator opt-in.

## Testing Requirements

- New validation rule: one accepted base/good case and one rejected edge/bad case.
- New wire field: semantic round trip plus malformed, missing and unknown-enum cases where applicable.
- New dependency class/edge: architecture-policy positive case and rejection fixture.
- New CLI behavior: process-level integration test with exit status and stable stdout/stderr assertions.
- Realtime/hardware behavior: host unit tests are necessary but never sufficient; target-board stress and safety gates remain mandatory.
- A process-backed fault test that launches sibling copies of its own test
  binary and exercises kernel-lock reclamation serializes its parent scenarios
  with a test-local mutex, while leaving the child-only entrypoint ungated.
  It uses a unique private temporary namespace and reaps every child before
  another scenario begins. Do not weaken lock assertions or hide this class of
  race behind an unbounded retry.

## Review Checklist

- Data is validated exactly once at its owning entry boundary.
- Semantic objects cannot be mutated into an invalid state after construction.
- Normalized configuration and schema outputs are deterministic.
- No new unbounded queue, hidden allocation or blocking operation enters a declared RT path.
- All new crates are classified and no application becomes a business-logic backdoor.
