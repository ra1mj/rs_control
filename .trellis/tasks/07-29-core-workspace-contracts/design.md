# Core workspace and contracts — design

## Boundary

This child materializes the parent architecture without adding runtime behavior. Contract ownership is:

```text
upper -> middle -> lower
apps  -> integration config + any layer, wiring only
```

`config-model` is a deployment input model and has no dependency on a production layer. `robotctl` owns parsing and presentation. Later composition roots will convert a validated `RobotConfig` into narrow layer-owned startup settings.

## Initial crates

- `crates/lower/lower-core`: monotonic time, lifecycle, quality, frames, transport/device traits and lower errors.
- `crates/middle/message-model`: semantic headers, clock/quality, lease and command validation; may depend on lower types only where a mapping boundary genuinely requires it.
- `crates/middle/proto-generated`: Prost output and descriptor-set ownership.
- `crates/middle/contract-codec`: one mapping/validation owner between semantic models and generated Protobuf.
- `crates/integration/config-model`: strict YAML AST, topology validation, normalization and hash.
- `crates/integration/test-support`: empty fixture library used only by dev-dependencies.
- `crates/upper/control-core`: minimal compile-time proof of the upper class, depending only on middle.
- `apps/robotctl`: configuration-only composition root.
- `tools/architecture-check`: Cargo-metadata dependency-policy gate.
- `tools/proto-compat-check`: disposable C++/Python generation smoke test using the pinned `protoc`.

## Architecture policy

Store crate classifications and composition-root allowlist in a checked-in policy file. A Rust architecture-check executable invokes `cargo metadata --format-version 1`, follows normal/build dependency edges between workspace members, and rejects edges not allowed by the table. Dev dependencies are allowed only when the target crate is explicitly classified as `test-support`; production dependency kinds to it are rejected.

The checker also rejects unclassified workspace members, so adding a crate cannot silently bypass the policy. Fixture manifests test each forbidden edge without modifying the real workspace.

## Lower contracts

Hot-path contracts are synchronous and caller-buffered. A fixed-capacity `FrameBatch<F, const N: usize>` owns initialized frames without growing after construction. Trait documentation states whether calls may block; concrete realtime implementations must be non-blocking even though this base task has no OS transport.

Errors are structured enums. Missing data is represented as absence/staleness, never fabricated zero values. Monotonic timestamps use a strongly typed nanosecond newtype and do not imply synchronization between machines.

## Configuration

Serde uses `deny_unknown_fields`. Parsing and semantic validation are separate so diagnostics distinguish syntax from topology errors. Morphology profiles centrally own their required logical roles:

- `mecanum_4`: front-left, front-right, rear-left, rear-right.
- `omni_3`: front, rear-left, rear-right.
- `omni_4`: front-left, front-right, rear-left, rear-right.
- `swerve_4`: drive and steering IDs for front-left/front-right/rear-left/rear-right modules.
- arm `none`: no IDs. Other syntactically valid named profiles require at least one binding; their exact joint-role set is validated against the selected compiled upper-layer profile by the later composition root before hardware enable.

Motor IDs are unsigned 11-bit CAN identifiers in this configuration phase, with zero rejected to avoid broadcast/reserved ambiguity in later device profiles. Duplicate IDs are rejected within each device group; a later composition root also checks cross-group conflicts when chassis and arm share a physical bus. Normalization uses a canonical typed projection rather than serializing an unordered map directly. The SHA-256 digest covers UTF-8 normalized YAML bytes.

## Protobuf and semantic validation

Schemas use package `robot.v1`, stable field numbers, `UNSPECIFIED = 0` enum sentinels and explicit units in names/comments. `proto-generated/build.rs` runs Prost and emits one descriptor set. Vendored `protoc` avoids host-tool variance; Buf owns source lint/evolution policy.

Generated messages are wire DTOs, not the upper API. `message-model` owns semantic domain types; `contract-codec` performs fallible conversions and is the only owner of wire validation. A common header includes source ID, boot ID, sequence, source monotonic timestamp, publish Unix timestamp, validity, frame ID, clock status/uncertainty and quality.

## Rollback

No hardware or external state is changed. Rollback consists of reverting the newly created workspace files as one unit; architecture rules must never be weakened merely to make a forbidden dependency compile.
