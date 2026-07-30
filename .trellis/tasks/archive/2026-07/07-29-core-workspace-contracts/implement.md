# Core workspace and contracts — implementation

## 1. Workspace gate

- [x] Add pinned toolchain, root workspace, shared dependency pins and lint profiles.
- [x] Scaffold the initial crate set and `robotctl`.
- [x] Add architecture policy, checker and invalid-graph fixtures.
- [x] Verify the real graph and all rejection fixtures.

## 2. hardware_module contracts

- [x] Implement strongly typed monotonic time, lifecycle and sample quality.
- [x] Implement fixed-capacity frame batches and synchronous transport/device contracts.
- [x] Add boundary and lifecycle tests plus API documentation.

## 3. Minimal YAML

- [x] Implement strict Serde input model and semantic validation.
- [x] Centralize morphology role requirements and ID validation.
- [x] Implement canonical normalization and SHA-256 hashing.
- [x] Add valid/invalid fixtures and `robotctl` commands.

## 4. Protobuf and protocol_module contracts

- [x] Add Buf configuration and `robot.v1` schemas.
- [x] Generate Prost types and descriptor set reproducibly.
- [x] Smoke-test standard C++ and Python generation with the same pinned `protoc`.
- [x] Implement protocol_module semantic model and the sole wire codec/validator.
- [x] Add round-trip, unknown-enum and malformed/invalid semantic tests.

## 5. Verification

```bash
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
cargo test --locked --workspace --all-features
cargo run --locked -p architecture-check -- check
cargo run --locked -p proto-compat-check
cargo run --locked -p robotctl -- config validate config/robots/example.yaml
cargo run --locked -p robotctl -- config print-normalized config/robots/example.yaml
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
```

`tools/bufw` pins and verifies Buf for Linux x86_64/aarch64. Rust and compatibility generation use the vendored `protoc` and do not depend on a global installation.
