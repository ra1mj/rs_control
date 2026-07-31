# Network input fallback implementation

## 1. Backend-neutral decision contract

- [x] Add borrowed cache and decision types to `middleware-api`.
- [x] Implement the complete fallback decision table with exclusive expiry.
- [x] Keep accepted sample identity opaque and reject cached prior-boot data.
- [x] Document that the function is pure and never refreshes cache state.

## 2. Tests

- [x] Cover current, held, unavailable and safe-stop results for all boundaries.
- [x] Prove zero allocations on the complete decision matrix.
- [x] Extend the Zenoh fault matrix with explicit accepted delay/jitter,
      exact expiries and unchanged fault/counter coverage.

## 3. Verification

- [x] Run format, targeted tests and Clippy.
- [x] Run the architecture check.
- [x] Run `tools/ci.sh`.
- [x] Mark only the two remaining parent D3 items complete.

## Validation commands

```bash
cargo fmt --all -- --check
cargo test --locked -p middleware-api -p middleware-zenoh
cargo clippy --locked -p middleware-api -p middleware-zenoh --all-targets -- -D warnings
cargo run --locked -p architecture-check -- check
tools/ci.sh
```
