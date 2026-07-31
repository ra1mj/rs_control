# Lease enforcement cache implementation

## 1. Fixed-capacity contract

- [x] Add `arrayvec`/`thiserror` production dependencies and allocation test
      support to `control-core`.
- [x] Add lease keys, events, entry states, effect/removal evidence and typed
      errors in a dedicated module.
- [x] Implement session, grant, renew, release, expiry, preempt and
      safe-confirm transitions.

## 2. Command enforcement

- [x] Authorize only exact active, current-session, unexpired authority.
- [x] Reject pending/released/expired/foreign/prior-session commands without
      mutating or refreshing state.
- [x] Preserve a single-owner synchronous API with bounded work.

## 3. Tests

- [x] Cover good/base/bad state transitions and capacity.
- [x] Cover two-phase takeover and exact typed removal evidence.
- [x] Cover interleaved competing sources, session loss and restart.
- [x] Measure zero hot-path allocations.

## 4. Verification

- [x] Run format, targeted tests and warnings-denied Clippy.
- [x] Run architecture check and `tools/ci.sh`.
- [x] Update executable lease specs and only proven parent G3 checkboxes.

## Validation commands

```bash
cargo fmt --all -- --check
cargo test --locked -p control-core
cargo clippy --locked -p control-core --all-targets -- -D warnings
cargo run --locked -p architecture-check -- check
tools/ci.sh
```
