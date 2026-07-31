# Motor feedback contract implementation

## 1. Semantic contract

- [x] Add `TemperatureObservation` and `MotorState` to `message-model`.
- [x] Validate all finite fields and add construction/getter tests.
- [x] Keep identifiers fixed-capacity and the type copyable.

## 2. Protobuf and canonical codec

- [x] Add the explicit additive temperature representation to
      `device.proto`.
- [x] Implement semantic/wire conversion and byte encode/decode in
      `contract-codec`.
- [x] Test raw, calibrated, legacy, missing, ambiguous, and non-finite cases.
- [x] Run Buf formatting, lint, breaking against the old golden, and
      `proto-compat-check`.

Rollback point: do not regenerate the golden descriptor until every old-golden
compatibility check passes.

## 3. Hardware sample mapping

- [x] Add the allowed `contract-codec -> motor-damiao` dependency.
- [x] Add the validated mapping context and allocation-free mapping function.
- [x] Test enabled/disabled/known-fault/unknown-status behavior and exact
      identity, timing, sequence, physical values, raw temperatures, quality,
      and fault-code results.

## 4. Integration and review

- [x] Regenerate `proto/golden/robot-v1.binpb` deterministically.
- [x] Run targeted format, Clippy, and tests for `message-model`,
      `contract-codec`, `proto-generated`, and `proto-compat-check`.
- [x] Run `cargo run --locked -p architecture-check -- check`.
- [x] Run `tools/ci.sh`.
- [x] Mark only parent Phase D2 hardware-sample mapping complete.

## Validation commands

```bash
cargo fmt --all -- --check
cargo clippy --locked -p message-model -p contract-codec --all-targets -- -D warnings
cargo test --locked -p message-model -p contract-codec -p proto-generated
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo run --locked -p proto-compat-check
cargo run --locked -p architecture-check -- check
tools/ci.sh
```
