# HSM selection spike implementation

## 1. Shared fixture and tool boundary

- [x] Add `hsm-spike` as an architecture-classified, non-production tool.
- [x] Define shared lifecycle states, events, fixed-capacity effects and
      projection traits.
- [x] Define valid, invalid, duplicate, restart and global-preemption traces.

## 2. Candidate prototypes

- [x] Implement the exhaustive direct-enum reducer.
- [x] Pin and implement synchronous no-macro `statig` traits with every
      optional feature disabled.
- [x] Prove identical state/effect projections, including ordered
      `Running` cancellation and global E-stop.

## 3. Inspectability and resource evidence

- [x] Generate deterministic transition reports and DOT evidence without
      hiding duplicated metadata.
- [x] Measure warmed zero-allocation dispatch for both candidates.
- [x] Build separate release binaries and record toolchain, feature tree and
      file/text/data size.
- [x] Compare hierarchy, transition visibility, async leakage, cancellation
      and test ergonomics.

## 4. Decision and verification

- [x] Record a scored selection and explicit rejection/trade-off reasons.
- [x] Update the executable FSM spec with the selected strategy.
- [x] Run format, targeted tests, warnings-denied Clippy and architecture
      policy.
- [x] Run `tools/ci.sh` and mark only parent G1 complete.

## Validation commands

```bash
cargo fmt --all -- --check
cargo test --locked -p hsm-spike
cargo clippy --locked -p hsm-spike --all-targets -- -D warnings
cargo run --locked -p architecture-check -- check
cargo tree --locked -p hsm-spike -e features
cargo build --locked --release -p hsm-spike --bins
tools/hsm-spike/measure.sh
tools/ci.sh
```
