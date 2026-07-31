# Cross-language algorithm and ROS 2 contract — Implementation plan

## 1. Preconditions and baseline

- [x] Load backend specs and pre-development checklist before editing code.
- [x] Run focused baseline tests for `proto-compat-check`,
      `middleware-zenoh` and `ros2-mapping`.
- [x] Confirm Python 3.10+, `uv`, vendored protoc and loopback TCP are
      available; do not probe camera, Orange Pi or hardware devices.

## 2. Phase A — Locked Python algorithm

- [x] Add the standalone Python project with exact Zenoh/Protobuf pins and a
      reproducible `uv.lock`.
- [x] Implement strict CLI parsing, canonical key inputs, per-start boot ID,
      monotonic sequence and structured readiness/error output.
- [x] Decode and validate odometry before computing the bounded SI
      heading-hold result.
- [x] Publish only `ChassisSetpointIntent`; keep local lease issuance and
      executable deadlines unrepresentable.
- [x] Document generation/run commands and the non-realtime/no-hardware
      boundary.

Validation:

```bash
uv sync --project examples/python-zenoh-algorithm --frozen
uv run --project examples/python-zenoh-algorithm --frozen \
  python -m py_compile examples/python-zenoh-algorithm/heading_hold.py
```

Rollback point: remove the example project; no canonical schema changes exist.

## 3. Phase B — Real cross-process qualification

- [x] Add an `integration`-classified test crate.
- [x] Generate temporary Python and C++ clients with vendored protoc.
- [x] Launch a Rust TCP Zenoh router plus the locked Python client and
      round-trip an odometry vector to a command intent.
- [x] Decode and validate the intent through `contract-codec`, `SourceGuard`
      and a pre-existing local lease.
- [x] Force-kill under continuing publications; assert nonblocking producer
      progress, absence of new intents and exclusive deadline expiry.
- [x] Restart with a new boot identity, explicitly authorize it and prove
      recovery.
- [x] Add AST/import-boundary and dependency-pin tests.

Validation:

```bash
cargo test -p cross-language-contract
cargo run -p proto-compat-check
```

Rollback point: remove the integration crate/workspace entries and example
without touching protocol crates.

## 4. Phase C — ROS runtime readiness/failure contract

- [x] Add explicit Zenoh version and standalone ros2dds deployment fields to
      the strict manifest schema and checked-in Humble profile.
- [x] Validate the supported 1.9.0 compatibility tuple and reject plugin or
      mismatched deployment values.
- [x] Add epoch-bound runtime evidence and deterministic route-admission
      evaluation.
- [x] Fail closed on adapter, native Zenoh, ROS graph/DDS or required ros2dds
      failure and on every identity mismatch.
- [x] Prove restart invalidates old evidence and that readiness exposes no
      lease-grant or deadline-extension operation.

Validation:

```bash
cargo test -p ros2-mapping
```

Rollback point: remove the runtime module and both additive manifest fields in
one change; restore the previous golden manifest fingerprint.

## 5. Phase D — Explicit route and safety verification

- [x] Strengthen the checked-in profile test for one sensor, command,
      service/query and action route.
- [x] Assert frame aliases, SI interface meanings, explicit bounded QoS,
      idempotent timeout, cancellation/result retention and command
      lease/staleness policy.
- [x] Add failure-matrix tests that pair closed bridge admission with existing
      local `CommandEnvelope` expiry checks.
- [x] Recompute and review the deterministic manifest fingerprint.

Validation:

```bash
cargo test -p ros2-mapping --test profile
cargo test -p ros2-mapping --test runtime_failure
```

## 6. Phase E — Quality and documentation

- [x] Run format, warnings-denied Clippy, architecture, Protobuf generation,
      Buf compatibility and dependency-policy gates.
- [x] Run the full software host gate. Platform-named host scripts may be
      skipped only if they would enter the user-excluded Orange Pi scope; any
      skip must be reported rather than treated as a pass.
- [x] Review the diff for generated artifacts, unbounded waits, ROS/hardware
      dependencies in core crates, Python embedding and accidental robot-YAML
      changes.
- [x] Update the executable backend spec with the cross-language process and
      runtime-evidence contracts.

Validation:

```bash
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
cargo test --locked --workspace --all-features
cargo run --locked -p architecture-check -- check
cargo run --locked -p proto-compat-check
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo deny check
```

## 7. Completion

- [x] Mark only evidence-backed task and parent Phase I2/I3 software
      checkboxes complete; leave camera/Orange Pi/live target qualification
      open.
- [ ] Commit implementation/task/spec records, archive the task, record the
      developer journal and push the branch.
