# Zenoh Protobuf middleware — Implementation plan

## 1. Preconditions and baselines

- [x] Load `trellis-before-dev` and the backend directory, robot-contract,
      error-handling and quality specs before editing production code.
- [x] Re-read this task's PRD/design and parent phase D1-D3; do not expand into
      mmap shared memory or target qualification.
- [x] Record a clean baseline for protocol tests, architecture policy,
      Protobuf golden compatibility and dependency policy.
- [x] Preserve unrelated untracked Trellis/bootstrap files and user changes.

Validation:

```bash
cargo test --locked -p message-model -p contract-codec -p proto-generated
cargo run --locked -p architecture-check -- check
cargo run --locked -p proto-compat-check
cargo deny check
```

Rollback point: no production edits before the baseline is understood.

## 2. Phase A — Workspace and backend-neutral API

- [x] Create `middleware-api`, `middleware-local` and `middleware-zenoh` under
      `crates/protocol_module/`; add exact workspace dependencies and classify
      all three in `architecture.toml`.
- [x] Implement fixed-capacity concrete key construction/parsing and named
      selector construction with exhaustive domain/version validation.
- [x] Implement endpoint direction/interaction/QoS/bounds/source/fallback
      descriptors and route-set duplicate/compatibility validation.
- [x] Define latest-value and ordered-event publisher/subscriber traits,
      outcomes, generation metadata and saturating counter snapshots without
      exposing a backend type.
- [x] Add accepted base/good tests and invalid/wildcard/version/duplicate/
      zero-bound/bad-combination tests.
- [x] Extend architecture fixtures for the new allowed protocol edges and
      forbidden application-to-backend/Zenoh dependency shapes if not already
      covered generically.

Validation:

```bash
cargo test --locked -p middleware-api
cargo run --locked -p architecture-check -- check
```

Rollback point: remove all three workspace/architecture entries together if
the public port/key contract needs redesign before another crate consumes it.

## 3. Phase B — Additive command-intent protocol

- [x] Add `CommandIntent`, `ChassisSetpointIntent` and `ActionGoalIntent` to
      `proto/robot/v1/control.proto` using unused field numbers and explicit
      requested-TTL/local-deadline comments.
- [x] Add validated semantic intent values to `message-model` without changing
      existing executable `CommandEnvelope`, `ChassisSetpoint` or `ActionGoal`
      behavior.
- [x] Implement authority-tuple matching and local materialization through
      `CommandEnvelope::from_bridge_ingress` with route TTL and lease-expiry
      capping.
- [x] Add `contract-codec` conversions for intent messages and malformed,
      missing, oversized, zero-TTL and unknown-enum/identity tests.
- [x] Prove a sender absolute monotonic deadline is absent from the intent and
      cannot influence the materialized local deadline.
- [x] Run Buf breaking against the old golden descriptor before updating the
      committed golden artifact, then verify Rust/C++/Python generation.

Validation:

```bash
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo test --locked -p message-model -p contract-codec -p proto-generated
cargo run --locked -p proto-compat-check
```

Rollback point: before publication, revert the additive messages and semantic
types together. Never reinterpret the existing local deadline field.

## 4. Phase C — Safe local backend

- [x] Pin and document the `waitfree-sync` and `rtrb` source/API/license/
      Rust-version review; keep their types private to `middleware-local`.
- [x] Wrap the triple buffer as nonclonable latest publisher/subscriber
      endpoints for reviewed `Copy` RT values.
- [x] Wrap `rtrb` as ordered event endpoints; validate nonzero capacity and
      preserve rejected-value ownership on full/disconnected results.
- [x] Add liveness, channel generation, full/overwrite/disconnect results and
      saturating counters with one-writer ownership.
- [x] Verify endpoint drops, a paused consumer and a full event queue cannot
      block producer calls.
- [x] Add dev-only steady-state allocation tests after construction/warm-up and
      multi-thread contention/accounting tests.
- [x] Record both host-selected primitives as target comparison candidates;
      do not claim either wins the RK3588 benchmark.

Validation:

```bash
cargo test --locked -p middleware-local
cargo clippy --locked -p middleware-api -p middleware-local --all-targets --all-features -- -D warnings
```

Rollback point: retain `middleware-api`; replace only the private local
implementation if source audit, allocation or contention evidence fails.

## 5. Phase D — Ingress guards and bridge core

- [x] Implement bounded route/source state with expected source, current boot,
      retired boots, last sequence and availability.
- [x] Implement strict duplicate, reorder, unexpected-source, unapproved-boot,
      retired-boot and bounded-restart exhaustion outcomes.
- [x] Implement synchronized-age-required and reception-bounded freshness
      policies plus `ReceivedSample::usable_at` local expiry checks.
- [x] Implement exact local lease/session/owner/resource matching before a
      command intent is materialized.
- [x] Define bounded bridge/action/query state and typed counters for size,
      decode, freshness, source, authority, queue, timeout and transport
      failures.
- [x] Pass local monotonic reception time explicitly and use scripted ingress
      frames for deterministic delay/loss/duplicate/reorder/restart/disconnect
      tests.

Validation:

```bash
cargo test --locked -p middleware-zenoh ingress
cargo test --locked -p middleware-zenoh fault
```

Rollback point: guards are pure and backend-neutral inside the adapter; stop
before opening real sessions if any fault case can accept stale authority.

## 6. Phase E — Zenoh 1.9 adapter

- [x] Add exact `zenoh = 1.9.0` with default, unstable and shared-memory
      features disabled; enable only `transport_tcp` for the reviewed
      explicit-router trusted-LAN and loopback profiles.
- [x] Convert project keys to checked Zenoh keys only after project validation.
- [x] Implement dedicated-thread bridge startup/readiness, bounded callback
      inboxes, cooperative shutdown and final counters.
- [x] Implement latest/state pub-sub pumps with pre-decode size checks and
      `contract-codec` conversion.
- [x] Implement bounded action command/ack retry/correlation and terminal
      lifecycle handling outside RT.
- [x] Implement bounded query/queryable zero/one-reply semantics and typed
      remote/multiple/timeout/size results; decode replies only through the
      codec boundary.
- [x] Implement liveliness token/subscription mapping to availability while
      keeping boot authorization as an explicit guarded transition.
- [x] Map semantic QoS to stable priority/drop controls without presenting
      transport reliability as application completion.
- [x] Ensure a session/worker failure marks routes unavailable and never
      refreshes a cached sample or lease.

Validation:

```bash
cargo test --locked -p middleware-zenoh
cargo clippy --locked -p middleware-zenoh --all-targets --all-features -- -D warnings
```

Rollback point: remove only `middleware-zenoh` and its dependency pin; the
local ports and intent/validation contracts remain independently usable.

## 7. Phase F — Real loopback and independence evidence

- [x] Add a serialized, timeout-bounded loopback test with explicit local
      router/client endpoints and unique keys; disable multicast discovery.
- [x] Round-trip one canonical state stream through real Zenoh and decode it
      back to an equal semantic value.
- [x] Round-trip one command intent, materialize it only against the matching
      local lease and assert route/lease TTL capping.
- [x] Exercise a real query/queryable and liveliness declaration/removal.
- [x] Stop/pause the bridge while repeatedly publishing local latest/events;
      assert producer results/counters progress without a Zenoh wait.
- [x] Keep timing thresholds out of host correctness tests; retain diagnostics
      sufficient to debug bounded timeout failures.

Validation:

```bash
cargo test --locked -p middleware-zenoh --test zenoh_loopback -- --test-threads=1
cargo test --locked -p middleware-local --test producer_independence
```

Rollback point: if real loopback setup is nondeterministic, fix explicit
endpoint ownership rather than weakening the test into multicast sleeps or an
ignored default-gate test.

## 8. Phase G — Full quality gate and durable contracts

- [x] Run formatting, warnings-denied Clippy, all workspace tests, real graph,
      cross-language Protobuf, Buf compatibility, dependency policy and robot
      YAML gates.
- [x] Review the graph and source for Zenoh/DTO leakage, wildcard concrete
      keys, unbounded storage, wall-clock safety enforcement, hidden blocking,
      runtime allocation and workspace-owned `unsafe`.
- [x] Update `.trellis/spec/backend/robot-contracts.md` with stable key, port,
      command-intent, freshness and non-RT bridge contracts learned here.
- [x] Update the parent phase D checklist only for evidence actually delivered;
      leave SHM, process-crash and RK3588 benchmark items unchecked and link the
      deferred child boundary.
- [x] Run the full gate again after spec/task changes if production files
      changed during review.

Full validation:

```bash
./tools/ci.sh
git diff --check
git status --short
```

## 9. Completion and handoff

- [x] Mark PRD and implementation checklists complete only with command/test
      evidence.
- [x] Create the named `local-shm-rk3588-qualification` follow-up only with
      explicit task-creation consent; otherwise record it as the next boundary.
- [ ] Commit implementation/spec/task records in reviewable commits without
      including unrelated workspace files.
- [ ] Archive this child, record the developer journal, push the resulting
      branch, and report the exact deferred parent criteria.
