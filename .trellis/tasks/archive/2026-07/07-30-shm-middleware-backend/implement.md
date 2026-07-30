# Shared-memory middleware backend — Implementation plan

## 1. Planning, baseline and source review

- [x] Load `trellis-before-dev` plus backend directory, robot-contract,
      error-handling, quality and realtime-platform specs before code edits.
- [x] Preserve unrelated untracked Trellis/agent files and record current full
      host baseline.
- [x] Record the exact Horus commit source review: layout, seqlock, cache-line,
      flock/reclaim and auto-POD findings; do not import its source/dependency.
- [x] Pin/review minimum-feature `memmap2`, `bytemuck` and benchmark/test
      dependencies for source, license, MSRV and advisory policy.

Validation:

```bash
./tools/ci.sh
cargo deny check
```

Rollback: no production change until baseline and the unsafe scope are explicit.

## 2. Phase A — architecture and explicit payload/layout contract

- [x] Add/classify `middleware-shm` and the integration benchmark package; add
      exact workspace dependencies and positive/negative architecture evidence.
- [x] Define strict channel names, schema identity, `ShmPayload`, bounded
      semantic `ShmCodec`, identity codec, channel kind, memory policy, typed
      startup errors and bounded diagnostics.
- [x] Define layout-v1 constants/headers with cache-line size assertions,
      checked size/offset/stride calculations and exhaustive compatibility
      validation independent of mmap.
- [x] Add accepted/base/bad tests for names, payload identity, capacities,
      arithmetic overflow and every layout mismatch dimension.

Validation:

```bash
cargo test --locked -p middleware-shm layout
cargo run --locked -p architecture-check -- check
```

Rollback: remove only new packages/policy entries; no existing port changes.

## 3. Phase B — reviewed mapping, namespace and lease boundary

- [x] Implement private mode-0700 namespace and mode-0600 `create_new` region/
      lock files without implicit truncate or unlink.
- [x] Add region lifetime shared locks plus exclusive initialization/cleanup;
      refuse cleanup while any region or endpoint lock is live.
- [x] Implement the single private unsafe mapping module with per-block safety
      comments, exact file length checks, aligned/checked views and no remap.
- [x] Initialize atomics/layout before release-publishing ready magic; attach
      acquire-loads ready then validates the complete immutable identity.
- [x] Populate and touch every page; implement optional required whole-map
      `mlock` with typed startup failure.
- [x] Implement publisher/subscriber nonblocking lifetime locks with lock-as-
      authority, derived PID/epoch repair, clean-drop order and explicit remove.

Validation:

```bash
cargo test --locked -p middleware-shm namespace
cargo test --locked -p middleware-shm attach
rg -n '\bunsafe\b' crates/protocol_module/middleware-shm
```

Rollback: retain pure layout/payload types while replacing only mapping/lease code.

## 4. Phase C — latest-value backend

- [x] Implement cache-line-separated producer/consumer state and a fixed
      multi-slot latest window.
- [x] Implement odd/even position-tagged publication and bounded seqlock copy
      with no invalid-value construction or unbounded retry.
- [x] Implement `LatestPublisher`/`LatestSubscriber`, exact generations,
      overwrite/disconnect behavior, common counters and backend diagnostics.
- [x] Test no-value/first/latest/overwrite/wrap boundary, forced odd writer,
      repeated lap races, bounded retry exhaustion and saturating counters.
- [x] Prove zero steady-state allocation and no file lock/syscall in hot methods.

Validation:

```bash
cargo test --locked -p middleware-shm latest
cargo clippy --locked -p middleware-shm --all-targets --all-features -- -D warnings
```

Rollback: keep mapping/layout; disable latest constructor until protocol is sound.

## 5. Phase D — ordered-event backend

- [x] Implement fixed SPSC head/tail/slot-version ring with checked nonzero
      capacity and exact FIFO generations.
- [x] Preserve ownership on full/disconnected and update common counters only
      for the established outcome meanings.
- [x] Reset a newly reclaimed subscriber to head and count abandoned backlog;
      never replay the dead endpoint's pending events.
- [x] Test empty/full/wrap/FIFO, stopped consumer, producer crash, consumer
      reclaim, exact skip/full/disconnect counters and saturation.
- [x] Prove zero steady-state allocation and bounded producer calls at full.

Validation:

```bash
cargo test --locked -p middleware-shm ordered
```

Rollback: keep latest backend and remove only ordered constructors if needed.

## 6. Phase E — real process and fault evidence

- [x] Add timeout-bounded peer helpers inside Linux integration tests without
      root or production namespace access.
- [x] Exercise latest and ordered exchange across real processes.
- [x] Refuse a second live publisher/subscriber and report endpoint exhaustion.
- [x] Prove clean drop and actual `Child::kill` release the endpoint lock and a
      new process reclaims the exact endpoint.
- [x] Corrupt diagnostic PID/epoch fields and prove they neither block a proven
      reclaim nor authorize a live-endpoint reclaim.
- [x] Kill a writer after odd-stamp publication and prove no torn payload is
      returned; reclaim then restores delivery.
- [x] Pause/kill consumers while the producer runs a fixed large iteration set;
      assert completion, exact full/overwrite/disconnect/skip counters and no wait.

Validation:

```bash
cargo test --locked -p middleware-shm --test process_faults -- --test-threads=1
```

Rollback: preserve failing objects/logs only in the test temp directory; never
weaken a crash case into a simulated-only test.

## 7. Phase F — target-capable benchmark handoff

- [x] Add/classify `middleware-qualification` integration binary depending only
      on the public middleware packages.
- [x] Define fixed 64/256/1024-byte explicit Pod benchmark payloads and bounded
      CLI settings for backend/kind/capacity/warm-up/sample/peer mode.
- [x] Preallocate measurements, record a bounded histogram/max/CPU/allocation/
      delivery counters, and emit deterministic versioned JSON after the run.
- [x] Implement in-process thread mode and cross-process SHM peer mode with
      explicit readiness, timeout and cleanup.
- [x] Reject zero/excessive warm-up, samples, capacity or output reuse; label
      all non-RK3588 invocations as host/research rather than qualified.
- [x] Add process-level CLI tests for stable success/error output and schema.

Validation:

```bash
cargo test --locked -p middleware-qualification
cargo run --locked -p middleware-qualification -- --help
```

Rollback: the SHM backend remains usable if the benchmark package is removed.

## 8. Phase G — full review and durable contract

- [x] Run formatting, warnings-denied Clippy, all workspace tests, architecture,
      dependency policy and full CI.
- [x] Audit raw operations, atomics/orderings, offset arithmetic, mapping/file
      lifetime, locks, SPSC ownership, bounded retries, allocations and syscalls.
- [x] Verify application crates contain no memmap/bytemuck/SHM file access and
      existing `middleware-local`/Zenoh behavior is unchanged.
- [x] Update backend specs with the reviewed unsafe/layout/reclaim scenario and
      parent D1 only for evidence actually delivered.
- [x] Run the complete gate once more after any production/spec correction.

Full validation:

```bash
./tools/ci.sh
git diff --check
git status --short
```

## 9. Completion and handoff

- [x] Mark PRD/checklists complete only with command and fault-test evidence.
- [x] Commit only task-scoped code/spec/planning files; preserve unrelated
      workspace bootstrap files.
- [x] Archive this host child, record the journal and push.
- [x] Leave the RK3588 child planning/in-progress with exact commands and
      evidence requirements; do not mark target performance criteria complete.
