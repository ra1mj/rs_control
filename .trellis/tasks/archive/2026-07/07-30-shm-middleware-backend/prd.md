# Shared-memory middleware backend

## Goal

Implement a Linux cross-process shared-memory backend for the existing
`middleware-api` latest-value and ordered-event ports. The backend must keep RT
producer operations bounded and allocation-free, reject incompatible memory
before use, survive peer crashes without PID-based guessing, and supply a
target-capable benchmark executable for the later RK3588 child.

## Confirmed facts

- `middleware-api` already defines nonclonable SPSC latest and ordered ports,
  typed publish outcomes, ownership-returning event backpressure, generations,
  and five common saturating counters.
- `middleware-local` is the safe in-process candidate and uses preallocated
  `waitfree-sync`/`rtrb` primitives. This task adds a backend; it does not
  replace or expose those implementation types.
- The workspace normally forbids `unsafe`. The cross-layer contract permits a
  newly reviewed private protocol-module mmap/raw-atomic boundary only when
  the architecture task records its invariants and tests.
- Linux/RK3588 supplies lock-free 64-bit atomics. The new crate will refuse to
  compile on targets without `target_has_atomic = "64"` and will reject
  non-little-endian attach in layout v1.
- Rust 1.97 provides nonblocking `File::try_lock`; Linux `flock` ownership is
  associated with the open file description and is released when its final
  descriptor closes, including process termination.
- `memmap2` file mappings require an explicit unsafe proof because another
  actor could truncate or modify the backing file. The project will use a
  private mode-0700 namespace, mode-0600 fixed-length files, a region lifetime
  lock, no remap/resize API, and raw access only through the reviewed module.
- `bytemuck::Pod` excludes padding, pointers, invalid bit patterns and interior
  mutability. A project payload also needs an explicit stable schema ID/version;
  `Pod`, size, alignment, or Rust type name alone is not compatibility identity.
- The exact Horus comparison commit uses cache-line-separated positions,
  position-tagged per-slot seqlocks, bounded retries and per-endpoint `flock`
  files, but its production auto-POD path still treats `!needs_drop` and size
  as sufficient. This task may reproduce the reviewed protocol ideas, not its
  code or claims.

## Requirements

- **SHM-01 — crate boundary:** Add `middleware-shm` under
  `crates/protocol_module/`, classify it as `protocol_module`, and keep mmap,
  lock files and layout types private. Public endpoints implement only the
  existing middleware traits plus backend-specific startup/diagnostic methods.
  An infallible bounded encode/fallible decode codec maps semantic `T` to the
  private fixed representation, so application code does not acquire a
  bytemuck or backend-layout dependency.
- **SHM-02 — explicit payload contract:** Shared representations implement a
  safe `ShmPayload: bytemuck::Pod + Send + Sync + 'static` trait with nonzero
  stable 16-byte schema ID and nonzero schema version. Zero-sized values and
  alignment above the reviewed layout limit are rejected. `needs_drop` is
  never queried. An identity codec exists only when semantic `T` is itself an
  explicitly approved `ShmPayload`.
- **SHM-03 — namespace safety:** Channel names use a strict bounded ASCII
  grammar and cannot contain separators or traversal. Creation is exclusive,
  existing objects are never silently truncated, files are private, and
  cleanup/recreate requires proof that no region/endpoint lock is live.
- **SHM-04 — versioned layout:** Layout v1 stores magic, version, endian and
  atomic-width tags, channel kind, capacity, payload schema/version/size/
  alignment, slot stride, total bytes and an initialization state. Magic/ready
  is published last with release ordering; attach reads it first with acquire.
- **SHM-05 — cache ownership:** Producer-written and consumer-written atomics
  occupy separate 64-byte cache lines. Every offset/stride uses checked
  arithmetic and is validated against the exact file length before a raw view
  exists.
- **SHM-06 — mapped-memory safety:** All workspace-owned unsafe operations are
  contained in one private `mapping` module. Each block documents mapping
  lifetime, alignment, bounds, atomic-only access, single-writer ownership and
  seqlock validity. The crate uses `deny(unsafe_code)` outside that module and
  `deny(unsafe_op_in_unsafe_fn)` inside it.
- **SHM-07 — startup memory policy:** Mapping uses populate/prefault before arm;
  every page is explicitly touched. A configuration can require `mlock`, which
  fails startup rather than degrading silently. Map creation, locks and cleanup
  are never called from hot port methods.
- **SHM-08 — latest semantics:** A single producer publishes to a fixed
  multi-slot latest window using odd-writing/even-complete position-tagged
  versions. A consumer performs at most a fixed number of seqlock retries,
  returns only a complete newest generation, and records overwritten/torn-race
  observations without spinning indefinitely.
- **SHM-09 — ordered semantics:** A fixed-capacity SPSC ring preserves FIFO.
  Publish returns the original `T` on full/disconnected, receive returns the
  oldest complete generation, and head/tail/slot versions cannot expose a
  partially written event.
- **SHM-10 — common counters:** The five `ChannelCounters` fields retain their
  established meaning and saturate. Backend diagnostics add attach rejection,
  bounded read-race, reclaimed endpoint, and skipped stale-backlog counters
  without reinterpreting common fields.
- **SHM-11 — endpoint leases:** Publisher and subscriber each hold a distinct
  exclusive nonblocking lock file for their lifetime. Active/PID metadata is
  derived diagnostics. A claim succeeds only after acquiring the lock; a live
  lock or lock error refuses reclaim. A replacement subscriber starts at the
  current producer head and accounts for abandoned backlog.
- **SHM-12 — process lifecycle:** Clean drop clears derived metadata before
  releasing its lock. Abrupt exit relies on kernel lock release; the next claim
  repairs stale metadata. Object unlink is explicit and never occurs in an
  endpoint destructor.
- **SHM-13 — deterministic faults:** Linux integration tests use real child
  processes and bounded timeouts to cover normal exchange, clean exit,
  `Child::kill`/SIGKILL reclamation, live endpoint refusal, same-process double
  attach, endpoint exhaustion, stale/corrupt PID metadata, incompatible layout,
  writer death in an odd seqlock phase, slow consumer and exact counters.
- **SHM-14 — hot-path evidence:** After construction/warm-up, latest/event
  publish and receive perform zero allocations. A stalled consumer cannot make
  producer methods wait, sleep, lock a file, retry unboundedly, or enter the
  kernel; full/disconnected outcomes remain immediate.
- **SHM-15 — benchmark executable:** Add an integration-layer executable that
  runs bounded warm-up/sample loops for `middleware-local` and
  `middleware-shm`, records deterministic JSON with raw identity/config,
  latency histogram/max, operation max, CPU time, allocation totals and
  delivery counters, and can coordinate thread or subprocess peers. It does
  not decide RK3588 qualification in host CI.
- **SHM-16 — dependency policy:** Pin `memmap2`, `bytemuck` and any benchmark-
  only dependency exactly, enable minimum features, record source/license/MSRV,
  and pass `cargo deny`. Horus is not linked or copied.

## Acceptance criteria

- [x] The real architecture graph contains a classified `middleware-shm` and
      application-facing code has no mmap/bytemuck/memmap dependency.
- [x] Compile-time/runtime checks enforce explicit `ShmPayload` identity and
      reject wrong magic/version/kind/endian/atomic/schema/size/alignment/
      capacity/stride/file length without exposing an endpoint.
- [x] The unsafe inventory is limited to the private mapping module, every
      block has a local safety argument, and no workspace `unsafe` exception is
      broadened to other crates.
- [x] Latest and ordered ports pass accepted/base/bad cases with exact common
      counters, codec-rejection diagnostics, bounded backend diagnostics and
      zero steady-state allocations.
- [x] Real subprocess tests pass for normal delivery, live endpoint refusal,
      clean reuse, SIGKILL reuse, PID-metadata corruption, writer death during
      publication, endpoint exhaustion and stale-backlog skip.
- [x] Slow/stopped/crashed consumers never block the producer; ordered full
      returns ownership and latest replacement remains exactly accounted.
- [x] The benchmark executable emits deterministic, schema-versioned evidence
      for local and SHM candidates and rejects invalid/unbounded run settings.
- [x] Formatting, warnings-denied Clippy, workspace tests, architecture,
      dependency policy and full `./tools/ci.sh` all pass.
- [x] Parent D1 items are updated only for delivered host evidence; RK3588
      latency/jitter/cycle selection remains unchecked for the physical child.

## Out of scope

- Multiple writers/readers, fanout matrices, arbitration, dynamic endpoint
  growth, variable-sized payloads, heap pointers or destructor-bearing values.
- Cross-machine shared memory, Zenoh SHM, transparent backend auto-selection,
  or changing `middleware-api` based on deployment location.
- Physical RK3588 pass/fail, profile qualification, or a whole-control-loop
  deadline claim; those belong to `07-30-rk3588-middleware-qualification`.
- Automatically deleting unknown/stale files or reclaiming based on `/proc`,
  PID existence, wall-clock heartbeat, or elapsed timeout alone.

## Traceability

- Planning parent: `07-30-local-shm-rk3588-qualification` LSQ-01 through
  LSQ-10 and LSQ-13.
- Framework parent: R22, R26, R28-R31, R64, R104-R105; implementation D1.
- Stable contracts: backend `robot-contracts.md`, `error-handling.md`,
  `quality-guidelines.md`, and `realtime-platform.md`.
