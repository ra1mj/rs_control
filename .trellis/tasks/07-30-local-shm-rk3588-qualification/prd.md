# Local SHM and RK3588 qualification

## Goal

Deliver a reviewed local cross-process middleware path and qualify the local
middleware candidates on the Orange Pi 5 Plus/RK3588 without changing the
backend-neutral port semantics established by `middleware-api` or presenting
host-only evidence as a realtime result.

## Background and scope split

The completed Zenoh/Protobuf child delivered nonclonable SPSC latest-value and
ordered-event ports, a safe in-process backend, and a non-RT network adapter.
The parent framework still requires an explicitly safe shared-memory layout,
crash reclamation, torn-read protection, and target-board comparison evidence.

Those deliverables have different completion conditions and are owned by two
children:

| Child | Deliverable | Completion boundary |
|---|---|---|
| `07-30-shm-middleware-backend` | Fixed-layout mmap SPSC backend, contained `unsafe`, real process-fault tests, and a target-capable benchmark executable | Host CI and Linux cross-process fault tests pass |
| `07-30-rk3588-middleware-qualification` | Orange Pi 5 Plus baseline/concurrent measurements, pinned Horus comparison, immutable evidence, and candidate decision | Reviewed physical-board evidence exists |

The parent remains open until both children are complete and their evidence is
reconciled with parent requirements R22, R26-R32, R48, R100, R104, and R105.

## Requirements

- **LSQ-01:** Application code continues to depend only on `middleware-api`;
  it never opens, maps, truncates, locks, or removes a shared-memory object.
- **LSQ-02:** Cross-process latest-value and ordered-event delivery preserves
  the established generations, latest/full/disconnected outcomes, ownership
  return, and saturating common counters.
- **LSQ-03:** Shared payload eligibility is an explicit reviewed fixed-layout
  contract with schema identity. `needs_drop::<T>() == false`, Rust type names,
  and matching byte size alone are never accepted as layout proof.
- **LSQ-04:** The only workspace-owned mmap/raw-pointer/atomic `unsafe` is in a
  named private protocol-module boundary with documented invariants and tests.
- **LSQ-05:** The on-memory format rejects wrong magic/version, channel kind,
  byte order, atomic capability, payload schema/version/size/alignment,
  capacity, stride, or total file size before exposing a port.
- **LSQ-06:** RT-visible storage is bounded, allocated, populated, and
  optionally memory-locked before an endpoint is armed. Hot publish/receive
  operations do not allocate, format, log, block, sleep, resize, or take a file
  lock.
- **LSQ-07:** Producer and consumer hot fields are cache-line separated.
  Payload publication uses position-tagged versions and a bounded seqlock read
  so a killed or preempted writer cannot expose a torn value.
- **LSQ-08:** Endpoint ownership is proven by a nonblocking kernel file lock
  held for the endpoint lifetime. PID/start metadata is diagnostic only and
  never authorizes reclamation; OS lock release is the crash-reclaim proof.
- **LSQ-09:** Clean drop and `SIGKILL` both permit deterministic endpoint reuse;
  a live endpoint is never reclaimed, endpoint exhaustion is typed, and a
  replacement consumer starts at a defined fresh boundary.
- **LSQ-10:** Host tests exercise incompatible layouts, initialization races,
  torn reads, slow/stopped consumers, overflow, exact sequence/drop counters,
  endpoint exhaustion, same-process double attach, PID metadata corruption,
  real subprocess exit, and real `SIGKILL` reclamation.
- **LSQ-11:** The target benchmark compares the safe in-process backend, the
  project SHM backend, and the exact pinned Horus source
  `fac3cfb8704e275bde9daec6d554a17c59dd8532`; Horus remains a research input,
  not a runtime dependency or copied implementation.
- **LSQ-12:** Target evidence records board/revision, kernel/RT patch, device
  tree, toolchain/build IDs, governor/affinity/IRQ profile, payload/capacity,
  thermal state, test duration, stressors, latency distribution and maximum,
  producer-call maximum, CPU/load, allocation count, overflow/gaps, and crash
  outcomes.
- **LSQ-13:** Candidate selection is safety-first. Any unbounded producer wait,
  torn value, unaccounted loss, hot allocation, live-endpoint mis-reclaim, or
  incomplete physical manifest disqualifies a candidate regardless of its
  average latency.
- **LSQ-14:** Host fixtures and short target research runs may validate the
  procedure but report `incomplete`; only immutable Orange Pi 5 Plus evidence
  may close the RK3588 criteria or update a profile to qualified.

## Acceptance criteria

- [x] The SHM backend child is archived with its full host gate passing and a
      reviewed contained-`unsafe` inventory.
- [x] Real subprocess tests prove clean-exit and `SIGKILL` endpoint recovery,
      bounded torn-read behavior, and producer independence from consumers.
- [x] The benchmark/evidence format is deterministic, bounded, and usable on
      the pinned target without fetching source or mutating hardware in CI.
- [ ] Baseline and concurrent Orange Pi 5 Plus runs compare all required
      candidates using the same payloads, capacities, CPU roles, durations,
      stress and thermal evidence.
- [ ] The selected backend and profile cite an immutable evidence SHA-256, or
      the result explicitly remains failed/incomplete with no qualification
      claim.
- [x] Only evidence-backed parent D1 and RK3588 checklist items are marked
      complete; later motion-runtime, motor-timeout and 8-hour system gates
      remain independent.

## Out of scope

- Zenoh shared-memory features, replacing the canonical Protobuf network path,
  or importing Horus as a production dependency.
- MPMC/business-level arbitration in the SHM backend; v1 ports remain
  nonclonable SPSC capabilities.
- Image/point-cloud buffer pools or DMABUF handle transport.
- Claiming the later `motion-rt` control loop, motor stop path, or whole-system
  eight-hour qualification from a middleware-only benchmark.

## Traceability

- Parent: `.trellis/tasks/07-29-rust-robot-framework/` requirements R22,
  R26-R32, R48, R100, R104-R105 and implementation phase D1.
- Stable contracts: `.trellis/spec/backend/robot-contracts.md` and
  `.trellis/spec/backend/realtime-platform.md`.
- Source lock: `platform/orangepi5plus/sources.lock`.
