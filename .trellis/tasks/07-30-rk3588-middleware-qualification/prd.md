# RK3588 middleware qualification

## Goal

Use the completed host SHM backend and benchmark executable to compare local
middleware candidates on the Orange Pi 5 Plus/RK3588, preserve immutable
baseline/concurrent evidence, and make an explicit safety-first backend/profile
decision without extrapolating desktop results or Horus marketing claims.

## Dependency and entry gate

This task starts only after `07-30-shm-middleware-backend` is archived. The
benchmark binary, evidence schema, payload definitions, failure semantics and
real process-fault tests must already pass host CI. Planning may be refined
earlier, but physical execution does not fetch or substitute unreviewed code.

## Requirements

- **RKMW-01:** Verify the exact Orange Pi 5 Plus board/revision, power/cooling,
  Ubuntu, pinned kernel/PREEMPT_RT source/config, device tree, Rust/build commit,
  CPU role/affinity, governor, scheduler budget, IRQ and memory-lock state.
- **RKMW-02:** Compare `middleware-local`, project `middleware-shm`, and Horus
  commit `fac3cfb8704e275bde9daec6d554a17c59dd8532`. Horus is built/run as a
  locked external comparison and never added to the robot runtime graph.
- **RKMW-03:** Use identical 64/256/1024-byte payloads where the candidate API
  permits, identical capacities, warm-up, sample count/duration, CPU placement,
  scheduling policy, and baseline/concurrent stress conditions. Any unavoidable
  semantic difference is recorded rather than normalized away.
- **RKMW-04:** Record one-way latency distribution and maximum, producer-call
  distribution/maximum, jitter, CPU time/cycles/load, memory, zero-after-arm
  allocations, delivered generations, overwrites/full/gaps/disconnects,
  endpoint reclamation and thermal/frequency observations.
- **RKMW-05:** Exercise latest and ordered semantics with a normal consumer,
  deliberately slow/stopped consumer, full queue, clean peer exit and killed
  peer. No candidate may hide a wait, loss, tear or reclaim error behind an
  average statistic.
- **RKMW-06:** Baseline and concurrent modes reuse the platform's safe evidence
  conventions: new output directory, raw inputs/results, `pass|fail|incomplete`,
  explicit reasons and complete SHA-256 manifest. Host fixtures are always
  incomplete for hardware qualification.
- **RKMW-07:** A full concurrent result includes CPU/memory/storage, real
  Ethernet, representative accelerator load, thermal/frequency monitoring and
  the platform's required duration. Short research runs remain incomplete but
  are retained.
- **RKMW-08:** Safety gates dominate selection. Torn data, live endpoint
  mis-reclaim, unaccounted loss, nonzero hot allocation, producer blocking,
  missing manifest data, thermal/frequency failure or an invalid source identity
  disqualifies the candidate regardless of percentile latency.
- **RKMW-09:** If the 1 kHz profile cannot satisfy its declared communication
  and whole-platform gates, preserve evidence and evaluate the separately
  declared 500 Hz profile. Never switch a running system or rewrite a candidate
  profile silently.
- **RKMW-10:** The selected implementation/profile records the evidence digest.
  A failed or incomplete comparison leaves all parent performance criteria
  unchecked and does not remove safer fallback implementations.

## Acceptance criteria

- [ ] The host SHM child and its full CI/fault suite are complete before target
      execution begins.
- [ ] The target procedure refuses wrong/unrecorded hardware, source, build,
      scheduler, affinity, memory-lock, duration or output state.
- [ ] Baseline evidence for all candidates includes raw versioned results and a
      complete checksum manifest.
- [ ] Concurrent evidence exercises the required platform stress/thermal matrix
      and records the same candidate/config identities as baseline.
- [ ] Slow/stopped/killed peer cases demonstrate bounded producer progress,
      explicit loss/full/disconnect accounting and correct endpoint reclaim.
- [ ] The comparison report states semantic differences, distributions,
      maxima, CPU/cycles/load, memory and allocation counts; it does not rank by
      average latency alone.
- [ ] A selected backend/profile cites immutable passing evidence, or the task
      reports failed/incomplete and leaves the parent unqualified.
- [ ] Evidence-backed parent D1/RK3588 items and platform profile fields are the
      only criteria changed.

## Out of scope

- Modifying motor behavior, enabling motion, transmitting CAN frames or
  claiming the later single-motor/whole-runtime acceptance sequence.
- Internet fetches during qualification, unpinned Horus revisions, or copying
  Horus implementation into production.
- RDK X5 qualification; it remains a later replay of the same evidence contract.

## Traceability

- Planning parent: `07-30-local-shm-rk3588-qualification` LSQ-11 through LSQ-14.
- Framework parent: R11, R22-R23, R31, R48, R100, R104-R105 and D1 target board
  benchmark criteria.
- Platform contract: `.trellis/spec/backend/realtime-platform.md`.
