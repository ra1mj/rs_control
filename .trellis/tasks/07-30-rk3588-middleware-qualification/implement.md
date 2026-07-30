# RK3588 middleware qualification — Implementation plan

## 1. Entry and measurement semantics

- [x] Archive the host SHM child with full CI and real process-fault evidence.
- [x] Confirm the current development host is not an RK3588 target and preserve
      the physical acceptance items as incomplete.
- [x] Change the benchmark payload to carry `CLOCK_MONOTONIC` send time and
      record one-way receive latency separately from producer-call time.
- [x] Add exact safe clock dependency policy and CLI tests for both process
      delivery kinds.

## 2. Target envelope

- [x] Add `qualify-middleware.sh` with strict bounded arguments, dry-run plan,
      absent output, exact aarch64/device-tree/kernel/profile/build/source checks
      and mandatory lock-required SHM mode.
- [x] Verify the project binary/hash and pinned external Horus checkout/runner
      without fetching or linking Horus into the workspace.
- [x] Run an identical latest/ordered × 64/256/1024 matrix for local, SHM thread,
      SHM process and the external comparison adapter under fixed CPU/scheduler
      placement.
- [x] Run the project crash/fault suite offline and require equivalent Horus raw
      fault evidence or retain an incomplete verdict.

## 3. Platform stress integration and evaluator

- [x] Reuse `qualify-rt.sh` baseline/concurrent stress and thermal evidence
      while the middleware matrix is active; require eight hours for a complete
      concurrent result.
- [x] Add a strict evaluator for schemas, identity/config equality, allocations,
      counters, fault outcomes, latency/operation maxima and safety-first
      eligibility without average-only ranking.
- [x] Emit deterministic verdict/comparison files and a complete sorted
      `SHA256SUMS`; never mutate the candidate runtime profile.

## 4. Host-safe verification

- [x] Extend platform host fixtures for shell parsing, dry-run commands, wrong
      architecture/device-tree/source/profile/output refusal and forced
      incomplete fixture evidence.
- [x] Test evaluator good/base/bad bundles, semantic mismatch, nonzero
      allocation, missing Horus, torn/fault failure and deterministic output.
- [x] Run `./tools/ci.sh`, task validation and `git diff --check`.

## 5. Physical execution gate

- [ ] On the identified Orange Pi 5 Plus, run baseline matrix and retain raw
      immutable results for every candidate/config.
- [ ] Run the complete concurrent matrix with platform loads/thermal evidence.
- [ ] Review safety gates and choose the backend/profile only from passing
      evidence, otherwise record failed/incomplete without profile promotion.
- [ ] Update parent RK3588/D1 items and profile digest only after independent
      evidence review; archive, journal, commit and push.

Rollback: failed/incomplete target output is preserved, no profile is changed,
and a corrected attempt uses a new absent directory.
