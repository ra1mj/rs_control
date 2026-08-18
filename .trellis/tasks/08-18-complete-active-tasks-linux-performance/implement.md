# Active-task completion with Linux performance discipline — execution plan

## 0. Planning Gate

- [x] Obtain review approval for this PRD, design and ordered plan.
- [x] Start the coordinating task, then activate only the first owning task.
- [x] Record the target-hardware availability decision without changing target
      gates into host-only claims. See [blockers.md](./blockers.md).

## 1. Reactive Task and Performance Integration

Owner: `08-03-arm-reactive-task-performance`.

- [x] Verify the archived advanced IK, sampling/roadmap and optimization/
      constraint children satisfy the dependency gate.
- [x] Load `trellis-before-dev` and relevant backend/task contracts.
- [x] Admit an exact-pinned benchmark dependency or implement the existing
      plan's bounded project-owned alternative after source/license review.
- [x] Implement and test singularity-aware prepared RMP composition, typed
      task-stage compilation, request/generation invalidation, cancellation and
      late-result rejection.
- [x] Add all-family setup/warm benchmarks, post-preparation allocation and
      capacity tests, same-host Release regression checks, and collision/
      authority/deadline boundary tests.
- [x] Run targeted checks, the full non-hardware workspace gate, `trellis-check`,
      durable-spec update, commit, and archive.

Exit: every acceptance criterion in the owning PRD passes with host identity
recorded; no result is represented as physical-arm or RK3588 qualification.

## 2. Arm Parent Closure

Owners: `08-03-advanced-arm-algorithms`, `07-31-kinetic-arm-stack`,
`07-31-kinetic-arm-algorithms` in that order.

- [x] Run each parent acceptance matrix against its archived children and the
      newly completed reactive child.
- [x] Verify deterministic seeds, prepared-state identity checks, bounded
      workspace behavior, collision-unchecked typestate and absence of KINETIC
      production dependencies.
- [x] Run required Clippy, architecture, dependency/source-policy and full host
      checks for each owner.
- [x] Update specifications, commit, and archive each task before beginning the
      next parent review.

Exit: all three arm-task parents are archived independently with their
requirement-to-test mapping intact.

## 3. Framework Host Work and Local IPC

Owners: `07-30-local-shm-rk3588-qualification` and
`07-29-rust-robot-framework`.

- [x] Recheck the completed host SHM backend and benchmark harness against the
      explicit fixed-layout, version, torn-read, SIGKILL/reclamation, endpoint
      exhaustion, sequence and bounded-producer rules.
- [x] Complete remaining host-verifiable `motion-rt` composition, thread
      separation, local lease enforcement, health/freshness and safe-failure
      tests under the framework task.
- [x] Complete bounded diagnostics, non-realtime export, MCAP recording,
      backpressure, quota, recovery and deterministic replay tests.
- [x] Run host stress/fault injection without interpreting it as a target-board
      realtime result. Leave platform profile selection and physical
      qualification open until real evidence exists.

Exit: host software tests pass and target-only acceptance criteria are listed
verbatim as remaining gates, not marked complete.

## 4. Behavior-tree Integration Closure

Owner: `08-06-behavior-tree-motion-participants`.

- [x] Re-run bounded reducer, action-deadline, navigation acknowledgement,
      cancellation, fault and participant-snapshot checks.
- [x] Run the motor-disabled E2E harness when its host dependencies are
      available; otherwise record the exact missing host dependency.
- [x] Confirm snapshot publication cannot delay or authorize motion, Nav2
      internals remain untouched, and all control remains motor-disabled.
- [x] Run the full host gate, update durable docs/specs, commit, and archive.

Exit: parent acceptance matrix is evidenced by its two archived children and
current integration checks.

## 5. Target-board Qualification

Owners: `07-29-orangepi-rt-platform`,
`07-30-rk3588-middleware-qualification`, and relevant framework gates.

- [ ] Build/boot the pinned Orange Pi image and collect immutable board,
      cooling, kernel, device-tree, governor, affinity and CAN evidence.
- [ ] Run the required baseline and full concurrent stress matrix, including
      slow/stopped/killed local peers and thermal conditions.
- [ ] Pass 1 kHz only with the declared jitter/deadline limits, or preserve its
      failure and independently qualify 500 Hz with the complete matrix.
- [ ] Perform the required motor-disabled and, only with safety fixture,
      physical safe-stop checks. Archive target tasks only from raw evidence.

These gates remain active. Their precise prerequisites, absent evidence and
required execution order are recorded in [blockers.md](./blockers.md).

## 6. Final Integration

- [x] Run the repository host quality gate for the final host delivery; target
      gates remain additive and were not represented as a clean-checkout target
      qualification.
- [x] Compare every active task against its PRD acceptance criteria and archive
      only tasks with passing evidence.
- [x] Record remaining hardware blockers, including exact criterion and
      evidence status, if the target environment is unavailable.

Host delivery evidence is in
`../archive/2026-08/08-18-host-motion-runtime-recording/verification.md`.
