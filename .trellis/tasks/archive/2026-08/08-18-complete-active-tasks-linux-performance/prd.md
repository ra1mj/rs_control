# Complete active tasks with Linux performance discipline

## Goal

Complete, verify, and archive the repository's active robot-framework and
behavior-tree deliveries without weakening their safety or realtime contracts.
All remaining work must be suitable for a Linux-based RK3588 deployment, with
measurable behavior on the target platform where target hardware is required.

## Confirmed Facts

- The repository has nine pre-existing active tasks. The Rust robot-framework
  parent reports 15 of 18 deliverables complete; its active descendants cover
  Orange Pi realtime qualification, local shared-memory qualification,
  RK3588 middleware qualification, KINETIC-derived arm algorithms, the arm
  stack, advanced arm algorithms, and reactive task/performance gates.
- The behavior-tree and motion-participants task is planning-complete (2 of 2
  child deliverables) but remains active and unarchived.
- The framework PRD defines a 1 kHz preferred control period, a verified
  500 Hz fallback, no unbounded queues or runtime allocation on realtime
  paths, isolated device I/O, and measured jitter, deadline, CPU, memory and
  stress behavior on RK3588-class hardware.
- The behavior-tree delivery is bounded and event-driven. It must not block,
  grant authority, refresh command freshness, or produce physical output in a
  motion-control path.
- Host-completable work remains in four areas: the multi-morphology arm stack
  and its reactive/task/performance integration; local shared-memory safety
  and failure semantics; realtime process wiring, observability and MCAP
  recording; and behavior-tree integration, deterministic tests and archive
  review. These remain subject to their own performance and safety gates.
- The user selected strictly sequential execution. The first delivery is
  `08-03-arm-reactive-task-performance`, followed by its advanced-algorithms,
  arm-stack and arm-algorithms parent reviews; then framework host work and
  behavior-tree closure; target-board gates remain last.

## Requirements

- R1: Complete each active task according to its existing approved PRD,
  design, implementation checklist, and dependency order. This coordinating
  task must not silently broaden or weaken a child task's acceptance criteria.
- R2: Preserve the Linux realtime discipline throughout: control and device
  I/O paths use bounded, preallocated data exchange with explicit overflow
  policy; high-priority threads perform no blocking I/O, network reconnect,
  disk logging, or nonessential runtime allocation.
- R3: Prefer event-driven device receive and independently scheduled control
  work over busy polling. Pinning, IRQ affinity, governor policy, memory lock
  and thermal settings must be measured and recorded, not assumed.
- R4: On the designated RK3588 target, qualify the configured control profile
  under CPU, memory, network, storage and available accelerator contention.
  A 1 kHz profile passes only with maximum wakeup jitter at most 100 us and no
  application deadline misses during the agreed stress soak; otherwise retain
  explicit evidence and qualify the 500 Hz fallback.
- R5: Keep safety fail-closed. Loss of lease, stale data, component failure,
  process loss, or shutdown must lead to the task-specific safe state without
  relying on a network, recorder, FSM event loop, or best-effort cleanup.
- R6: Complete shared-memory/local-fast-path work only with a versioned,
  fixed-layout contract, bounded capacity, cache-line isolation, explicit
  producer/consumer crash semantics, and target-board benchmark evidence.
- R7: Complete behavior-tree work as an event-driven, bounded orchestration
  layer. It must remain outside the realtime control loop and must expose
  observable, command-correlated participant state without altering authority
  or stopping semantics.
- R8: Archive a task only after its implementation work, focused checks, full
  integration checks, and required project-spec update are complete. Report
  target-hardware limitations explicitly rather than claiming host-only proof.
- R9: Execute one independently verifiable delivery at a time. Do not start a
  dependent delivery until the preceding delivery's tests, review and task
  state update have passed.

## Acceptance Criteria

- [ ] Every pre-existing active task has either been completed and archived, or
      has a documented external blocker with the exact unmet acceptance
      criterion and evidence.
- [ ] Remaining source changes meet the owning task's checks and the complete
      repository quality gate passes.
- [ ] The realtime path has tests or measurements demonstrating bounded work,
      no producer blockage from a stalled consumer, and no control-path disk,
      network, or unbounded-allocation dependency.
- [ ] RK3588 hardware-required work records the board, kernel, RT patch,
      device tree, CPU governor, cooling conditions, stress setup, results and
      selected 1 kHz or 500 Hz profile.
- [ ] Process exit, lease expiry, and relevant failure cases are proven to
      reach the required motor-disabled or motion-inhibited state.
- [ ] The behavior-tree framework has deterministic tests for action timeout,
      cancellation and fault paths, and a motor-disabled integration check for
      motion-participant observability.

## Scope Boundaries

- Do not add new hardware capabilities, modify Nav2's internal tree, or
  replace the existing transport, middleware, mission, or safety architecture.
- Do not represent unavailable physical hardware qualification as completed;
  collect reproducible host evidence and leave the exact board gate active.
- Avoid broad refactors unless an owning active task requires one to meet its
  acceptance criteria.

## Open Questions

- Whether the required Orange Pi/RK3588 board, configured CAN hardware, motor,
  safety fixture, and stress environment are available to run the physical
  qualification gates.
