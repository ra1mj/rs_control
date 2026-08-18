# Active-task completion with Linux performance discipline — technical design

## Purpose and Boundary

This is a coordination task, not a new runtime subsystem. It sets the
dependency order, shared Linux performance rules, evidence expectations, and
archive gates for existing task-owned deliveries. Source and test changes stay
in the owning task's scope and do not gain a parallel implementation path here.

## Delivery Order

```text
08-03 arm reactive/task/performance
  -> 08-03 advanced algorithms parent review/archive
  -> 07-31 arm stack review/archive
  -> 07-31 arm algorithms parent review/archive
  -> 07-30 local IPC host review and framework host work
  -> 08-06 behavior-tree integration review/archive
  -> Orange Pi/RK3588 platform, middleware and physical safety gates
```

The first four stages are host-only. The local-IPC stage may implement and
test host semantics, but it cannot select or qualify a target runtime profile.
The last stage stays explicitly incomplete until target evidence exists.

## Shared Linux Performance Contract

Every completion stage that changes an execution path preserves these rules:

- Realtime producers use fixed-capacity, preallocated queues or latest-value
  mailboxes with a documented overflow result. A stalled or crashed consumer
  cannot block a producer.
- Event-driven receive and independently periodic control are preferred over
  busy waiting. CPU affinity, scheduling policy, IRQ placement, governor,
  memory locking and thermal policy are applied outside the control loop and
  validated by readback and measurements.
- Priority threads do not make network, filesystem, logging, dynamic loader,
  allocator-growth or reconnection calls. Diagnostics use bounded compact
  records and are drained by non-realtime workers.
- Timing claims record host/board identity, kernel, compiler/build profile,
  topology and concurrent load. Host timing is only a regression signal; it
  is not RK3588 or physical-control qualification.
- Safety remains local and fail-closed: old leases, stale data, process loss,
  queue failure and cancellation cannot preserve or create control authority.

## Stage Ownership and Exit Rules

| Stage | Owning task | Exit rule |
|---|---|---|
| Reactive integration | `08-03-arm-reactive-task-performance` | All task stages, invalidation, allocation/work evidence, isolated Release benchmark, and host quality gates pass. |
| Arm parent closure | `08-03-advanced-arm-algorithms`, `07-31-kinetic-arm-stack`, `07-31-kinetic-arm-algorithms` | Each parent maps its requirements to passing children, updates durable specs, commits, and archives in order. |
| Local IPC/framework host work | `07-30-local-shm-rk3588-qualification`, `07-29-rust-robot-framework` | Fixed-layout safety/failure tests and host-only framework checks pass; any target selection claim remains deferred. |
| Behavior tree | `08-06-behavior-tree-motion-participants` | Deterministic core/navigation integration and motor-disabled participant checks pass; no Nav2-internal or authority change is introduced. |
| Physical qualification | `07-29-orangepi-rt-platform`, `07-30-rk3588-middleware-qualification`, framework target gates | Immutable target evidence satisfies the exact board, thermal, timing, CAN and failure criteria; otherwise record the blocking criterion and preserve evidence. |

## Operational Controls

- Before a source edit, activate the owning task and load its applicable
  backend guidance through `trellis-before-dev`.
- Before a dependent task begins, check the prior task's acceptance matrix and
  archive state rather than trusting checklist counts alone.
- Use targeted checks while implementing; run the owning full host gate before
  closing a task. Run the repository quality gate after integration boundaries.
- Preserve raw failures, benchmark outputs, captures and target evidence. A
  failed timing profile may establish the documented 500 Hz fallback only when
  the complete required matrix passes for that profile.

## Rollback

Each stage reverts only its own unvalidated source changes. No rollback changes
motor parameters, deletes raw evidence, rewrites task history, or converts a
target-only acceptance gate into a host claim.
