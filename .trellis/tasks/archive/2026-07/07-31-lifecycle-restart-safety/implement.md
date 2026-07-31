# Implementation plan

## 1. Production reducer

- [x] Add application-owned lifecycle state, event, effect, rejection and
  projection types.
- [x] Add boot-correlated machine construction and fixed-capacity dispatch.
- [x] Implement exhaustive transitions, reconstruction outcomes and authority
  sequence guards.
- [x] Implement global E-stop latch and ordered Running-exit effects.

## 2. Diagnostics

- [x] Enumerate all stable states and event kinds.
- [x] Generate a complete transition report from canonical reducer dispatch.
- [x] Generate deterministic DOT from changed-state projections.

## 3. Verification

- [x] Test every state/event-kind projection and report row.
- [x] Test restart, stale boot, duplicates, reorder, event loss and timeout.
- [x] Test E-stop preemption/latch/reset and exact Running-exit order.
- [x] Test deterministic replay/report/DOT and zero hot-path allocation.
- [x] Run formatting, warnings-denied Clippy, package tests, architecture
  policy and the full host CI gate.

## 4. Documentation and delivery

- [x] Update the lifecycle FSM specification with the final boot-correlation
  and reset contract.
- [x] Check the parent G2 items backed by host evidence.
- [x] Commit implementation separately from task archival and push `main`.
