# Arm reactive, task and performance integration — implementation plan

## 1. Dependency and contract review

- [x] Verify all algorithm children are committed and archived.
- [x] Load `trellis-before-dev` and applicable backend/task contracts.
- [x] Admit and exact-pin the benchmark dependency, or implement a bounded
      project tool if admission fails.

## 2. Reactive and task integration

- [x] Add singularity-aware prepared RMP component and combination evidence.
- [x] Add task compile context, request/generation identity and typed results.
- [x] Compile MoveJoint/MovePose/Cartesian/Pick/Place/Gripper/Hold stages.
- [x] Add cancellation, failure, late-result and profile/reload invalidation.

## 3. Performance and authority gates

- [x] Add all-family preparation and warm-query benchmarks.
- [x] Add post-preparation allocation/capacity/work-counter tests.
- [x] Add isolated Release microsecond and same-host regression gate.
- [x] Add cross-child collision typestate and authority/deadline tests.

## 4. Finish

- [x] Run targeted and full non-hardware workspace gates.
- [x] Run `trellis-check` and update durable specs.
- [ ] Commit/archive this child and complete the parent acceptance matrix.

Rollback removes task/reactive/benchmark integration only and preserves the
independently completed algorithm kernels.
