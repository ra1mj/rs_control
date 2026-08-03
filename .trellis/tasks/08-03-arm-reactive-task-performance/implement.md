# Arm reactive, task and performance integration — implementation plan

## 1. Dependency and contract review

- [ ] Verify all algorithm children are committed and archived.
- [ ] Load `trellis-before-dev` and applicable backend/task contracts.
- [ ] Admit and exact-pin the benchmark dependency, or implement a bounded
      project tool if admission fails.

## 2. Reactive and task integration

- [ ] Add singularity-aware prepared RMP component and combination evidence.
- [ ] Add task compile context, request/generation identity and typed results.
- [ ] Compile MoveJoint/MovePose/Cartesian/Pick/Place/Gripper/Hold stages.
- [ ] Add cancellation, failure, late-result and profile/reload invalidation.

## 3. Performance and authority gates

- [ ] Add all-family preparation and warm-query benchmarks.
- [ ] Add post-preparation allocation/capacity/work-counter tests.
- [ ] Add isolated Release microsecond and same-host regression gate.
- [ ] Add cross-child collision typestate and authority/deadline tests.

## 4. Finish

- [ ] Run targeted and full non-hardware workspace gates.
- [ ] Run `trellis-check` and update durable specs.
- [ ] Commit/archive this child and complete the parent acceptance matrix.

Rollback removes task/reactive/benchmark integration only and preserves the
independently completed algorithm kernels.
