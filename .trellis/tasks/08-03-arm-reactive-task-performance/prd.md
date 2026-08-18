# Arm reactive, task and performance integration

## Goal

Integrate completed advanced algorithms into singularity-aware reactive control
and task-stage compilation, then enforce portable allocation/work contracts and
isolated Release performance gates across every algorithm family.

## Dependencies

The advanced-IK, sampling/roadmap and optimization/constraint children must be
committed and archived before this child starts.

## Requirements

- **IT-01:** Add singularity-aware RMP damping using prepared Jacobian evidence;
  bound reach, damping, joint-limit and singularity contributions separately.
- **IT-02:** Compile MoveJoint, MovePose, Cartesian, Pick and Place stages into
  bounded context-bound collision-unchecked plan/trajectory proposals.
- **IT-03:** Gripper and Hold remain semantic intents and never construct
  hardware commands.
- **IT-04:** Every asynchronous/pending algorithm request carries task request
  identity, profile/group/order and generation. Cancellation/context change
  invalidates it immediately; late results cannot advance state or authority.
- **IT-05:** Add deterministic cross-child tests for prepared state invalidation,
  planner/solver identity, lease/deadline non-authority and collision typestate.
- **IT-06:** Add an exact-pinned, dependency-admitted benchmark harness or a
  project tool providing P50/P95/P99, throughput, allocations, prepared bytes
  and work counters for all kernels.
- **IT-07:** Portable default gates reject post-preparation allocation/growth,
  capacity overflow and nondeterministic counters. Isolated Release gates apply
  the parent microsecond targets, release floors and <=15% same-host regression.
- **IT-08:** Benchmark evidence records host identity and is never labeled
  RK3588, realtime or physical-arm qualification.

## Acceptance Criteria

- [x] RMP stays finite and within acceleration limits near singularities and
      joint boundaries; collision RMP remains unavailable.
- [x] Each supported task stage produces the correct typed proposal or exact
      failure evidence under the selected solver/planner.
- [x] Cancellation, stage failure and profile/prepared-generation changes reject
      late results and cannot renew lease/deadline validity.
- [x] Compile/runtime tests prove collision-unchecked results cannot reach
      physical admission.
- [x] Benchmarks cover every named solver/planner with separated preparation
      and warm-query measurements.
- [x] Warm nominal targets and release floors pass on the recorded reference
      host; regressions are reported against the same host/profile only.
- [x] Full non-hardware workspace quality gates pass.

## Out of Scope

Collision, physical execution, Orange Pi/RK3588 qualification, CAN, live ROS,
GPU, Python and online learning remain outside this child.
