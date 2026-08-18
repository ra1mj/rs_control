# Arm reactive, task and performance integration — technical design

## Reactive policy

Prepared Jacobian analysis supplies a bounded singularity metric and adaptive
damping contribution. RMP components return separate accelerations/weights;
the combiner validates each component, sums deterministically and clamps once
to compiled acceleration limits. Collision-dependent components stay typed
unavailable.

## Task compilation

`TaskCompileContext` references exact prepared kinematics/planner state and an
immutable selection policy. `TaskRequestId` plus generation identifies one
active stage compilation. Results are an enum of collision-unchecked plan,
trajectory, gripper or hold intent. They do not carry leases or command
deadlines.

The cursor accepts completion only when request, program profile/group/order,
stage index and generation match. Cancel acceptance increments generation
before cleanup; late work is discarded. A profile switch or prepared-state
reload follows the same invalidation path.

## Performance harness

Dependency admission selects an exact benchmark framework only after
`cargo deny` and source/license review. The harness uses fixed checked-in
fixtures and separates setup, preparation and warm complete-call groups.
Portable tests use `allocation-counter` and deterministic work evidence.

Absolute timing runs only in an isolated Release profile with recorded CPU,
features, OS, compiler and load assumptions. It reports microsecond targets
and release floors separately. Timing results from one host are not used as
claims for another architecture.

## Cross-boundary proof

Integration tests construct collision-unchecked plans through every family and
show that `arm-control` physical admission cannot consume them. Separate tests
show task compilation never creates a lease or refreshes a deadline.
