# Advanced IK kernels — technical design

## Boundary

All code remains in `arm-algorithms::kinematics`. Existing project model and
semantic joint vectors remain authoritative. KINETIC types and dependencies do
not enter the crate, but its reviewed solver files are the exclusive source of
IK kernel behavior. Adaptations may harden validation, determinism, storage and
performance without substituting algorithms from another source.

## Modules and contracts

Refactor the current `ik.rs` surface behind an `ik/` module with internal
kernel modules for DLS, FABRIK, SQP, Bio-IK and analytical subproblems. Keep the
public `IkSolverKind`, capabilities and solution semantics source-compatible.

`PreparedKinematics` owns immutable profile/group/order identity, compact chain
geometry, limit arrays and solver compatibility. `IkWorkspace` owns validated
scratch buffers and a `ChaCha8Rng` reset from each request seed. The two are
separate so immutable prepared data can be shared while scratch remains
single-owner.

`GeometryCapabilities` records OPW, six-axis subproblem and redundant seven-DOF
compatibility plus rejection reasons. It is computed once from project model
axes/origins with explicit tolerances and is part of the prepared-generation
identity.

`IkCandidates` uses fixed capacity. A common finalizer validates project FK,
finite residuals, exact joint order and limits, then stable-sorts by residual,
seed distance and lexicographic coordinates.

## Algorithms

- FABRIK performs forward/backward point passes and bounded orientation
  refinement.
- SQP uses a projected bounded pose objective with reusable Jacobian and normal
  equation scratch.
- Bio-IK uses a fixed population, deterministic tournament/selection, seeded
  mutation and explicit objective weights.
- OPW and Paden-Kahan use fixed-size arrays for analytical candidates.
- Seven-DOF subproblem samples the redundant coordinate from a bounded seeded
  sequence and invokes the verified six-axis decomposition.

The existing local DLS kernel is replaced/rebased on the reviewed KINETIC DLS
implementation and moved to the same prepared workspace so all IK solvers share
one source policy and benchmark conditions.

## Cache and performance

The cache is bounded by entry count and bytes, uses deterministic quantized
pose keys and never evicts implicitly during a warm lookup. Warm lookup and
analytical solve include context validation and result construction in timing.
Tests use `allocation-counter`; isolated Release benchmarks record P50/P95/P99.

Rollback restores exact typed unavailability for an unfinished kernel. It must
not change the public name to an alias.
