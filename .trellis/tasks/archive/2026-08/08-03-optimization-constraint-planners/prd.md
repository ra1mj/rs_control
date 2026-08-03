# Optimization and constrained planners

## Goal

Implement distinct project-native CHOMP, STOMP, IRIS, GCS, Cartesian and
constrained-RRT kernels with explicit preparation/online-query boundaries and
microsecond-class warm nominal calls.

## Dependency

`08-03-advanced-ik-kernels` must be committed and archived first. This child
uses its prepared FK/Jacobian/IK state. It does not depend on completion of the
sampling/roadmap child except for shared primitives already stabilized in the
parent design.

## Requirements

- **OP-01:** CHOMP uses a precomputed smoothness metric/factorization and
  bounded covariant gradient updates over a valid seed trajectory.
- **OP-02:** STOMP uses explicit seeded fixed noise samples, bounded population
  cost evaluation and probability-weighted updates.
- **OP-03:** IRIS builds conservative caller-constraint regions during bounded
  preparation. It makes no collision-free claim.
- **OP-04:** GCS performs bounded graph search/refinement over prepared regions
  and validates the exact constraint/profile/group generation on every query.
- **OP-05:** Cartesian planning uses bounded pose steps, prepared IK,
  continuity/jump checks and exact joint/path validation.
- **OP-06:** Constrained RRT has distinct seeded sampling and bounded manifold
  projection; setup timing is not presented as solve timing.
- **OP-07:** Cost, gradient, projection and region-provider contracts are
  caller-owned, deterministic, non-allocating during evaluation and have
  explicit capacity/iteration bounds.
- **OP-08:** Complete warm nominal P95 targets are <=200 us for prepared GCS
  query and <=1,000 us for GCS build+query, Cartesian, nominal CHOMP/STOMP and
  nominal projected constrained planning.
- **OP-09:** Every result is context-bound and `CollisionUnchecked`; IRIS/GCS
  terminology and evidence cannot imply collision qualification.

## Acceptance Criteria

- [x] CHOMP and STOMP have distinct objective-improvement/non-convergence tests
      and preserve endpoints/limits/constraints.
- [x] Seeded STOMP repeats complete samples, costs, updates and results.
- [x] IRIS regions contain only points accepted by the supplied semantic
      provider in tested fixtures and are never typed collision checked.
- [x] Prepared GCS queries exercise region graph search and meet OP-08.
- [x] Cartesian tests cover pose interpolation, IK failure, jump detection,
      partial paths and exact bounds.
- [x] Constrained RRT tests cover projection success/failure, search exhaustion
      and behavior distinct from unconstrained RRT-Connect.
- [x] Profile/group/order/constraint-generation changes invalidate every
      prepared artifact.
- [x] Post-preparation allocation, latency, architecture, Clippy and test gates
      pass.

## Out of Scope

Collision construction/qualification, GPU optimization, dual-arm,
floating-base and hardware tests remain outside this child.
