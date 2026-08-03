# Distinct sampling and roadmap planners

## Goal

Replace the shared bidirectional expansion fallback with independent bounded
RRT-Connect, RRT*, bidirectional RRT*, BiTRRT, EST, KPIECE and PRM kernels,
using the completed prepared-kinematics contracts.

## Dependency

`08-03-advanced-ik-kernels` must be committed and archived first. This child
must not duplicate chain preparation or introduce a second profile identity.

## Requirements

- **SP-01:** Each public planner owns distinct search state and transition
  logic matching its named KINETIC-informed family.
- **SP-02:** Share only validated primitives such as metrics, interpolation,
  semantic state/edge checks, seeded sampling and bounded storage.
- **SP-03:** Use explicit `ChaCha8Rng` request seeds and deterministic work
  budgets; no wall-clock or implicit entropy inside kernels.
- **SP-04:** Preallocate node, neighbor, grid and path storage from validated
  capacities; warm iterations cannot grow containers.
- **SP-05:** RRT* variants expose parent choice, rewiring and best-cost
  evidence; BiTRRT exposes transition/temperature evidence; EST exposes
  density evidence; KPIECE exposes cell/coverage evidence.
- **SP-06:** PRM separates bounded roadmap preparation from online query and
  binds the roadmap to profile/group/order/constraint generation.
- **SP-07:** A common direct-edge precheck is permitted, but blocked-direct
  tests must execute and distinguish every family kernel.
- **SP-08:** Every plan is finite, bounded, context-bound and
  `CollisionUnchecked`; planners never grant authority.
- **SP-09:** Prepared PRM nominal complete-call P95 is <=200 us. Other simple
  warm planner calls target <=1,000 us where the family finds a solution;
  adversarial search obeys the 50 ms orchestration deadline and typed work
  exhaustion.

## Acceptance Criteria

- [x] Family-specific fixtures and statistics prove no identity aliases
      RRT-Connect or direct interpolation.
- [x] Identical seeds reproduce complete plans and counters; changed seeds are
      recorded and remain within all constraints.
- [x] Start/goal/edge invalidity, bad seed/config, capacity and every budget
      exhaustion mode are tested.
- [x] RRT* variants demonstrate cost/rewiring behavior; BiTRRT transition state,
      EST density, KPIECE cell coverage and PRM reuse are each exercised.
- [x] Roadmap identity/generation mismatch fails before query.
- [x] Warm allocation and Release performance gates meet SP-09.
- [x] No collision-unchecked plan reaches physical admission.

## Out of Scope

Collision detection, dual-arm, floating-base, optimization planners and
hardware qualification are outside this child.
