# Project-native advanced IK kernels

## Goal

Implement the KINETIC-informed advanced inverse-kinematics algorithms as
distinct, deterministic project-native kernels with prepared morphology-bound
state and microsecond-class warm queries.

## Background

The current crate implements a project-local DLS only. FABRIK, SQP, Bio-IK, OPW,
Paden-Kahan/subproblem and redundant seven-DOF are advertised but report typed
unavailability. KINETIC commit
`3dbaa61579c4359ce4c35f246fc83e1d3110b646` supplies reviewed Apache-2.0
reference implementations and is the sole permitted IK kernel source,
including for DLS. It uses its own types, implicit randomness and heap
allocation patterns that cannot cross into this project.

## Requirements

- **IK-00:** Adapt every IK kernel exclusively from the reviewed KINETIC commit.
  Do not retain an independently sourced DLS or add another IK library. If an
  upstream path cannot satisfy the contract, leave it typed unavailable.
- **IK-01:** Rebase DLS and implement FABRIK, SQP, Bio-IK, OPW,
  Paden-Kahan/subproblem and redundant seven-DOF as separate KINETIC-derived
  kernels; no solver name may route through DLS.
- **IK-02:** Add prepared profile/group/order-bound kinematic geometry,
  compatibility and reusable scratch workspaces with explicit capacities.
- **IK-03:** Add a bounded cache whose keys include target quantization and
  whose identity includes profile, group, joint order and preparation
  generation.
- **IK-04:** Use caller-provided recorded seeds for randomized search. Identical
  prepared state, request and seed must produce identical candidates/evidence.
- **IK-05:** Analytical solvers use fixed-capacity candidates and validate every
  candidate with project FK, limits and tolerance before return.
- **IK-06:** Generic solvers support compatible variable-DOF groups and exact
  joint kinds; specialized solvers reject incompatible geometry explicitly.
- **IK-07:** `Auto` selects the fastest compatible prepared solver in stable
  order and reports the actual kernel. Explicit requests never fall back.
- **IK-08:** Warm complete-call P95 targets are <=50 us for analytical/cache-hit
  IK and <=500 us for numerical IK; the accepted release floor remains 5 ms.
- **IK-09:** Preparation may allocate within a checked byte ceiling. A warm
  solve may reuse scratch but may not grow containers.
- **IK-10:** Substantially adapted kernels carry exact Apache-2.0 provenance and
  modification records.

## Acceptance Criteria

- [ ] Every solver has reachable/unreachable, limit, singular, wrong-dimension,
      NaN/Inf and deterministic-repeat tests.
- [ ] Multi-DOF and mixed-joint fixtures prove generic compatibility rules.
- [ ] Dedicated six-axis and seven-axis fixtures prove analytical compatibility
      and near-miss rejection.
- [ ] Solver-specific statistics prove no advanced solver aliases DLS.
- [ ] Source/provenance review proves every IK kernel maps to one reviewed
      KINETIC source file and no alternative IK implementation is present.
- [ ] Cache hits are deterministic; profile/group/order/generation changes
      reject old entries rather than returning or rebuilding silently.
- [ ] Fixed candidate/workspace capacities fail with typed evidence.
- [ ] Warm allocation and isolated Release latency gates meet IK-08.
- [ ] Architecture, Clippy, tests and dependency/source gates pass.

## Out of Scope

Collision, dual-arm, floating-base, closed-chain, hardware and target-board
qualification remain outside this child.
