# Advanced IK kernels — implementation plan

## 1. Preparation contracts

- [x] Load `trellis-before-dev` and relevant backend specs.
- [x] Add per-file provenance template, one-to-one KINETIC source mapping and a
      repository scan rejecting other IK implementation sources.
- [x] Add `PreparedKinematics`, geometry capabilities and bounded workspace
      constructors.
- [x] Add exact context/generation checks and portable allocation tests.
- [x] Replace/rebase DLS from KINETIC behind the prepared surface and preserve
      project-facing accepted behavior where the contracts agree.

## 2. Generic solvers

- [x] Implement FABRIK with bounded orientation refinement.
- [x] Implement SQP with projected limits and reusable matrices.
- [x] Implement seeded fixed-population Bio-IK.
- [x] Add solver-specific convergence/work evidence and all failure tests.

## 3. Analytical solvers

- [x] Implement geometry compatibility analysis.
- [x] Implement fixed-capacity OPW candidates and final validation.
- [x] Implement Paden-Kahan primitives and six-axis decomposition.
- [x] Implement redundant seven-DOF sampling/decomposition.
- [x] Add compatible and near-miss golden fixtures.

## 4. Cache, batch and selection

- [x] Implement bounded profile/group/generation-bound cache.
- [x] Add batch queries using one prepared context and caller-owned output.
- [x] Implement stable `Auto` selection and explicit no-fallback behavior.
- [x] Prove cache/reload/profile invalidation.

## 5. Gates

- [x] Add warm allocation and Release latency benchmarks.
- [x] Run `cargo fmt`, warnings-denied Clippy and targeted tests.
- [x] Run architecture and dependency/source policy.
- [x] Run `trellis-check` before planner children consume the prepared API.
- [ ] Commit and archive this child before planner children consume the
      prepared API.

Rollback after each section is the previous truthful capability state, never a
fallback alias.
