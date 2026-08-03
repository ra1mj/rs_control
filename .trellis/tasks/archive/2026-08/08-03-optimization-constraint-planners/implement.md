# Optimization and constrained planners — implementation plan

## 1. Contracts and preparation

- [x] Verify advanced-IK child completion.
- [x] Load `trellis-before-dev`, source audit and backend specs.
- [x] Add deterministic non-allocating cost/projection/region contracts.
- [x] Add prepared artifact identity, capacity and generation validation.

## 2. Trajectory optimization

- [x] Implement prepared CHOMP smoothness/factorization and bounded updates.
- [x] Implement seeded prepared STOMP populations and weighted updates.
- [x] Add objective, endpoint, limit, constraint and non-convergence tests.

## 3. Region and local planners

- [x] Implement bounded deterministic constraint-region preparation.
- [x] Implement prepared GCS graph/query/refinement.
- [x] Implement Cartesian pose stepping, prepared IK and jump/partial evidence.
- [x] Implement distinct projected constrained RRT.

## 4. Performance and finish

- [x] Add post-preparation allocation and complete-call Release benchmarks.
- [x] Meet prepared GCS <=200 us and nominal complete-call <=1,000 us targets.
- [x] Run targeted Clippy/tests, architecture and dependency/source gates.
- [x] Run `trellis-check`, commit and archive independently.

Rollback restores typed unavailability for the exact kernel. It must not use a
generic RRT/direct path under an optimization planner name.
