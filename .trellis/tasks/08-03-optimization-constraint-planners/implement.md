# Optimization and constrained planners — implementation plan

## 1. Contracts and preparation

- [ ] Verify advanced-IK child completion.
- [ ] Load `trellis-before-dev`, source audit and backend specs.
- [ ] Add deterministic non-allocating cost/projection/region contracts.
- [ ] Add prepared artifact identity, capacity and generation validation.

## 2. Trajectory optimization

- [ ] Implement prepared CHOMP smoothness/factorization and bounded updates.
- [ ] Implement seeded prepared STOMP populations and weighted updates.
- [ ] Add objective, endpoint, limit, constraint and non-convergence tests.

## 3. Region and local planners

- [ ] Implement bounded deterministic constraint-region preparation.
- [ ] Implement prepared GCS graph/query/refinement.
- [ ] Implement Cartesian pose stepping, prepared IK and jump/partial evidence.
- [ ] Implement distinct projected constrained RRT.

## 4. Performance and finish

- [ ] Add post-preparation allocation and complete-call Release benchmarks.
- [ ] Meet prepared GCS <=200 us and nominal complete-call <=1,000 us targets.
- [ ] Run targeted Clippy/tests, architecture and dependency/source gates.
- [ ] Run `trellis-check`, commit and archive independently.

Rollback restores typed unavailability for the exact kernel. It must not use a
generic RRT/direct path under an optimization planner name.
