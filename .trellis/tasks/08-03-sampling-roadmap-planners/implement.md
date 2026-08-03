# Sampling and roadmap planners — implementation plan

## 1. Foundation

- [ ] Verify advanced-IK child completion and prepared API stability.
- [ ] Load `trellis-before-dev` and KINETIC provenance inventory.
- [ ] Add validated problem/workspace, bounded node storage and family evidence.
- [ ] Retain common direct-edge validation with explicit fast-path evidence.

## 2. Tree planners

- [ ] Rewrite RRT-Connect against the bounded workspace.
- [ ] Implement single-tree RRT* rewiring and informed sampling.
- [ ] Implement bidirectional RRT* with independent rewiring.
- [ ] Implement BiTRRT transition/temperature/frustration mechanics.
- [ ] Add deterministic family-specific success/failure/budget tests.

## 3. Exploration and roadmap planners

- [ ] Implement EST density-weighted selection.
- [ ] Implement KPIECE projected-cell exploration.
- [ ] Implement bounded PRM preparation, optional lazy validation and A* query.
- [ ] Add profile/group/order/constraint-generation invalidation tests.

## 4. Performance and finish

- [ ] Add post-preparation allocation checks and per-family benchmarks.
- [ ] Meet prepared PRM <=200 us and nominal fast-query targets.
- [ ] Run targeted Clippy/tests, architecture and dependency/source gates.
- [ ] Run `trellis-check`, commit and archive independently.

Rollback removes only the failing named kernel and restores typed
unavailability. It never maps the name to RRT-Connect.
