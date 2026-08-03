# Sampling and roadmap planners — implementation plan

## 1. Foundation

- [x] Verify advanced-IK child completion and prepared API stability.
- [x] Load `trellis-before-dev` and KINETIC provenance inventory.
- [x] Add validated problem/workspace, bounded node storage and family evidence.
- [x] Retain common direct-edge validation with explicit fast-path evidence.

## 2. Tree planners

- [x] Rewrite RRT-Connect against the bounded workspace.
- [x] Implement single-tree RRT* rewiring and informed sampling.
- [x] Implement bidirectional RRT* with independent rewiring.
- [x] Implement BiTRRT transition/temperature/frustration mechanics.
- [x] Add deterministic family-specific success/failure/budget tests.

## 3. Exploration and roadmap planners

- [x] Implement EST density-weighted selection.
- [x] Implement KPIECE projected-cell exploration.
- [x] Implement bounded PRM preparation, optional lazy validation and A* query.
- [x] Add profile/group/order/constraint-generation invalidation tests.

## 4. Performance and finish

- [x] Add post-preparation allocation checks and per-family benchmarks.
- [x] Meet prepared PRM <=200 us and nominal fast-query targets.
- [x] Run targeted Clippy/tests, architecture and dependency/source gates.
- [x] Run `trellis-check`, commit and archive independently.

Rollback removes only the failing named kernel and restores typed
unavailability. It never maps the name to RRT-Connect.
