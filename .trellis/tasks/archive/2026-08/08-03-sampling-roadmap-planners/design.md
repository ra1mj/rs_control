# Sampling and roadmap planners — technical design

## Common validated core

`PlanningProblem` validates chain context, start/goal vectors, limits, seed,
work capacities and the semantic validator once. `PlannerWorkspace` owns
preallocated node states, parents, costs, scratch neighbors, path scratch and a
request-reset deterministic RNG. Planner kernels receive these narrow values
and cannot allocate new unbounded collections.

The direct-edge feasibility precheck is common optimization, not planner
identity evidence. A result records whether the fast path or family kernel
executed.

## Independent kernels

- RRT-Connect alternates start/goal trees and uses extend/connect states.
- RRT* owns one cost tree, radius-based near set, best-parent selection and
  subtree cost propagation on rewire.
- Bidirectional RRT* owns two such trees and evaluates bounded cross-tree
  connections.
- BiTRRT owns cost, temperature, rejection streak and frustration state with a
  seeded Boltzmann transition test.
- EST owns neighborhood-density estimates and inverse-density selection.
- KPIECE owns a bounded projected-cell table with coverage, selections,
  successes, failures and border classification.
- PRM owns a prepared graph with bounded nearest connections and online A*;
  lazy validation is an explicit mode with generation accounting.

Nearest-neighbor search begins as deterministic bounded linear/vectorized
search for small capacities. An optional prepared spatial index may be added
only if benchmarks prove benefit without changing result ordering.

## Evidence and performance

`PlannerStatistics` expands to family-neutral counters plus a typed
family-specific evidence enum. Tests require the expected evidence variant.
Prepared PRM query is benchmarked separately from roadmap build. Allocation
tests reset and reuse workspaces between runs.

All state validity is collision independent, so output typestate remains
`CollisionUnchecked` even when the caller validator rejects semantic regions.
