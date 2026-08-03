# Optimization and constrained planners — technical design

## Constraint contracts

Introduce narrow generic contracts for bounded state feasibility, manifold
projection, differentiable trajectory cost/gradient and conservative region
membership. Evaluation methods receive caller-owned output scratch so they do
not allocate. Each provider has an immutable generation included in prepared
artifact identity.

## Prepared optimizers

`PreparedChomp` stores the validated waypoint/DOF shape, smoothness matrix,
factorization and gradient/update buffers. `PreparedStomp` additionally stores
the deterministic noise matrix and sample/cost/weight buffers. Endpoints are
immutable; all intermediate updates are projected through limits and the
caller constraint.

Results contain initial/final cost, completed iterations/evaluations, limiting
evidence and convergence reason. Invalid or higher-cost output is never
returned as success.

## Regions and GCS

`ConstraintRegion` stores bounded halfspaces, center and provider generation.
`PreparedRegionGraph` stores regions, overlap edges and reusable bounded A*
scratch. Region preparation and graph build are timed separately from the warm
query. Refinement revalidates the full path against the same provider.

Without collision evidence, names, docs and typestate state only
constraint-feasible/collision-unchecked behavior.

## Cartesian and constrained RRT

Cartesian planning interpolates poses into bounded caller-owned storage and
solves each pose with prepared IK using the prior solution as seed. It checks
joint jumps and can return typed partial-progress evidence without presenting
the partial result as executable.

Constrained RRT owns a bounded tree and seeded RNG but projects every sample and
extension through the constraint provider. It records projection iterations
and failures separately from tree work.

## Performance

Prepared calls reuse all matrices, graph queues, pose/joint buffers and sample
populations. Benchmarks separate construct/prepare/query, validate complete
calls, and use isolated Release measurements. The 1,000 us target applies to
bounded nominal fixtures; adversarial requests return typed exhaustion by the
outer deadline.
