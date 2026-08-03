# Advanced arm algorithms — technical design

## 1. Scope and task decomposition

This task is an integration parent. Four children own independently verifiable
deliverables:

1. `08-03-advanced-ik-kernels`
2. `08-03-sampling-roadmap-planners`
3. `08-03-optimization-constraint-planners`
4. `08-03-arm-reactive-task-performance`

Children 2 and 3 depend on the prepared kinematic evaluation surface from
child 1. Child 4 depends on all three algorithm children. Tree position does
not imply these dependencies; each child artifact records them explicitly.

## 2. Source and ownership boundary

KINETIC commit `3dbaa61579c4359ce4c35f246fc83e1d3110b646`
is the reviewed algorithm reference. For IK it is the exclusive implementation
source: every solver kernel, including DLS, is adapted from the corresponding
reviewed KINETIC file. No other IK library or independently substituted solver
kernel is permitted. Planning kernels also use the reviewed KINETIC algorithms
as their selected reference. All integration contracts are owned here:

- `arm-model` owns compiled morphology, group, joint, limit and transmission
  truth;
- `arm-algorithms` owns kinematics, planning, trajectory, dynamics, reactive,
  grasp and task semantics;
- `message-model` remains the only owner of cross-layer semantic identities and
  SI joint values;
- `arm-control` remains the authority/admission boundary;
- protocol and hardware crates remain unaware of IK/planner implementation
  details.

No KINETIC crate, path, Git source, submodule, public type or runtime object
crosses these boundaries. Source exclusivity does not mean runtime coupling:
the algorithms are maintained as project-native adapted code. Any substantially
adapted file carries the upstream repository, commit, source path, SHA-256,
Apache-2.0 notice and local modification summary.

## 3. Prepared-state architecture

Microsecond-class online behavior requires preparation to be explicit.

```text
CompiledArmModel + KinematicGroup
        |
        v
PreparedKinematics(profile, group, order, generation)
  - canonical chain transforms/axes/limits
  - geometry compatibility (generic, OPW, subproblem, 7-DOF)
  - reusable FK/Jacobian/numerical workspaces
        |
        +--> PreparedIkCache / analytical solver data
        +--> PlannerWorkspace(capacity, seed, counters)
        +--> PreparedRoadmap(profile/group/constraint generation)
        +--> PreparedRegionGraph(profile/group/constraint generation)
        +--> PreparedOptimizer(factorizations, noise/sample buffers)
```

Prepared objects are immutable except for caller-owned scratch storage and
monotonic generation counters. Every query validates exact profile ID, group
ID, joint order, DOF and prepared-generation identity before reading cached
state. Reload/profile switch invalidates all derived artifacts; mismatches are
typed errors, never cache misses that silently rebuild on the online path.

Preparation has explicit byte, node, region, sample and matrix-size ceilings.
It may allocate within those validated bounds. A prepared query must not grow a
container. Fixed-capacity outputs use existing semantic limits such as
`MAX_JOINTS` and `MAX_TRAJECTORY_POINTS`.

## 4. IK design

Each `IkSolverKind` owns a distinct kernel:

- DLS: replace/rebase the existing local kernel with the reviewed KINETIC DLS,
  adapted to reusable factored scratch matrices and deterministic step buffers;
- FABRIK: forward/backward link-point passes followed by bounded orientation
  refinement, with explicit compatibility for active joint kinds;
- SQP: bounded constrained pose objective using preallocated Jacobian/Hessian
  approximation and projected joint-limit steps;
- Bio-IK: seeded bounded population search with fixed-capacity population,
  deterministic selection/mutation and explicit multi-objective weights;
- OPW: fixed-size analytical candidate generation for verified six-axis
  spherical-wrist geometry;
- Paden-Kahan/subproblem: fixed-size subproblem candidates for compatible
  six-axis geometry;
- redundant seven-DOF: bounded redundant-coordinate sampling plus analytical
  six-axis subproblem, with the seed and sample count in evidence.

`Auto` chooses the fastest compatible prepared solver in stable order:
analytical/cache hit, specialized seven-DOF, SQP/FABRIK, then DLS. It records
the actual selected solver. An explicit incompatible solver fails with
`UnsupportedGeometry`; it never falls back.

If the reviewed KINETIC implementation of a solver is incomplete or cannot be
adapted without violating project contracts, that solver stays explicitly
unavailable. An implementation from another source is not an allowed fallback.

Solution collections use fixed capacity. All candidates are re-evaluated with
the project FK, finite/unit/limit checks and requested pose tolerances before
being returned.

## 5. Sampling and roadmap planner design

All planners share only validated primitives: metric, interpolation, semantic
state/edge validation, seeded sampling, nearest-neighbor helpers, bounded node
storage and plan construction. Their search policies remain distinct:

- RRT-Connect: alternating bidirectional extend/connect;
- RRT*: single-tree cost tracking, near-radius parent choice and rewiring;
- bidirectional RRT*: two independently rewired trees and best connection;
- BiTRRT: bidirectional transition tests with deterministic temperature and
  frustration state;
- EST: density-weighted node selection and neighborhood expansion;
- KPIECE: projection grid, cell importance/coverage and border preference;
- PRM: bounded prepared roadmap construction and A*/Dijkstra query, with lazy
  validation as an explicit mode.

A common direct-edge fast path is allowed only as a documented validated
precheck. Tests with a blocked direct edge must prove each named planner
executes family-specific state transitions and statistics. The result records
planner identity, seed, work counters and path cost.

Randomness always comes from request-owned `ChaCha8Rng` seeded by the validated
configuration. Kernels use deterministic work ceilings; orchestration owns a
separate monotonic deadline and discards late results.

## 6. Optimization and constrained planner design

- CHOMP precomputes the smoothness metric/factorization and performs bounded
  covariant gradient updates over a caller-provided valid seed path.
- STOMP precomputes seeded noise samples and smoothing matrices, evaluates a
  fixed sample population and applies bounded probability-weighted updates.
- IRIS compiles caller-supplied semantic feasibility samples into conservative
  convex constraint regions. Without collision evidence the regions are named
  `ConstraintRegion`, not collision-free regions.
- GCS queries a prepared overlap graph with bounded A* and refines the region
  sequence under the same semantic constraint provider.
- Cartesian planning performs bounded pose interpolation, compatible prepared
  IK at each sample, continuity/jump checks and exact limit validation.
- constrained RRT performs seeded sampling, bounded constraint projection and
  distinct constrained edge validation.

IRIS/roadmap construction is a preparation operation. GCS/PRM online queries
must not rebuild their graph. CHOMP/STOMP online calls reuse prepared matrices
and buffers. Every result remains `ArmPlan<CollisionUnchecked>`.

## 7. Constraint and collision evidence

`ArmStateValidator` remains the collision-independent semantic state contract.
It is extended or paired with bounded, deterministic contracts for:

- state feasibility;
- edge feasibility at an explicit resolution;
- projection onto a constraint manifold;
- differentiable cost/gradient where required;
- convex-region membership/overlap evidence.

These traits are generic/static-dispatch on hot paths. They must not allocate,
format or perform I/O during an evaluation. A validator cannot manufacture
`CollisionChecked`; only a later qualified collision provider may do that.

## 8. Task and reactive integration

The existing `ArmTaskProgram` remains a semantic program. A new bounded task
compiler converts one active stage into a context-bound proposal:

```text
stage + prepared algorithms + exact context
  -> pending request identity/generation
  -> CollisionUnchecked plan/trajectory/gripper/hold proposal
  -> task cursor accepts completion only for the same request and context
```

MoveJoint, MovePose, Cartesian, Pick and Place use the prepared solver/planner
surfaces. Gripper and Hold remain semantic intents. Cancel acceptance bumps the
request generation immediately; late results from the previous generation are
discarded and cannot advance the cursor or renew authority.

RMP adds a singularity-aware damping policy using prepared Jacobian evidence.
Reach, damping, joint-limit and singularity contributions are separately
bounded before combination. Obstacle/self-collision policies continue to
return `CollisionCapabilityUnavailable`.

## 9. Performance design

### Measurement classes

- Preparation: bounded and reported separately; never hidden in a warm query.
- Warm nominal complete call: includes semantic validation and result
  construction and is the microsecond performance target.
- Adversarial search: has deterministic work bounds and a caller-owned 50 ms
  deadline; success is not guaranteed.

### Targets

| Surface | Warm nominal P95 target |
|---|---:|
| analytical/cache-hit IK | 50 us |
| numerical IK | 500 us |
| prepared GCS/PRM query | 200 us |
| GCS build+query, Cartesian, nominal CHOMP/STOMP, nominal projected constrained plan | 1,000 us |
| reusable core iteration | 10 us |

P95 10 ms and P99 20 ms are release-blocking floors, not targets. The
adversarial deadline is 50 ms. Benchmark evidence records CPU, target features,
OS, Rust version, profile, fixture identity, sample count, latency distribution,
allocation count, prepared bytes and deterministic work counters. RK3588 and
physical timing remain explicitly unqualified.

Absolute timing is checked only by an opt-in isolated Release performance gate
to avoid noisy debug CI claims. Default host CI enforces all portable proxies:
fixed capacities, no post-preparation growth/allocation, deterministic work
counts, cache hits/misses and result identity. A same-host median regression
above 15% fails the isolated performance gate.

## 10. Failure and diagnostic model

Errors remain typed `ArmAlgorithmError` variants or validated evidence values:

- incompatible geometry or joint kind;
- dimension/unit/profile/group/order/generation mismatch;
- workspace/candidate/node/region capacity exhaustion;
- invalid seed/configuration;
- constraint projection or objective non-convergence;
- singular/degraded solution;
- work budget/deadline exhaustion;
- collision capability unavailable.

Libraries emit no stdout/stderr. Saturating counters and immutable result
statistics are returned to the caller. Formatted benchmark output belongs to a
bench/tool process boundary only.

## 11. Compatibility, rollout and rollback

Existing DLS, direct planning, RMP and task APIs remain source-compatible where
their current semantics are truthful. New prepared APIs are additive. Existing
advanced names change from typed unavailable or generic fallback behavior to
their own kernels; tests pin the new behavior.

Each child is independently revertible. Removing one advanced kernel restores
typed unavailability for that exact capability; rollback must never redirect
its public name to another algorithm. The final integration child can be
reverted without removing completed solver/planner kernels.
