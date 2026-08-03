# Arm Sampling and Roadmap Planning

## Scenario: Add or change a sampling planner

### 1. Scope / Trigger

Use this contract for code under
`crates/application_module/arm-algorithms/src/planning/` that changes
RRT-Connect, RRT*, bidirectional RRT*, BiTRRT, EST, KPIECE, PRM, prepared
planning identities, planner workspaces or family evidence.

KINETIC commit `3dbaa61579c4359ce4c35f246fc83e1d3110b646` is the
algorithm source for these seven kernels. Each adapted module records the
exact upstream file SHA-256. Project code owns types, prepared identity,
deterministic randomness, semantic validation, storage and performance; it
must not add a KINETIC dependency or route one named family through another.

### 2. Signatures

The reusable public boundary is:

```rust
let kinematics = PreparedKinematics::new(chain, model_generation)?;
let planning = PreparedPlanning::new(&kinematics, constraint_generation)?;
let mut workspace = planning.workspace(config)?;
let plan = planning.plan(kind, start, goal, config, validator, &mut workspace)?;

let roadmap = PreparedRoadmap::build(
    &planning,
    config,
    PrmQueryOptions::default(),
    validator,
    &mut workspace,
)?;
let plan = roadmap.query(&planning, start, goal, config, validator, &mut workspace)?;
```

`plan_joint_space` remains a compatibility wrapper for one-shot calls.
Repeated or latency-sensitive callers retain `PreparedKinematics`,
`PreparedPlanning` and `PlannerWorkspace`.

### 3. Contracts

- `PreparedPlanning` borrows the existing `PreparedKinematics`; it never
  invents a second profile identity. Its identity is profile, group, exact
  joint order, model generation and non-zero constraint generation.
- `PlannerWorkspace` is prepared from validated request capacities. Tree,
  neighbor, density, cell, path and A* scratch never grow in a warm kernel.
  An owned `ArmPlan` may allocate its returned tagged waypoint representation.
- Kernels reset `ChaCha8Rng` from the request's recorded 32-byte seed and never
  read a clock or implicit entropy. The caller enforces
  `maximum_duration_ns`; kernels enforce deterministic iteration, sample and
  aggregate-node ceilings.
- A valid direct edge may use the common `DirectEdge` fast path. A blocked
  direct edge must dispatch to the requested family and return its matching
  `PlannerEvidence` variant.
- RRT* variants perform radius-based parent selection and rewiring. BiTRRT
  records transition, rejection, annealing and frustration state. EST records
  inverse-density selection. KPIECE records projected-cell coverage and
  expansion outcomes.
- PRM construction and query are separate. A roadmap stores fixed-degree
  adjacency, construction statistics and the full prepared identity. Lazy
  mode defers edge validation to query and records each lazy edge check.
- Planner validation is semantic and independent of collision detection.
  Every result remains `ArmPlan<CollisionUnchecked>` and grants no authority.
- Coordinated dual-arm, floating-base and parallel-mechanism requests stay
  typed unsupported until their dedicated composite state spaces exist.

### 4. Validation and Error Matrix

| Condition | Required result |
|---|---|
| constraint generation is zero | `InvalidBudget { field: "planner.constraint_generation" }` |
| workspace or roadmap generation differs | `PreparedGenerationMismatch` |
| profile, group, order or DOF differs | `PreparedWorkspaceMismatch` or `JointOrderMismatch` |
| coordinate is non-finite/outside limits/wrong SI family | `NonFinite`, `JointLimit` or `UnitMismatch` |
| state or interpolated edge fails the validator | `ConstraintViolation` for endpoint admission; bounded search continues for rejected samples |
| iteration, sample or node ceiling ends search | `WorkBudgetExhausted` with consumed counters |
| workspace, path, cell or roadmap capacity is insufficient | `Capacity` |
| a deferred optimization planner needs a blocked-path kernel | `UnsupportedCapability` |

### 5. Tests Required

- A blocked-direct fixture executes all seven named kernels and asserts the
  matching evidence variant. No family test may pass solely through direct
  interpolation.
- Equal seeds reproduce the complete plan and counters. Start/goal rejection,
  impossible constraints, work exhaustion and deferred topology errors remain
  typed.
- RRT* evidence covers parent candidates/cost/rewires; BiTRRT, EST and KPIECE
  expose their distinct state counters.
- PRM tests cover graph reuse, fixed capacity, lazy validation and model plus
  constraint-generation invalidation before query.
- A warmed allocation test permits only owned result construction, not scratch
  growth.
- Run the ignored Release gate explicitly:

  ```text
  cargo test --release -p arm-algorithms --test planning_performance -- --ignored --nocapture
  ```

  Prepared PRM complete-call P95 must be at most 200 microseconds. Nominal
  successful family calls must be at most 1 millisecond. These host results are
  not target-board or collision-system qualification.
- Run `tools/ci.sh`, rustdoc with warnings denied, architecture checks and
  dependency/source policy.

### 6. Wrong vs Correct

#### Wrong

```rust
match planner {
    PlannerKind::RrtStar | PlannerKind::Est | PlannerKind::Prm => rrt_connect(...),
    _ => unimplemented!(),
}
```

#### Correct

```rust
let plan = planning.plan(
    PlannerKind::RrtStar,
    start,
    goal,
    config,
    validator,
    &mut workspace,
)?;
assert!(matches!(
    plan.statistics().evidence(),
    PlannerEvidence::RrtStar { .. }
));
```
