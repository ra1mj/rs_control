# Arm Planning

## Scenario: Add or change a joint, optimization or constrained planner

### 1. Scope / Trigger

Use this contract for code under
`crates/application_module/arm-algorithms/src/planning/` that changes a named
planner, prepared planning identity, reusable workspace, semantic constraint,
Cartesian path or planner evidence.

KINETIC commit `3dbaa61579c4359ce4c35f246fc83e1d3110b646` is the
algorithm source for RRT-Connect, RRT*, bidirectional RRT*, BiTRRT, EST,
KPIECE, PRM, CHOMP, STOMP, IRIS, GCS, Cartesian and constrained RRT. Every
adapted module records the exact upstream file SHA-256. Project code owns the
types, prepared identity, deterministic randomness, validation, storage and
performance boundary. It must not add a KINETIC dependency or route one named
family through another.

### 2. Signatures

The shared prepared joint-planning boundary is:

```rust
let kinematics = PreparedKinematics::new(chain, model_generation)?;
let planning = PreparedPlanning::new(&kinematics, constraint_generation)?;
let mut workspace = planning.workspace(config)?;
let plan = planning.plan(kind, start, goal, config, validator, &mut workspace)?;
```

Repeated optimization, region and Cartesian callers retain their preparation:

```rust
let optimization = OptimizationConfig::host_default(seed);
let mut optimization_workspace = OptimizationWorkspace::new(&planning, optimization)?;
let chomp = PreparedChomp::new(&planning, optimization)?;
let stomp = PreparedStomp::new(&planning, optimization)?;

let regions = PreparedConstraintRegions::build(
    &planning,
    region_seeds,
    IrisConfig::host_default(),
    region_provider,
)?;
let graph = PreparedRegionGraph::build(
    &planning,
    &regions,
    config,
    validator,
    &mut workspace,
)?;
let plan = graph.query(
    &planning,
    start,
    goal,
    config,
    validator,
    &mut workspace,
)?;

let mut cartesian = PreparedCartesianPlanner::new(
    &planning,
    maximum_waypoints,
    maximum_ik_population,
)?;
let diagnostic_path = cartesian.plan(
    &planning,
    start,
    target_pose,
    cartesian_config,
    validator,
)?;

let constrained = planning.plan_constrained(
    start,
    goal,
    config,
    manifold_projector,
    validator,
    &mut workspace,
)?;
```

`plan_joint_space` remains a one-shot compatibility wrapper. Cartesian partial
progress is a `CartesianPlan<CollisionUnchecked>`, never an executable
`ArmPlan`.

### 3. Contracts

- `PreparedPlanning` borrows the sole `PreparedKinematics` identity. Every
  artifact binds profile, group, exact joint order, model generation and
  non-zero constraint generation.
- Workspaces allocate bounded tree, graph, optimizer, population, IK and path
  scratch during preparation. Warm kernels may allocate the owned returned
  plan, but may not grow scratch storage.
- Kernels reset `ChaCha8Rng` from the recorded 32-byte request seed and never
  read a clock or implicit entropy. Callers enforce `maximum_duration_ns`;
  kernels enforce deterministic iteration, sample, population and node bounds.
- A valid direct edge may use `PlannerEvidence::DirectEdge`. A blocked edge
  dispatches to the requested family and returns its matching evidence.
- CHOMP uses a precomputed covariant smoothness metric and bounded line search.
  STOMP uses stable seeded raw-noise vectors, precomputed temporal covariance,
  bounded rollout weights and learning-rate-scaled updates. Endpoints never
  move, and success requires a finite improvement larger than tolerance.
- `TrajectoryCost`, `ManifoldProjector` and `RegionConstraintProvider`
  evaluation is deterministic and allocation-free. Each provider generation
  equals the planning constraint generation.
- IRIS regions are caller-certified closed semantic boxes. GCS stores a
  bounded prepared region graph, runs reusable A* scratch and validates every
  exact returned edge. Neither name implies collision qualification.
- Cartesian planning uses bounded translation/rotation steps, the existing
  prepared KINETIC IK boundary, previous-solution seeding, joint-jump limits and
  exact state/edge validation. Partial output reports progress and cannot be
  confused with an executable joint plan.
- Constrained RRT projects every random sample and every tree extension before
  semantic validation. Start and goal must already lie on the supplied
  manifold; it records projection calls, iterations and rejections separately.
- RRT* variants perform radius parent selection and rewiring. BiTRRT records
  transition/temperature state; EST records inverse density; KPIECE records
  projected cells. PRM construction and query remain separate.
- Every result remains collision unchecked. Coordinated dual-arm,
  floating-base and parallel-mechanism requests stay typed unsupported until
  dedicated composite state spaces exist.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| constraint/provider generation is zero | `InvalidBudget` |
| prepared model or constraint generation differs | `PreparedGenerationMismatch` |
| profile, group, order, DOF or prepared shape differs | `PreparedWorkspaceMismatch` or `JointOrderMismatch` |
| coordinate/config/cost is non-finite, out of limits or wrong SI family | `NonFinite`, `JointLimit` or `UnitMismatch` |
| endpoint or exact edge fails semantic validation | `ConstraintViolation` |
| optimizer cannot reduce a feasible seed above tolerance | `OptimizationNonConvergent` |
| endpoint projection fails or changes an off-manifold endpoint | `ProjectionFailed` |
| iteration, sample, population or node ceiling ends search | `WorkBudgetExhausted` |
| workspace, path, graph, optimizer or Cartesian capacity is insufficient | `Capacity` |
| Cartesian partial mode stops at IK, jump or constraint | diagnostic `CartesianTermination`, not executable success |
| deferred composite topology is requested | `UnsupportedCapability` |

### 5. Good / Base / Bad Cases

- Good: prepare one profile/group/generation, retain graph/optimizer/IK scratch,
  run repeated seeded calls, verify family evidence and invalidate everything
  after model or semantic generation changes.
- Base: use `plan_joint_space` for one deterministic direct or bounded
  joint-space request and retain `CollisionUnchecked`.
- Bad: call RRT-Connect under a CHOMP/GCS name, regenerate STOMP noise inside
  each covariance product, accept numerically unchanged optimization output,
  label IRIS regions collision free, return a Cartesian partial path as an
  `ArmPlan`, or reuse a prepared graph after profile reload.

### 6. Tests Required

- Blocked-direct fixtures execute every named family and assert matching
  `PlannerEvidence`; no family test may pass solely through direct interpolation.
- Equal seeds reproduce complete paths, costs, rollout updates and counters.
  Start/goal rejection, impossible constraints and work exhaustion stay typed.
- CHOMP/STOMP tests assert objective improvement, endpoint preservation,
  constraints, non-convergence and stable seeded STOMP rollouts.
- IRIS fixtures test every prepared corner against the caller provider. GCS
  tests cover graph reuse, exact path validation, scratch allocation and
  profile/group/order/model/constraint invalidation.
- Cartesian tests cover translation/rotation stepping, prepared IK failure,
  jump detection, exact bounds and diagnostic partial progress.
- Constrained-RRT tests cover projection success/failure, off-manifold
  endpoints, search exhaustion and evidence distinct from RRT-Connect.
- Run the ignored Release gates explicitly:

  ```text
  cargo test --release -p arm-algorithms --test planning_performance -- --ignored --nocapture
  cargo test --release -p arm-algorithms --test planning_optimization_performance -- --ignored --nocapture
  ```

  Prepared PRM and GCS query P95 must be at most 200 microseconds. Nominal
  complete family, GCS build+query, Cartesian, CHOMP/STOMP and constrained-RRT
  P95 must be at most 1 millisecond. These are host measurements, not target
  board, realtime or collision-system qualification.
- Run `tools/ci.sh`, rustdoc with warnings denied, architecture checks and
  dependency/source policy.

### 7. Wrong vs Correct

#### Wrong

```rust
match planner {
    PlannerKind::Chomp | PlannerKind::Gcs | PlannerKind::ConstrainedRrt => {
        rrt_connect(prepared, start, goal, config, validator, workspace)
    }
    _ => unimplemented!(),
}
```

#### Correct

```rust
let plan = planning.plan_constrained(
    start,
    goal,
    config,
    manifold_projector,
    validator,
    &mut workspace,
)?;
assert!(matches!(
    plan.statistics().evidence(),
    PlannerEvidence::ConstrainedRrt {
        projection_calls: 1..,
        ..
    }
));
```

## Scenario: Compile bounded task stages and report host performance

### 1. Scope / Trigger

Use this contract when converting an `ArmTaskProgram` stage to a semantic
proposal, adding task cancellation/reload correlation, exposing prepared-byte
evidence, or changing an isolated Release algorithm benchmark.

### 2. Signatures

```rust
let compiler = ArmTaskCompiler::new(&program, &planning, policy)?;
let mut workspace = compiler.workspace()?;
let request = cursor.begin_compile(compiler.context().clone())?;
let result = compiler.compile(request, &start, validator, &mut workspace)?;
cursor.accept_compilation(&result, compiler.context())?;

workspace.allocated_bytes();
```

The shared Release test report takes one named kernel, warm duration samples,
`allocation_counter::AllocationInfo`, prepared bytes and a deterministic work
counter. Its optional baseline inputs are:

```text
ARM_ALGORITHMS_BASELINE_HOST=<exact emitted host fingerprint>
ARM_ALGORITHMS_BASELINE_<KERNEL>_P50_NS=<positive integer>
```

### 3. Contracts

- A request records non-zero request ID, stage index, cursor generation,
  profile, group, exact order, model generation and constraint generation.
  Cancellation, failure and reload increment the cursor generation before any
  late result can be accepted.
- `ArmTaskCompiler` accepts only an exactly matching request/context and a
  `TaskCompileWorkspace` bound to the same prepared planning identity.
- `MoveJoint`, `MovePose`, `Cartesian`, `Pick` and `Place` return only
  `CollisionUnchecked` plans; `Gripper` and `Hold` remain semantic intents.
  Proposals carry no lease, command deadline, command envelope or device
  bytes. `Recovery` remains typed unavailable until dispatch is designed.
- Workspace capacity is fixed at preparation. A larger Cartesian policy used
  with a smaller workspace returns `Capacity`; it must not grow scratch.
- Every ignored Release benchmark prints P50/P95/P99, throughput, total output
  allocations/bytes, prepared bytes, work count, host fingerprint and
  `qualification=host-only`. Output allocation for a returned plan is reported
  separately from prepared scratch and is not hidden as a warm-path allocation.
- A median regression gate is inactive without a baseline. If a baseline is
  supplied, its host fingerprint must match exactly and P50 greater than 115%
  of baseline fails. Results never compare unlike hosts or profiles.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Request/context differs from compiler | `StaleTaskResult` |
| Prepared model or constraint generation differs | `PreparedGenerationMismatch` or `StaleTaskResult` |
| Workspace context differs | `StaleTaskResult` |
| Stage index is outside the program | `StaleTaskResult` |
| Cartesian request exceeds prepared waypoints | `Capacity { field: "planner.cartesian_waypoints", .. }` |
| Late result arrives after cancellation/reload | `StaleTaskResult`; cursor does not advance |
| Recovery stage reaches compiler | `UnsupportedCapability { capability: "task_recovery_branch_dispatch" }` |
| Baseline P50 is malformed/zero | isolated Release test fails before comparison |
| Baseline host differs | isolated Release test fails rather than compare data |

### 5. Good, Base and Bad Cases

- Good: compile a stage against retained prepared kinematics/planning, record
  result identity, then accept exactly that result while the cursor is running.
- Base: compile `Hold` to a zero-allocation semantic proposal.
- Bad: let a late result renew authority, attach a lease/deadline to a task
  proposal, treat `CollisionUnchecked` as physically executable, or label a
  workstation benchmark as a target-board result.

### 6. Tests Required

- Cover every supported stage, task cancellation/reload rejection, compiler
  context mismatch, workspace mismatch and Cartesian capacity refusal without
  a change in `allocated_bytes()`.
- Keep a `compile_fail` proof that `ArmTaskProposal` cannot satisfy
  `PhysicalArmExecutor`, alongside existing arm-control lease/deadline
  admission tests.
- For every named IK, planning, optimization, gravity, reactive and task
  kernel, run its ignored Release report with `--nocapture`; retain output only
  as host evidence and assert the documented P95 target separately.
- Run `cargo deny check`, architecture check and `tools/ci.sh` after changing
  the report convention or prepared workspace accounting.

### 7. Wrong vs Correct

#### Wrong

```rust
// A task proposal has no authority and cannot bypass collision typestate.
physical_executor.execute(&compiled.proposal());
```

#### Correct

```rust
let compiled = compiler.compile(request, &start, validator, &mut workspace)?;
assert!(matches!(compiled.proposal(), ArmTaskProposal::Hold { .. }));
// A separate collision, lease, deadline and arm-control admission path is required.
```
