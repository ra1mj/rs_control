# Arm Kinematics and Inverse Kinematics

## Scenario: Add or change an arm IK kernel

### 1. Scope / Trigger

Use this contract for code under
`crates/application_module/arm-algorithms/src/kinematics/` that changes inverse
kinematics, prepared chain geometry, reusable workspaces, analytical geometry
admission, IK caching or batch solving.

KINETIC commit `3dbaa61579c4359ce4c35f246fc83e1d3110b646` is the sole
permitted source for every IK algorithm kernel, including DLS. Project code may
adapt types, validation, deterministic randomness, bounded storage, caching and
performance. It must not add a KINETIC dependency, vendor its workspace or
substitute another IK implementation.

### 2. Signatures

The prepared public boundary is:

```rust
PreparedKinematics::new(chain, generation) -> Result<PreparedKinematics, ArmAlgorithmError>
PreparedKinematics::workspace(maximum_population) -> Result<IkWorkspace, ArmAlgorithmError>
PreparedKinematics::workspace_with_byte_limit(maximum_population, maximum_bytes)
PreparedKinematics::solve(solver, target, initial, config, workspace)
PreparedKinematics::solve_cached(solver, target, initial, config, workspace, cache)
PreparedKinematics::solve_batch(solver, targets, initials, config, workspace, output)
IkCache::new(prepared, maximum_entries, maximum_bytes, translation_resolution, rotation_resolution)
```

`KinematicChain::solve_ik` is a compatibility wrapper. New repeated or
performance-sensitive callers use the prepared boundary.

### 3. Contracts

- Prepared identity contains profile, group, exact joint order and a non-zero
  caller-managed generation.
- `IkWorkspace` belongs to exactly one prepared identity and owns all warm
  scratch. Warm calls may mutate and reuse it but may not grow a container.
- Randomized kernels reset `ChaCha8Rng` from the request's recorded 32-byte
  seed. Equal prepared state, target, initial state, config and seed produce
  byte-equal semantic results and work evidence.
- A population kernel ranks preallocated members with an allocation-free
  in-place ordering. Its comparator must total-order fitness and a persistent
  member ordinal: Rust's stable `sort_by` may allocate a temporary buffer in a
  warm generation.
- Generic DLS and SQP accept variable-DOF revolute, continuous and prismatic
  groups. FABRIK accepts rotary groups. Bio-IK requires an explicit seed.
- OPW accepts only its measured Y-Z-Y wrist convention. Paden-Kahan accepts
  measured spherical Z-Y-Z wrists. Seven-axis decomposition accepts the
  corresponding redundant spherical-wrist convention. DOF alone is never
  sufficient geometry evidence.
- Every analytical candidate is checked with project FK, exact joint limits,
  finite residuals and the caller tolerances before return.
- `Auto` uses stable priority: OPW, Paden-Kahan, redundant seven-axis, DLS.
  Explicit solver requests never fall back or alias another named kernel.
- Cache keys quantize pose, while cache identity retains profile, group, order
  and generation. A full cache rejects a new key; it does not evict silently.
- Collision remains outside IK. An IK result carries no collision-free claim.

### 4. Validation and Error Matrix

| Condition | Required result |
|---|---|
| generation is zero | `InvalidBudget { field: "ik.prepared_generation" }` |
| workspace/cache generation differs | `PreparedGenerationMismatch` |
| workspace/cache profile, group, order or shape differs | `PreparedWorkspaceMismatch` |
| initial order differs | `JointOrderMismatch` |
| coordinate is non-finite or outside a joint/mimic limit | `NonFinite` or `JointLimit` |
| solver DOF/joint kind/axis convention is incompatible | `UnsupportedGeometry` |
| caller omits the seeded prepared path for randomized solver | `UnsupportedCapability { capability: "explicit_seed_required" }` |
| work ends outside requested tolerances | `NonConvergent` with final residuals |
| workspace, population, cache or batch output exceeds its bound | `Capacity` |
| SQP augmented system loses required rank | `Singular` |

### 5. Good, Base and Bad Cases

- Good: prepare one profile/group/generation, allocate one workspace, execute
  repeated seeded solves, and invalidate both workspace and cache on reload.
- Base: use the compatibility wrapper for a one-shot deterministic DLS call.
- Bad: select OPW because a group has six joints, reuse a cache after a model
  generation change, let Bio-IK call thread-local randomness, or return the
  closest analytical candidate when it misses the requested tolerances.

### 6. Tests Required

- Generic fixtures cover multiple DOFs, mixed rotary/prismatic joints,
  reachable/unreachable targets, limits, order mismatch and deterministic
  repeats.
- Dedicated six- and seven-axis fixtures cover each admitted axis convention,
  wrist singularities and a near-miss geometry rejection.
- Solver evidence asserts the actual kernel name and solver-specific
  iterations/evaluations/generations/candidate counts.
- Cache tests cover quantized hits, full capacity and profile/group/order/
  generation rejection. Batch tests prove caller-owned capacity behavior.
- `allocation-counter` reports and asserts zero allocations for warmed calls.
- Run the ignored Release gate explicitly:

  ```text
  cargo test --release -p arm-algorithms --test ik_performance -- --ignored --nocapture
  ```

  Analytical/cache P95 targets are 50 microseconds; numerical IK P95 target is
  500 microseconds. These host results are not target-board qualification.
- Run `tools/ci.sh`, architecture check and dependency/source policy.

### 7. Wrong vs Correct

#### Wrong

```rust
// DOF is not analytical compatibility, and an explicit request cannot fall back.
if chain.dof() == 6 {
    solve_dls(target)
}
```

#### Correct

```rust
let prepared = PreparedKinematics::new(chain, model_generation)?;
let mut workspace = prepared.workspace_with_byte_limit(64, byte_ceiling)?;
let solution = prepared.solve(
    IkSolverKind::Opw,
    target,
    initial,
    config,
    &mut workspace,
)?;
assert_eq!(solution.solver(), IkSolverKind::Opw);
```

## Scenario: Add a prepared reactive RMP policy

### 1. Scope / Trigger

Use this contract when a reactive arm policy consumes FK/Jacobian evidence or
adds singularity handling under `arm-algorithms::reactive`. The policy remains
collision independent and is not a physical command path.

### 2. Signatures

```rust
let rmp = PreparedRmp::new(&kinematics, RmpConfig::host_default());
let mut workspace = rmp.workspace()?;
let output = rmp.accelerate(&positions, &velocities, &target, &mut workspace)?;

output.smallest_singular_value();
output.singularity_rotations();
workspace.allocated_bytes();
```

### 3. Contracts

- `PreparedRmp` borrows exactly one `PreparedKinematics`; its workspace binds
  profile, group, DOF and non-zero model generation.
- Preparation owns Jacobian and IK scratch. A warmed `accelerate` call has no
  heap allocation, container growth, clock read, logging, I/O or blocking.
- Reach, ordinary damping, finite joint-limit repulsion and singularity
  damping are individually finite and clamped before their deterministic sum
  is clamped to each joint's acceleration limit.
- The singularity metric is computed from the prepared Jacobian using the
  fixed-size Jacobi workspace. `singularity_rotations` is deterministic work
  evidence, not a collision or physical-safety claim.
- `RmpWorkspace::allocated_bytes()` reports preparation-owned dynamic and
  fixed scratch for test and host evidence only; it is not a runtime budget
  refresh mechanism.
- Obstacle/self-collision RMP remains `CollisionCapabilityUnavailable` until a
  separately validated collision provider exists.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| RMP workspace generation differs | `PreparedGenerationMismatch` |
| Profile, group or DOF differs | `PreparedWorkspaceMismatch` |
| Position/target/velocity order differs | `JointOrderMismatch` |
| Velocity coordinate kind differs | `UnitMismatch` |
| Active joint has no finite limits | `InvalidModel` |
| Gain, threshold or component becomes non-finite | `NonFinite` |
| Collision-dependent policy is requested | `CollisionCapabilityUnavailable` |

### 5. Good, Base and Bad Cases

- Good: prepare once, retain the workspace, repeatedly evaluate the same
  seeded/prepared arm state, and record zero warm allocations plus the bounded
  Jacobi rotation count.
- Base: apply only reach and ordinary damping with a valid `RmpConfig`.
- Bad: dynamically allocate an SVD in the servo call, combine unbounded
  components before validation, reuse a workspace after model reload, or
  interpret a small singular value as collision qualification.

### 6. Tests Required

- Assert each reported component and total is finite and within the compiled
  acceleration limits near a joint boundary and a singular configuration.
- Assert warmed `accelerate` reports zero allocations, stale generation is
  rejected, and rotation evidence is within the fixed Jacobi iteration bound.
- Run the isolated Release host report:

  ```text
  cargo test --release -p arm-algorithms --test reactive_task_performance -- --ignored --nocapture
  ```

  The output is host-only evidence, never RK3588, realtime or physical-arm
  qualification.

### 7. Wrong vs Correct

#### Wrong

```rust
// Dynamic matrix decomposition and formatted telemetry are not servo-safe.
let singularity = jacobian.svd(true, true).singular_values;
println!("singularity={singularity:?}");
```

#### Correct

```rust
let output = rmp.accelerate(&positions, &velocities, &target, &mut workspace)?;
assert!(output.singularity_rotations() <= MAX_JOINTS as u32 * MAX_JOINTS as u32 * 4);
```
