# Advanced arm algorithms — implementation plan

## 1. Parent planning and source admission

- [x] Record the user's requirement that every advertised algorithm use an
      independent kernel.
- [x] Record the microsecond online target and separate it from release floors
      and adversarial deadlines.
- [x] Audit KINETIC commit
      `3dbaa61579c4359ce4c35f246fc83e1d3110b646`, its benchmarks, dependencies,
      random sources, allocation behavior and selected file hashes.
- [x] Preserve KINETIC as an Apache-2.0 implementation reference only.
- [x] Split work into independently verifiable child deliverables.
- [x] Complete and review all parent/child planning artifacts.
- [x] Start only child 1 after explicit planning approval.

## 2. Child execution order

### Child 1 — advanced IK kernels

Task: `../08-03-advanced-ik-kernels`

Exit criteria:

- distinct FABRIK, SQP, Bio-IK, OPW, Paden-Kahan and redundant seven-DOF
  kernels exist;
- prepared geometry/workspace/cache contracts are stable for planners;
- explicit incompatible solvers fail without fallback;
- warm IK performance and allocation gates pass.

Rollback: restore exact typed unavailability for an unfinished solver. Never
alias it to DLS.

### Child 2 — sampling and roadmap planners

Task: `../08-03-sampling-roadmap-planners`

Hard dependency: child 1 prepared kinematic evaluation is complete.

Exit criteria:

- RRT-Connect, RRT*, BiRRT*, BiTRRT, EST, KPIECE and PRM have distinct search
  state and family-specific tests;
- deterministic seeds, fixed capacities and exact work evidence pass;
- prepared PRM queries meet the online performance target;
- every output remains collision unchecked.

Rollback: restore typed unavailability for only the failing planner identity.
Do not route it through RRT-Connect.

### Child 3 — optimization and constrained planners

Task: `../08-03-optimization-constraint-planners`

Hard dependency: child 1 prepared IK/evaluation is complete. It may execute in
parallel with child 2 only at the task level; inline implementation remains
sequential.

Exit criteria:

- CHOMP, STOMP, GCS, IRIS, Cartesian and constrained RRT have distinct kernels;
- preparation/query boundaries and caller-owned constraints are explicit;
- GCS/IRIS make no collision-free claim;
- warm nominal complete-call performance targets pass.

Rollback: remove prepared artifacts and restore typed unavailability for the
exact planner; do not weaken validation or collision evidence.

### Child 4 — reactive, task and performance integration

Task: `../08-03-arm-reactive-task-performance`

Hard dependency: children 1-3 are committed and archived.

Exit criteria:

- singularity-aware RMP and task-stage compilation use prepared algorithms;
- cancellation/context generation invalidates late results;
- portable allocation/work gates and isolated Release latency gates cover all
  families;
- cross-child identity, authority and collision typestate tests pass.

Rollback: remove integration/benchmark harness only. Do not roll back valid
algorithm kernels or relax task authority.

## 3. Cross-child implementation rules

- [x] Load `trellis-before-dev` before each child changes production files.
- [x] Use project-owned semantic types; do not import KINETIC crates/types.
- [x] Use the reviewed KINETIC files as the sole IK kernel source, including
      replacement/rebase of DLS; do not add another IK implementation source.
- [x] Put substantially adapted code behind per-file Apache-2.0 provenance.
- [x] Use explicit request seeds; no `thread_rng` or implicit entropy.
- [x] Preallocate validated workspaces; no post-preparation container growth.
- [x] Validate exact profile/group/order/generation at every prepared query.
- [x] Keep wall-clock enforcement outside deterministic kernels and discard
      late worker/orchestration results.
- [x] Keep every result `CollisionUnchecked`; do not change physical admission.
- [x] Do not add hardware, Orange Pi, CAN, camera, live ROS, GPU or Python work.

## 4. Parent integration review

- [x] Prove advanced names cannot alias DLS, direct interpolation or
      RRT-Connect using behavior-specific fixtures and statistics.
- [x] Prove profile/group/order/generation mismatch invalidates prepared IK,
      roadmap, region graph and optimizer state.
- [x] Prove deterministic repeatability for seeds, work counters, candidates
      and complete plans.
- [x] Prove workspace and output capacity exhaustion is typed and bounded.
- [x] Prove GCS/IRIS regions are constraint-feasible only and remain collision
      unchecked.
- [x] Prove task cancellation and context change reject late algorithm results.
- [x] Prove no plan/result grants or renews lease, deadline or hardware command
      authority.
- [x] Compare isolated Release benchmark results with KINETIC audit values and
      record differences without inheriting upstream claims.

## 5. Validation commands

Targeted child gates:

```bash
cargo fmt --all -- --check
cargo clippy -p arm-algorithms -p arm-control --all-targets --all-features -- -D warnings
cargo test -p arm-model -p arm-algorithms -p arm-control
cargo run --locked -p architecture-check -- check
```

Dependency and provenance gates:

```bash
cargo metadata --locked --no-deps --format-version 1
cargo deny check
rg -n "softmata|KINETIC|kinetic[_-]" crates/application_module/arm-algorithms
```

Isolated performance gates:

```bash
cargo test --release -p arm-algorithms --test ik_performance -- --ignored --nocapture
cargo test --release -p arm-algorithms --test planning_performance -- --ignored --nocapture
cargo test --release -p arm-algorithms --test planning_optimization_performance -- --ignored --nocapture
cargo test --release -p arm-algorithms --test reactive_task_performance -- --ignored --nocapture
```

Full non-hardware host gates:

```bash
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo test --locked --workspace --all-features -- --test-threads=1
```

Do not run Orange Pi, live CAN, camera, physical-arm or live ROS gates.

## 6. Finish

- [x] Run `trellis-check` for each child and the final parent matrix.
- [x] Update project specs with durable prepared-algorithm and performance
      contracts after implementation proves them.
- [x] Commit and archive each child independently in dependency order.
- [ ] Update the original `07-31-kinetic-arm-stack` checklist only from passing
      evidence.
- [ ] Archive this parent, then finish the original arm-stack and
      kinetic-arm-algorithms parents after their integration gates pass.
