# Complete advanced arm algorithm implementations

## Goal

Complete the native, host-qualified mechanical-arm algorithm stack so that
the advanced IK, planning, reactive and task surfaces represent bounded,
testable algorithms rather than names routed through one fallback kernel or
typed placeholders. Preserve the existing multi-morphology profile/group/order
contracts and fail-closed collision boundary.

## Background

- The parent task requires the full KAA-10 software capability surface and
  explicitly defers collision detection, coordinated dual-arm, floating-base,
  parallel mechanisms, GPU, Python, visualization, Orange Pi and physical-arm
  qualification.
- `kinematics/ik.rs` currently implements DLS only. FABRIK, SQP, Bio-IK, OPW,
  Paden-Kahan and redundant seven-DOF capabilities report `implemented: false`
  and explicit requests return `UnsupportedCapability`.
- `planning/mod.rs` exposes twelve planner identities, but a valid direct edge
  returns the same interpolation for every identity; obstructed sampling
  planners share one bidirectional tree expansion, while GCS, IRIS and CHOMP
  reject a blocked direct seed. The planner label therefore does not yet prove
  the advertised algorithm family was executed.
- Joint-space RMP reach, damping and limit repulsion already exist. Collision
  RMP remains correctly unavailable without a collision provider.
- Task stages and cancellation state exist, but stage effects are not yet
  connected to bounded planning/trajectory proposal results.
- All outputs in this task remain host-only, single-group and
  `CollisionUnchecked`; none may acquire lease, deadline or hardware authority.
- A 250 ms P95 target for ordinary constrained/optimization fixtures was
  explicitly rejected as too slow. Performance acceptance must distinguish a
  fast nominal-case target from a bounded hard-case deadline; the latter must
  not be presented as normal algorithm latency.
- The accepted minimum floor for a warmed Release-mode nominal 6-DOF problem
  with at most 64 waypoints is P95 <= 10 ms and P99 <= 20 ms. Adversarial or
  infeasible problems have a strict 50 ms deadline and return typed
  exhaustion/non-convergence at the deadline.
- These millisecond limits are minimum acceptance floors, not the intended
  operating point. The selected implementations are expected to target
  microsecond-scale execution through precomputation, bounded workspaces,
  cache reuse and allocation-free inner kernels.
- The user selected the algorithms implemented by KINETIC commit
  `3dbaa61579c4359ce4c35f246fc83e1d3110b646` as the implementation reference.
  They must still be integrated as project-native code rather than fixing the
  project to KINETIC's workspace, dependencies or public contracts.
- KINETIC is the sole implementation source for every IK solver kernel,
  including DLS. No other IK library or independently substituted solver
  implementation may be mixed into the stack. Project-native work is limited
  to adaptation, validation, determinism, bounded storage, performance
  engineering and integration around those KINETIC kernels.

## Requirements

### AA-01 — truthful capability reporting

- Every advertised solver and planner must either execute its own documented
  algorithm family or return a typed, specific incompatibility/unavailability
  result before work begins.
- No implementation may report success by silently routing an advanced name
  through DLS, RRT-Connect or direct interpolation.
- Capability reports must include implementation status, supported joint
  kinds/DOF, geometry prerequisites and deterministic-work requirements.

### AA-02 — bounded multi-morphology IK

- Rebase the existing DLS behavior on the reviewed KINETIC DLS kernel so the
  complete IK family has one implementation source.
- Implement deterministic, bounded FABRIK, SQP and multi-objective Bio-IK for
  compatible variable-DOF serial groups.
- Implement OPW, Paden-Kahan/subproblem and redundant seven-DOF paths only for
  profiles that pass explicit geometry-compatibility analysis.
- Add reusable geometry analysis and cache/batch query surfaces bound to the
  immutable profile and group identity.
- Enforce exact dimensions, units, finite values, joint limits, iteration/work
  budgets and recorded seeds where randomized restarts are used.
- Return evidence for convergence, non-convergence, singularity, degraded
  solutions and incompatible geometry without producing executable commands.

### AA-03 — distinct bounded planners

- Implement a genuinely distinct bounded kernel for each public planner
  identity: RRT-Connect, RRT*, bidirectional RRT*, BiTRRT, EST, KPIECE and PRM.
- Implement distinct optimization kernels for CHOMP and STOMP, with explicit
  seed path, iteration and objective evidence.
- Implement Cartesian and constrained-RRT planning using caller-owned semantic
  constraints independent of collision checking.
- Provide GCS and IRIS-region behavior only against explicit caller-provided
  convex/feasibility contracts; never invent collision-free regions.
- Every randomized result must be reproducible from its seed and must report
  exact sample/node/iteration exhaustion.
- Shortcut, smoothing and pipeline composition must revalidate every generated
  state and preserve profile/group/order identity.

### AA-04 — reactive and task integration

- Complete RMP reach, damping, joint-limit and singularity-aware policies;
  obstacle/self-collision policies remain typed unavailable.
- Compile MoveJoint, MovePose, Cartesian, Pick and Place stages into bounded
  collision-unchecked plan/trajectory proposals using the selected compatible
  solver/planner.
- Preserve task cancellation semantics: accepting cancellation is nonterminal,
  cleanup is explicit, and no late planning result may advance the task.
- Gripper and Hold remain semantic intents and do not construct hardware
  commands.

### AA-05 — safety and architecture invariants

- Preserve project-owned SI types, fixed capacities, immutable profile/group/
  order identity and existing layer boundaries.
- Keep KINETIC as an attributed reference only: no Git/path dependency,
  submodule, vendored workspace or public KINETIC namespace.
- Treat KINETIC as the exclusive IK algorithm source. If a reviewed KINETIC IK
  path is incomplete or cannot satisfy project safety contracts, keep that
  exact capability typed unavailable rather than importing or inventing an
  alternative solver.
- Substantially adapted kernels must retain exact Apache-2.0 provenance and a
  modification record.
- All potentially expensive algorithms run as bounded host/non-realtime work;
  no realtime or allocation-free qualification is claimed.
- `CollisionUnchecked` results cannot enter physical admission, renew a lease
  or deadline, or construct raw protocol/hardware commands.

### AA-06 — performance contract

- Use caller-owned or reusable preallocated solver/planner workspaces. After
  workspace initialization, inner iterations must not grow containers or
  perform unbounded allocation.
- Enforce explicit memory, sample, node, iteration and objective-evaluation
  ceilings before starting work; capacity exhaustion returns typed evidence.
- Avoid repeated FK/Jacobian/model reconstruction by using profile/group-bound
  caches whose identity is checked before every reuse.
- Add release-mode benchmarks for every solver/planner family, separated into
  compatible 6-DOF, variable-DOF and constrained fixtures. Benchmarks report
  latency distribution, throughput, peak workspace bytes, allocation count
  and deterministic work counters.
- Define separate nominal and adversarial benchmark classes. Nominal latency
  is a performance gate; adversarial problems have a strict deadline and may
  return typed exhaustion/non-convergence rather than blocking or emitting an
  invalid path.
- Treat P95 <= 10 ms and P99 <= 20 ms as release-blocking floors, with a 50 ms
  adversarial deadline; the complete-call microsecond targets below are the
  intended operating point.
- Performance claims remain host-software claims until the separate RK3588 or
  physical-target qualification is executed.
- Split every eligible algorithm into a bounded preparation phase and a warm
  online query phase. Geometry analysis, factorization, region/roadmap build
  and workspace allocation are preparation work; online queries reuse only
  profile/group/order-bound prepared state.
- For warmed Release-mode nominal fixtures, target complete-call P95 latency
  (including input validation and result construction) as follows:
  - analytical or cache-hit IK: <= 50 us;
  - numerical IK: <= 500 us;
  - prepared GCS/PRM query: <= 200 us;
  - GCS build plus query, Cartesian planning, nominal CHOMP/STOMP and nominal
    projected constrained planning: <= 1,000 us;
  - each reusable core iteration: <= 10 us.
- The microsecond target does not manufacture feasibility. A sampling search
  that cannot solve within its configured work/deadline returns typed
  exhaustion; it does not silently switch algorithms or exceed the 50 ms
  adversarial deadline.

## Acceptance Criteria

- [ ] Capability tests prove every solver/planner is truthful and no advanced
      identity aliases DLS, direct interpolation or generic RRT-Connect.
- [ ] FABRIK, SQP and Bio-IK pass reachable, unreachable, singular,
      limit-boundary, wrong-dimension and NaN/Inf cases on multiple DOF and
      mixed-joint fixtures where compatible.
- [ ] OPW, Paden-Kahan and seven-DOF solvers pass dedicated compatible geometry
      goldens and reject near-miss geometries deterministically.
- [ ] Batch/cache queries reject profile/group generation mismatches and are
      bit-for-bit repeatable for identical inputs and seeds.
- [ ] Each planner has deterministic success and failure fixtures that exercise
      behavior unique to its family, plus work-budget exhaustion evidence.
- [ ] GCS/IRIS never claim collision-free regions without an explicit
      constraint/region provider.
- [ ] CHOMP/STOMP objectives improve or return typed non-convergence without
      emitting an invalid path.
- [ ] RMP outputs stay finite and within acceleration limits near singularities
      and joint boundaries; collision RMP remains unavailable.
- [ ] Task compilation produces bounded collision-unchecked proposals and
      cancellation/reload/context mismatch invalidates pending results.
- [ ] Compile/runtime tests prove collision-unchecked plans cannot reach
      physical admission or extend authority.
- [ ] Targeted Clippy, workspace tests, architecture, dependency/source policy,
      schema compatibility and the complete non-hardware host gate pass.
- [ ] Release benchmarks cover every independently implemented kernel and fail
      on workspace-capacity overflow, post-initialization inner-loop allocation
      or an agreed latency regression threshold.
- [ ] Warm nominal benchmarks meet the AA-06 microsecond targets on the recorded
      x86_64 reference host; the 10 ms P95 / 20 ms P99 values remain only
      release-blocking floors, never the reported target.

## Out of Scope

- Collision detection or collision-free qualification.
- Coordinated dual-arm, floating-base and closed-chain/parallel planning.
- Physical arm, CAN transmission, Orange Pi, camera or live ROS testing.
- GPU acceleration, Python bindings, viewer and online learning/training.
