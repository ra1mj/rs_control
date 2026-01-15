# Project-native multi-morphology arm stack — implementation plan

## 1. Dependency and provenance admission

- [x] Record the reviewed KINETIC commit and per-module source inventory.
- [x] Add the project provenance/notice file before porting the first kernel.
- [x] Review exact registry versions for `nalgebra`, XML parsing,
      deterministic RNG and the proposed dynamics backend. `urdf-rs 0.9.0`
      failed the advisory gate; use exact `quick-xml 0.41.0` through a bounded
      project parser instead.
- [x] Run license/advisory/source checks before writing wrappers around a new
      dependency.
- [x] Stop and return to planning if `featherstone = 0.1.0` fails source,
      license, precision or API review; do not add a deny exception
      automatically.

Rollback: no production crate should yet depend on rejected libraries.

## 2. Protocol semantic foundation

- [x] Add `ArmProfileId`, `KinematicGroupId`, joint-kind and tagged
      angular/prismatic values to `message-model`.
- [x] Add bounded profile/group-bound joint observations and proposal types.
- [x] Add absolute position, position residual, velocity, direct rotary torque
      and MIT proposal variants.
- [x] Add validated bounded arm trajectory action goal/feedback/result types.
- [ ] Add good/base/bad tests for every constructor, all capacities,
      duplicate/order mismatch and NaN/Inf.

Validation:

```bash
cargo test -p message-model
```

## 3. Arm model and profile compiler

- [x] Scaffold and classify `arm-model`.
- [x] Implement bounded URDF parse and project-owned link/joint/tree types.
- [x] Validate names, topology, axes, origins, limits and group base/tip paths.
- [x] Implement fixed/continuous/revolute/prismatic behavior.
- [x] Validate mimic relationships without adding mimic joints as independent
      DOF.
- [x] Validate optional dynamics inertia without fabricating missing data.
- [x] Implement one-to-one rotary actuator/transmission bindings.
- [x] Implement canonical serialization and `ArmProfileId` SHA-256.
- [x] Make group order explicit and deterministic.
- [x] Add synthetic 6-DOF, variable-DOF, mixed-joint and multi-group fixtures.
- [x] Prove fixture profiles cannot enter the production profile registry.
- [x] Test semantic identity stability and mutation sensitivity.

Validation:

```bash
cargo test -p arm-model
```

Rollback: remove the new crate and protocol types together if profile identity
cannot be made deterministic; do not use source XML bytes as the semantic hash.

## 4. Kinematics

- [x] Scaffold `arm-algorithms` and its kinematics modules.
- [x] Implement chain extraction, FK/all-link/batch FK and 6×N Jacobian.
- [x] Implement condition/manipulability with typed singular evidence.
- [ ] Port/rewrite DLS and FABRIK with strict dimension/limit checks.
- [ ] Port/rewrite SQP and Bio-IK with explicit bounded work.
- [ ] Port/rewrite OPW, Paden-Kahan/subproblem and compatible 7-DOF paths.
- [ ] Implement geometry compatibility analysis, cache and batch queries.
- [x] Implement deterministic workspace/reachability sampling.
- [x] Add solver capability reporting and deterministic `Auto` selection.
- [x] Add per-file provenance only where code is substantially derived.

Tests:

- [x] Golden FK/Jacobian at zero, nominal and limits.
- [ ] Reachable IK, unreachable target, wrong DOF, NaN/Inf, singular,
      non-convergent and limit-violating cases.
- [x] Specialized solver rejection on incompatible geometry.
- [x] Repeat identical seeds and compare complete results/evidence.

## 5. Trajectory and dynamics

- [x] Implement bounded trajectory/path constructors and caller-owned sampler.
- [x] Implement trapezoidal, TOTP, jerk-limited, cubic/clamped spline and
      blending surfaces.
- [x] Implement validation for time, position, velocity, acceleration, jerk
      where applicable and exact group dimensions.
- [x] Admit the reviewed dynamics backend and implement the strict project
      adapter.
- [x] Implement inverse/forward dynamics, gravity, mass matrix, dynamic
      manipulability and effort feasibility.
- [x] Reject missing inertia, untimed feasibility input and singular/invalid
      dynamics; never return empty success for unavailable analysis.

Tests:

- [x] Analytic one/two-link dynamics golden vectors and matrix properties.
- [x] Nominal and boundary trajectory golden vectors.
- [ ] Every violation kind with exact waypoint/joint evidence.

## 6. Planning

- [x] Define planner/state-validator traits and
      `ArmPlan<CollisionEvidence>`.
- [x] Implement deterministic configs with explicit seed/work budgets.
- [ ] Port/rewrite RRT-Connect, RRT*, BiRRT* and BiTRRT.
- [ ] Port/rewrite EST, KPIECE and PRM.
- [ ] Port/rewrite GCS and IRIS-region surfaces.
- [ ] Port/rewrite CHOMP and STOMP.
- [ ] Implement Cartesian and constrained-RRT planning.
- [ ] Implement shortcut, smoothing and pipeline composition.
- [x] Make every first-delivery result `CollisionUnchecked`.
- [x] Return typed unsupported evidence for coordinated dual-arm planning.

Tests:

- [ ] Deterministic success/failure for every planner on small fixtures.
- [ ] Sample/node/iteration/time-budget exhaustion.
- [ ] Constraint violation, invalid seed and incompatible group rejection.
- [ ] Compile/runtime tests proving unchecked plans cannot enter physical
      admission.

## 7. Reactive, grasp and task surfaces

- [x] Implement twist, joint-jog and pose-tracking servo.
- [x] Implement no-filter, EMA and Butterworth filters.
- [ ] Implement RMP reach, joint-limit, singularity and damping policies.
- [x] Return collision capability unavailable for obstacle/self-collision
      policies.
- [x] Implement bounded primitive object/gripper descriptions.
- [x] Implement deterministic antipodal, top-down, side and suction candidates
      plus ranking/reachability.
- [x] Mark candidates collision unchecked.
- [x] Implement bounded move/Cartesian/gripper/pick/place/sequence task stages.
- [ ] Integrate task effects with existing action/cancellation semantics.

## 8. Arm control and simulated/logged execution

- [ ] Scaffold and classify `arm-control`.
- [ ] Implement profile/group/order/freshness/generation gates.
- [ ] Implement joint/actuator limit projection and per-field slew checks.
- [ ] Implement power-consistent transmission projection.
- [ ] Implement direct torque using fresh position and zero MIT gains.
- [ ] Implement fixed-gain MIT semantic proposal construction.
- [ ] Implement collision evidence typestate and closed physical admission.
- [ ] Implement deterministic sim execution and caller-owned bounded log
      records without stdout/stderr or I/O.
- [ ] Integrate existing lease/deadline/action/cancellation checks.

## 9. Protobuf and Damiao boundary

- [ ] Add additive `robot.v1` arm profile/group/trajectory action messages.
- [ ] Add semantic-to-wire and wire-to-semantic mappings only in
      `contract-codec`.
- [ ] Add strict capacity, presence, enum, identity, unit/order and NaN/Inf
      tests.
- [ ] Add validated semantic MIT-to-`MitCommand` mapping in `contract-codec`.
- [ ] Expose only the minimum Damiao validation API needed by that mapper;
      preserve the existing eight-byte frame layout and golden vectors.
- [ ] Do not add an external raw MIT/CAN Protobuf message.
- [ ] Update generated descriptor identities/goldens only after breaking and
      cross-language gates pass.

Validation:

```bash
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo test -p proto-generated -p message-model -p contract-codec -p motor-damiao
```

## 10. Architecture and full verification

- [ ] Add all workspace dependencies through `[workspace.dependencies]`.
- [ ] Classify all new crates in `architecture.toml`.
- [ ] Add positive real-graph checks and retain reverse-edge negative tests.
- [ ] Scan the production graph for `kinetic`, KINETIC namespaces and
      unreviewed Git/path sources.
- [ ] Run targeted all-feature Clippy with warnings denied.
- [ ] Run architecture, dependency and full host CI.

```bash
cargo metadata --no-deps --format-version 1
cargo clippy -p arm-model -p arm-algorithms -p arm-control --all-targets --all-features -- -D warnings
cargo test -p arm-model -p arm-algorithms -p arm-control
cargo run --locked -p architecture-check -- check
cargo deny check
./tools/ci.sh
```

## 11. Child completion gate

- [ ] Public arm contracts are documented and stable for the RL child.
- [ ] Every KAS requirement maps to at least one test.
- [ ] No collision-unchecked value reaches executable admission.
- [ ] No host output is described as physical or realtime qualification.
- [ ] Commit and archive this child before starting
      `07-31-rl-policy-control`.
