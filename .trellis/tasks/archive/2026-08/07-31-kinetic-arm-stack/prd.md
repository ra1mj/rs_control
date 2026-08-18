# Project-native multi-morphology arm algorithm stack

## Goal

Build native Rust arm model, kinematics, trajectory, dynamics, planning,
reactive, task/grasp and simulated/log execution surfaces, informed by KINETIC
but owned by this project, with collision-unchecked fail-closed admission.

## Requirements

- **KAS-01:** Implement project-native crates, namespaces, errors and
  configuration without a KINETIC Git/path dependency, submodule, vendored
  workspace or parallel public API.
- **KAS-01A:** For source substantially copied or adapted from the reviewed
  Apache-2.0 KINETIC reference at commit
  `3dbaa61579c4359ce4c35f246fc83e1d3110b646`, retain per-file provenance,
  attribution and modification records. That commit is an audit reference,
  not a version lock on the project implementation.
- **KAS-01B:** Selectively port only reviewed algorithm kernels. Public
  contracts, multi-morphology models, configuration, error mapping,
  collision-evidence admission and control integration are project-native;
  whole upstream directories are not copied as a frozen subsystem.
- **KAS-02:** Provide project-owned bounded SI arm model/joint/pose/trajectory
  types with explicit angular versus prismatic units. KINETIC `Vec`, URDF,
  errors and namespaces do not appear in the production dependency graph or
  public API.
- **KAS-02A:** Adapt multiple validated arm morphologies without hard-coded
  DOF, link geometry, solver or motor layout. Each compiled profile has an
  immutable identity/hash over base/tip links, stable active-joint order, DOF,
  joint types, limits and controller-resource mapping.
- **KAS-02B:** A robot model exposes one or more named kinematic-group
  identities. Public contracts do not assume one global arm or chain, and
  current plans target exactly one group.
- **KAS-03:** Cover model/config loading, FK/IK/Jacobian/workspace analysis,
  trajectory, dynamics, planning, reactive servo/RMP, task/pick/place/grasp and
  simulated/log execution.
- **KAS-03A:** Cover the KAA-10 capability matrix, except collision and the
  explicitly deferred dual-arm/floating-base/parallel-mechanism algorithms.
  Randomized algorithms require an explicit seed and bounded work budget.
- **KAS-03B:** Dynamics requires valid link inertia and fails
  `DynamicsUnavailable` when absent; it never substitutes a nominal mass.
- **KAS-04:** All algorithms validate dimensions, finiteness, joint ordering,
  limits, deterministic bounds and typed convergence/degradation evidence.
- **KAS-05:** Collision is not qualified in this delivery. Every result from a
  collision-dependent surface is `CollisionUnchecked` and executable admission
  rejects it.
- **KAS-06:** Algorithms run outside the hardware layer and do not grant a
  lease, extend a deadline, complete cancellation early or emit a driver
  command.
- **KAS-07:** Exclude Viewer, Python bindings, Horus bridge and GPU optimizer.
- **KAS-08:** Use a checked-in model-neutral synthetic 6-DOF URDF with stable
  base-to-tool joint order and golden FK/IK vectors. It cannot be selected as
  a production hardware profile and makes no physical-arm claim.
- **KAS-09:** Add conformance fixtures spanning different DOF,
  revolute/continuous/prismatic/fixed joints and multiple named base-to-tip
  planning groups. Generic and specialized solvers advertise compatibility
  and reject unsupported geometry instead of silently falling back.
- **KAS-10:** Keep explicit model/command extension points for dual-arm
  composite groups and floating-base frame state. Closed-chain/parallel,
  floating-base and coordinated dual-arm requests are not implemented in this
  delivery and return typed `UnsupportedCapability` instead of being reduced
  to a fixed-base/single-arm request.

## Dependencies

- Depends on existing `message-model` SI/authority types and `control-core`
  lease/action lifecycle contracts.
- Produces the bounded joint/model types consumed by
  `07-31-rl-policy-control`; the RL child must not depend on upstream KINETIC
  types.

## Acceptance Criteria

- [x] A clean checkout builds without any KINETIC
      Git/path/submodule/workspace dependency, while substantially derived
      files retain auditable Apache-2.0 provenance.
- [x] Golden FK/Jacobian/IK/trajectory/dynamics tests cover nominal, boundary,
      unreachable, singular and invalid inputs.
- [x] The synthetic 6-DOF fixture has deterministic joint order and cannot be
      mistaken for a real-arm hardware profile.
- [x] Profile identity prevents plans and control proposals from crossing
      incompatible morphology, joint-order, limit or controller mappings.
- [x] A multi-group fixture isolates group joint sets, while floating-base and
      coordinated dual-arm requests produce typed unsupported evidence.
- [x] Planning/reactive/task/grasp/sim/log surfaces are testable and cannot
      elevate `CollisionUnchecked` output into an executable action.
- [x] The public API is bounded, finite, SI-valued and stable for the RL child.
- [x] Architecture, Clippy, tests, dependency policy and full host CI pass.
