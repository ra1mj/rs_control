# Project-native arm algorithms informed by KINETIC

## Goal

Build a project-native pure-software mechanical-arm algorithm boundary informed
by and legally traceable to
[`softmata/kinetic`](https://github.com/softmata/kinetic), while preserving this
repository's typed SI contracts, local lease/deadline enforcement and
application/protocol/hardware layer separation. KINETIC is an implementation
reference, not a runtime/workspace dependency or separately frozen subsystem.
Add a second, independently deployable reinforcement-learning policy module in
the control layer.

## Confirmed background

- The parent architecture reserves `application_module/arm-control` for Rust
  arm coordination, but no arm kinematics crate or arm trajectory Protobuf
  payload exists yet.
- Robot topology supports a named arm morphology, but all checked-in robot
  examples currently select `arm.morphology: none`; no production arm geometry
  or motor-role profile is available.
- Canonical `Pose3` and aggregate joint-state messages already exist. Generic
  action/lease contracts exist, but only a concrete chassis trajectory action
  is currently defined.
- KINETIC advertises FK, Jacobian, multiple IK solvers, collision checking,
  planning, trajectory generation, reactive servo, dynamics and execution. It
  is Apache-2.0.
- The reviewed upstream snapshot is commit
  `3dbaa61579c4359ce4c35f246fc83e1d3110b646` dated 2026-07-28. It has no tag
  or release. This identity is retained only for source/provenance auditing.
- That snapshot is not directly consumable as a Git dependency:
  `cargo metadata` fails because `kinetic-core` references an absent sibling
  path `../../../softmata-core`; optional KINETIC crates also reference absent
  sibling workspaces such as `../../../aegis`.
- This repository rejects unknown Git sources, so any external code must have
  a reviewed, immutable source/notice policy and pass `cargo deny`.
- KINETIC uses heap-backed `Vec` types and randomized IK restart options.
  Planning/IK therefore belongs on a bounded non-realtime worker; it must not
  be represented as a qualified allocation-free realtime loop.
- KINETIC's planning, reactive, task, grasp and GPU crates hard-depend on its
  collision/scene crates. Collision can be disabled as a qualified capability,
  but those upstream compile-time dependencies cannot simply be omitted while
  retaining every dependent module unchanged.
- This repository currently has no ML runtime, model-artifact manifest,
  reinforcement-learning observation/action schema or policy watchdog.
- RTen 0.24.0 is a released, end-to-end Rust CPU inference runtime that loads
  ONNX directly, supports ARM64 SIMD and permits a caller-owned fixed-size
  thread pool.

## User scope decisions

- Include the complete mechanical-arm algorithm capability set, rather than a
  kinematics-only MVP.
- Collision detection is deferred from the first qualified feature set.
- Add a control-layer module capable of deploying reinforcement-learning
  policies.
- Include all Rust arm algorithm/control capabilities listed in KAA-10.
- Exclude KINETIC Viewer, Python bindings, Horus bridge and GPU optimizer from
  this delivery; these are separate tooling/acceleration surfaces.
- Standardize the first reinforcement-learning deployment backend on ONNX
  models and exact-pinned RTen 0.24.0 CPU inference.
- Support learned direct rotary-actuator torque proposals and MIT
  impedance-control proposals in addition to position, position-residual and
  velocity action semantics.
- For learned MIT control, the policy outputs rotary-actuator position,
  velocity and feed-forward torque. It does not dynamically modify `kp/kd`;
  validated fixed gains from the immutable policy manifest complete the MIT
  proposal.
- Use a checked-in model-neutral synthetic 6-DOF URDF and golden vectors for
  the first host-qualified arm profile. A real arm remains a later
  configuration/model import and must not inherit synthetic limits.
- The integration must adapt multiple arm morphologies from validated model
  profiles; it must not hard-code six joints, one link geometry, one solver or
  one motor layout. The synthetic 6-DOF arm is only one conformance fixture.
- The first implementation covers open serial chains and multiple independently
  addressable base-to-tip planning groups. Closed-chain/parallel mechanisms,
  floating-base kinematics and coordinated dual-arm constraints are deferred,
  but the model and command identities must leave explicit extension points
  for dual arms, floating bases and coordinated groups.
- Implement the arm stack as native crates under this repository's existing
  layers. Do not add KINETIC as a Git/path dependency, submodule, vendored
  workspace or parallel public type system.
- Use a hybrid implementation strategy: redesign project contracts,
  multi-morphology models and control integration natively, while selectively
  porting mature KINETIC algorithm kernels when that reduces correctness risk.
  Ported kernels retain per-file Apache-2.0 provenance and are then maintained
  as project code.
- Continue as host/pure-software work without physical-arm, Orange Pi or live
  ROS qualification.

## Mechanical-arm requirements

- **KAA-01:** The primary integration is Rust and remains outside
  `hardware_module` and the qualified realtime control hot path.
- **KAA-02:** Mechanical-arm crates, APIs, errors and configuration are owned
  and versioned by this project. KINETIC is not a Cargo dependency, submodule,
  vendored workspace or public namespace.
- **KAA-02A:** Any implementation substantially copied or adapted from KINETIC
  retains Apache-2.0 notices, exact file/commit provenance and a local
  modification record. The provenance commit is an audit reference, not a
  version lock on this project's evolving implementation.
- **KAA-02B:** Porting is selective, not a directory snapshot. Each candidate
  kernel is reviewed for numerical behavior, allocation, randomness,
  dimensions and dependency fit; integration shells, public contracts,
  configuration and authority/admission code are written natively.
- **KAA-03:** The public project-facing API uses validated SI types and bounded
  joint counts. Angular and prismatic coordinates/efforts remain explicitly
  tagged (`rad/rad/s/N·m` versus `m/m/s/N`); parsed URDF, dynamic work buffers
  and library errors do not cross into `protocol_module` or hardware drivers.
- **KAA-03A:** A compiled arm profile has an immutable identity/hash and
  declares base/tip links, stable active-joint order, DOF, joint types,
  position/velocity/acceleration/effort limits and controller-resource
  mapping. Plans, observations, policy manifests and MIT proposals are bound
  to that profile identity and rejected on mismatch.
- **KAA-03B:** A robot model owns one or more named kinematic-group identities;
  no public contract assumes one global arm or one global base-to-tip chain.
  Current commands target exactly one supported group. Future composite
  dual-arm groups and floating-base frame state can be added without changing
  the meaning of existing single-group messages.
- **KAA-04:** Robot/model loading rejects malformed URDF, duplicate
  link/joint/group names, cycles or disconnected group paths, unknown base/tip
  links, unsupported joint types, invalid axes, missing/non-finite required
  limits and a topology inconsistent with the selected profile. Fixed joints
  require no motion limits; continuous joints require finite velocity/effort
  limits but no finite position interval.
- **KAA-05:** FK, Jacobian and IK reject wrong dimensions and non-finite input.
  IK has deterministic iteration/tolerance bounds, enforces joint limits and
  reports non-convergence, singularity/conditioning and degraded solutions
  without producing executable commands. Randomized search/restart algorithms
  require an explicit recorded seed and bounded work budget.
- **KAA-06:** Any joint trajectory output is finite, time-ordered, bounded in
  points/duration and checked against position, velocity and acceleration
  constraints before it becomes an action result. A dynamics-qualified result
  additionally requires valid inertial data and effort feasibility; missing
  inertia is `DynamicsUnavailable`, never a fabricated small mass.
- **KAA-07:** An arm goal remains a plan/command intent. Only the existing local
  lease, action lifecycle, command deadline and cancellation path may authorize
  execution.
- **KAA-08:** Host tests use a checked-in synthetic 6-DOF URDF with stable
  base-to-tool joint order and golden FK/IK vectors. The fixture is
  model-neutral, cannot be selected as a production hardware profile and makes
  no physical-arm accuracy or safety claim.
- **KAA-08A:** Multi-morphology conformance tests also cover different DOF,
  revolute/continuous/prismatic/fixed joints and more than one named
  base-to-tip planning group. Solver selection is capability-based: generic
  numeric solvers may serve compatible variable-DOF chains, while specialized
  6/7-DOF solvers fail closed when geometry prerequisites are not met.
- **KAA-09:** Camera, Orange Pi, live ROS, physical motor actuation and physical
  motion remain out of scope. Reuse of the existing Damiao MIT codec/driver in
  host-only semantic mapping tests does not qualify hardware execution.
- **KAA-10:** The complete algorithmic capability surface includes robot model
  loading/configuration; FK, batch FK, Jacobian, manipulability, workspace,
  cached/batch IK and compatible analytical/numerical IK; trapezoidal, TOTP,
  jerk-limited, spline, blend and trajectory validation; inverse/forward
  dynamics, gravity, mass matrix and effort feasibility; deterministic bounded
  RRT-Connect, RRT*, BiRRT*, BiTRRT, EST, KPIECE, PRM, GCS, CHOMP, STOMP,
  Cartesian, constrained-RRT and IRIS-region surfaces; servo, filters and RMP;
  task stages, pick/place and parallel-jaw/suction grasp generation; and
  simulated/logged execution. Dual-arm planning is the explicit KAA-13
  exception.
- **KAA-11:** Native planning/reactive/task/grasp modules must not reproduce
  KINETIC's hard compile-time coupling to collision/scene crates. While
  collision validation is unavailable, every affected result is explicitly
  `CollisionUnchecked`; it cannot enter the executable action path or be
  presented as safe.
- **KAA-12:** GPU optimization, visualization, Python bindings and Horus bridge
  are explicitly outside this delivery.
- **KAA-13:** Closed-chain/parallel, floating-base and coordinated dual-arm
  requests return typed `UnsupportedCapability` evidence in the first
  delivery. They must never be silently projected onto a fixed-base or
  single-arm problem. Independent planning groups within one robot tree remain
  supported.

## Reinforcement-learning control requirements

- **RLP-01:** Add an `application_module` policy-control boundary. Model
  loading, tensor allocation and inference run on a named non-realtime worker;
  no ML runtime appears in hardware/protocol crates or an allocation-free
  control hot path.
- **RLP-02:** A policy artifact is ONNX loaded by exact-pinned RTen 0.24.0 and
  has an immutable manifest containing format/runtime version, model SHA-256
  and byte limit, exact input/output
  names/dtypes/shapes, observation ordering, normalization, action semantics,
  action scaling/bounds, inference-period/deadline and fallback behavior.
- **RLP-02A:** The first artifact format is a bounded single-file ONNX model
  with embedded weights and static `f32` tensor shapes. External-data files,
  dynamic dimensions and unsafe memory-mapped loading are rejected.
- **RLP-03:** Startup validates the manifest and model graph before exposing
  readiness. Unknown/dynamic dimensions, unsupported operators/types, hash or
  shape mismatch, non-finite constants and output/robot-DOF mismatch fail
  closed.
- **RLP-04:** Every inference input is a coherent, freshness-checked semantic
  snapshot. Missing/stale/faulted required observations skip inference and
  invoke the configured safe fallback; missing values are never silently
  replaced with zero. The only v1 fallback is `WithdrawProposal`, allowing the
  existing local deadline/driver safe-stop path to act without manufacturing a
  fresh hold, zero or torque command.
- **RLP-05:** Outputs are finite, exact-shape and clamped/projected through
  project-owned position/velocity/acceleration/effort constraints. A model
  output is a proposal and never grants authority or bypasses a controller.
- **RLP-06:** Policy readiness is bound to a model generation/hash. Reload is a
  disarmed/startup operation; old outputs and recurrent state are invalidated
  across reload, restart, timeout or model failure.
- **RLP-07:** The worker uses bounded queues and an explicit inference time
  budget. Queue overflow, late inference, worker/channel or supervised process
  loss, or repeated invalid output becomes typed degraded/unavailable evidence
  and cannot renew a command deadline. Release-profile panic abort is not
  presented as recoverable inside the same process.
- **RLP-08:** A deterministic fixture policy proves model load, normalization,
  action decoding, bounds, recurrent-state reset where applicable, watchdog,
  fallback and model-generation invalidation on x86_64 host CI.
- **RLP-09:** Training, simulation environment construction and automatic
  online learning are outside this task; it deploys already-trained policy
  artifacts only.
- **RLP-10:** The manifest selects an explicit action semantic from an
  allowlist: absolute joint position, joint-position residual, joint velocity,
  direct rotary-actuator torque in N·m, or MIT impedance control. MIT proposals
  use the project-owned SI tuple `(position_rad, velocity_radps, kp, kd,
  feed_forward_torque_nm)` in the stable compiled joint-to-actuator binding
  order. For a learned MIT policy, the model tensor supplies position, velocity
  and feed-forward torque. Per-actuator `kp/kd` values are immutable manifest
  data tied to the model generation, validated against arm-control caps and the
  hardware protocol boundary, and cannot change during inference.
- **RLP-11:** Direct-torque and MIT outputs are semantic control proposals, not
  raw CAN frames. They cannot select a bus, motor ID, opcode or wire encoding;
  only the protocol/hardware boundary may map an admitted proposal to the
  existing Damiao `MitCommand`.
- **RLP-12:** Every torque/MIT field is finite and subject to manifest bounds,
  compiled joint-to-actuator binding limits and configured slew/rate limits.
  Application bounds must be no wider than the driver's verified runtime
  `PMAX/VMAX/TMAX`; MIT gains must remain within the protocol ranges
  `0 <= kp <= 500` and `0 <= kd <= 5`. The driver remains the final authority
  for verified limits, fresh feedback, enabled state and bus-set generation.
- **RLP-13:** Direct torque and MIT support receive deterministic host
  qualification only in this task. No physical-arm stability, force/torque
  accuracy or safe-contact claim is made.

## Acceptance criteria

- [ ] A clean checkout builds the native arm stack without any KINETIC
      Git/path/submodule/workspace dependency and passes license/source policy;
      every substantially derived source file has auditable provenance.
- [ ] A bounded project-owned arm model loads a reference URDF and exposes an
      exact, stable base-to-tool joint order.
- [ ] Multiple fixture morphologies produce distinct immutable profile
      identities; plans, RL tensors and MIT proposals cannot be replayed across
      incompatible joint counts, ordering, limits or controller mappings.
- [ ] A multi-group fixture proves that commands identify one exact group and
      cannot leak joints across groups; floating-base and coordinated dual-arm
      requests fail with typed unsupported evidence suitable for later
      extension.
- [ ] Golden FK and Jacobian tests pass for zero, nominal and joint-boundary
      configurations.
- [ ] Seeded deterministic IK recovers reachable poses within explicit
      position/orientation tolerances and rejects unreachable, singular,
      non-convergent and limit-violating cases.
- [ ] Every trajectory generator produces points obeying all dimensional,
      time, position, velocity, acceleration and applicable effort bounds.
- [ ] Planning, reactive, task/grasp and simulated/logged execution surfaces
      are present; any path without collision evidence is mechanically marked
      `CollisionUnchecked` and rejected by executable admission.
- [ ] Planning output cannot grant a lease, extend a local deadline or bypass
      action cancellation.
- [ ] A reviewed policy manifest and model fixture load through the selected
      Rust inference runtime with exact hash, tensor and robot-DOF validation.
- [ ] Policy tests cover valid inference, stale/missing observation, malformed
      model/manifest, wrong tensor shape/type, NaN/Inf output, action limiting,
      timeout, queue saturation, restart/reload and fallback.
- [ ] Golden tests cover direct-torque and MIT decoding in stable compiled
      joint-to-actuator binding order, SI units, gain/effort/rate limiting and
      rejection before any raw motor frame can be constructed.
- [ ] The learned policy never directly produces an executable motor command;
      lease, local deadline, lifecycle and project-owned constraints remain
      authoritative.
- [ ] Architecture, warnings-denied Clippy, workspace tests, dependency policy
      and the full host software gate pass.
