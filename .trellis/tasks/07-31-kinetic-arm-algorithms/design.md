# Project-native arm algorithms and policy control — technical design

## 1. Design objective

Add a project-owned multi-morphology arm stack and an independently deployable
reinforcement-learning policy runtime without creating a KINETIC runtime
dependency or weakening the existing authority, timing, architecture and
hardware-driver boundaries.

The two executable invariants are:

```text
algorithm or learned policy
  -> bounded project-owned proposal
  -> profile/group/generation/freshness/constraint admission
  -> existing lease + deadline + lifecycle admission
  -> protocol-owned semantic-to-device mapping
  -> existing hardware driver
```

```text
application_module -> protocol_module -> hardware_module
```

The current task is host-only. No path created by this task is physical-motion
qualified.

## 2. Fixed decisions

| Area | Decision |
|---|---|
| KINETIC relationship | Implementation reference and selective Apache-2.0 kernel provenance only; no Git/path dependency, submodule, vendored workspace or public KINETIC API |
| Native crate shape | `arm-model`, `arm-algorithms`, `arm-control`, and `rl-policy-control` under `crates/application_module/` |
| Morphology | Runtime-validated open serial chains and multiple named base-to-tip groups; no hard-coded six-joint assumption |
| First fixtures | Model-neutral 6-DOF golden arm plus variable-DOF, mixed revolute/prismatic/fixed and multi-group fixtures |
| Deferred topology | Closed-chain/parallel mechanisms, floating base and coordinated dual-arm constraints return typed unsupported evidence; identifiers preserve later extension |
| Collision | Deferred; affected products carry compile-time `CollisionUnchecked` evidence and cannot enter executable admission |
| Algorithm scope | Full KAA-10 model, kinematics, trajectory, dynamics, planning, reactive, task/grasp and sim/log surface, except the explicitly deferred dual-arm/collision functions |
| Control modes | Position, position residual, velocity, direct rotary torque and fixed-gain MIT impedance proposals |
| Learned MIT | ONNX emits actuator-space `q`, `dq` and feed-forward torque in stable compiled joint-to-actuator binding order; immutable per-actuator `kp/kd` comes from the policy manifest |
| ML backend | Exact-pinned RTen 0.24.0, CPU, caller-owned fixed-size pool, named non-RT worker |
| Failure fallback | Withdraw the proposal; never synthesize a fresh hold, zero, velocity or torque command |
| Qualification | Deterministic x86_64 host software evidence only; no Orange Pi, live ROS, camera or physical-arm claim |

## 3. Task and dependency map

The parent owns cross-child contracts and final integration review. The
children remain independently verifiable.

| Order | Child | Owns | Explicit dependency |
|---|---|---|---|
| 1 | `07-31-kinetic-arm-stack` | Semantic arm contracts, native model/algorithms/control, MIT mapping, provenance and fixtures | Existing `message-model`, `control-core`, action/lease contracts and Damiao codec |
| 2 | `07-31-rl-policy-control` | Artifact manifest, ONNX/RTen worker, observation/action decoding, direct-torque/MIT policy proposals | Stable profile, group, joint-vector and proposal contracts completed by child 1 |
| 3 | Parent integration review | Cross-child generation/profile invalidation, architecture, dependency and complete host gates | Both children complete |

The RL child must not start by inventing substitute arm types. It waits for the
first child's contracts and imports only project-owned types.

## 4. Workspace and layer ownership

### 4.1 Protocol-owned semantic contract

`crates/protocol_module/message-model` owns values that must cross an
application/device boundary:

- nonzero 32-byte `ArmProfileId`;
- named `KinematicGroupId`;
- bounded stable joint ordering and tagged angular/prismatic state;
- profile/group-bound joint position, velocity, effort and trajectory values;
- direct torque and full MIT semantic proposals;
- fixed-capacity arm trajectory action goal/feedback/result values.

Angular and linear coordinates are distinct. A prismatic coordinate is never
stored in a field named `_rad`, and a force is never relabeled `_nm`.

`proto/robot/v1/control.proto` and `contract-codec` add the concrete bounded arm
trajectory action wire contract. Generated DTOs stop at `contract-codec`. No
external MIT/CAN payload is added: learned/manual low-level proposals remain a
local semantic control interface.

`contract-codec` is also the legal mapping owner between a validated semantic
rotary MIT setpoint and `motor_damiao::MitCommand`, because it may depend on
both protocol and hardware crates. The mapper:

1. matches the exact resource/profile/group context;
2. converts checked `f64` SI values to `f32`;
3. refuses precision/range loss outside policy;
4. validates against the motor's verified runtime `PMAX/VMAX/TMAX`;
5. delegates protocol gain/range validation to the Damiao codec.

It does not select a CAN bus, motor ID or construct a CAN frame.

### 4.2 Application-owned native arm stack

```text
arm-model
  validated URDF/profile/group/transmission model
  immutable profile hash and stable joint order

arm-algorithms
  kinematics/
  trajectory/
  dynamics/
  planning/
  reactive/
  grasp/
  task/

arm-control
  proposal projection and slew limits
  profile/group/generation/freshness checks
  collision-evidence typestate
  lease/deadline/action admission integration
  deterministic simulation and caller-owned log records

rl-policy-control
  manifest/model validation
  non-RT RTen worker
  observation encoder, action decoder and watchdog evidence
```

`arm-algorithms` may allocate only in explicitly non-RT planning/query calls.
It never owns a clock, lease, lifecycle transition, middleware backend or
driver. `arm-control` exposes bounded step/admission types suitable for a
future qualified loop, but this task does not claim a realtime rate.

### 4.3 Deployment configuration

The existing robot YAML remains minimal:

```yaml
arm:
  morphology: <profile-name>
  motor_ids: <logical-role-to-id>
```

URDF, inertia, solver tuning, controller gains, policy artifacts and tensor
contracts do not enter `config/robots/*.yaml`. A future composition root loads
the selected profile artifact, validates the YAML role map against it and
passes narrow immutable values to each layer.

Synthetic fixtures live under test fixture directories, not
`config/robots/`, and cannot be selected as production profiles.

## 5. Identity and multi-morphology model

An `ArmProfileId` is the SHA-256 of a canonical, versioned representation of:

- profile schema version and name;
- normalized URDF model identity;
- named base/tip groups and stable active-joint order;
- joint type, axis, origin and required limits;
- link inertial values where present;
- controller-resource and actuator/transmission mapping;
- control limit profile.

Ordering uses explicit profile group order, not hash-map iteration or incidental
URDF traversal. Every plan, trajectory, observation, learned output and MIT
proposal contains the profile and group identity. Mismatches fail before
numeric decoding or projection.

The first model supports revolute, continuous, prismatic and fixed joints in an
acyclic tree. Fixed joints have no motion coordinate. Continuous joints require
finite velocity/effort limits but have no finite position interval. A group is
one validated base-to-tip open chain. Multiple groups may share fixed/root
links, but current commands target one exact group and cannot leak active
joints across groups.

The public shape leaves room for a future composite group and explicit base
frame state. Current closed-chain, parallel, floating-base and coordinated
dual-arm requests return `UnsupportedCapability`; they are not projected onto
a supported chain.

## 6. Algorithm and evidence boundaries

All algorithms share:

- exact profile/group and dimension validation;
- finite SI input/output checks;
- explicit iteration, sample, waypoint, duration and memory budgets;
- deterministic seeded random search where sampling is required;
- typed convergence, conditioning, saturation and unavailable evidence;
- no panics for model, policy or user input.

Specialized analytical IK advertises a compatibility predicate. Automatic
selection uses a specialized solver only after that predicate passes, otherwise
it chooses a generic numerical solver. An explicitly requested incompatible
solver returns `UnsupportedGeometry`; it never silently changes algorithms.

Planning algorithms consume a project-owned state-validity interface. The first
delivery provides joint-limit/constraint validity only. With no collision
provider, planning, reactive avoidance, grasp filtering and task products are
typed `CollisionUnchecked`.

Collision evidence uses typestate:

```text
ArmProduct<CollisionUnchecked> -> simulation/log inspection only
ArmProduct<CollisionChecked>   -> future executable-admission candidate
```

No current production constructor creates `CollisionChecked`. Tests may use a
private fixture proof solely to verify that admission is structurally
impossible for unchecked products.

Dynamics requires valid inertia. Missing, non-finite or non-physical mass and
inertia yields `DynamicsUnavailable`; nominal substitute mass is forbidden.

## 7. Authority and control flow

An algorithm or policy returns a proposal, never a command envelope.
`arm-control` performs:

1. exact profile, group, model/policy generation and joint-order checks;
2. coherent observation freshness and quality checks;
3. position/velocity/acceleration/effort and slew projection;
4. action-semantic decoding;
5. collision-evidence admission;
6. existing `control-core` lease authorization and action/cancellation checks;
7. exclusive local command-deadline construction.

Direct rotary torque becomes an MIT semantic tuple only after fresh feedback is
available, with zero impedance gains and bounded feed-forward torque. Learned
MIT fills position, velocity and feed-forward torque from the model output and
uses immutable manifest gains. Neither mode reaches a Damiao driver unless the
separate runtime limit/configuration and existing driver lifecycle gates pass.

Because collision is deferred, the delivered sim/log executor accepts
`CollisionUnchecked`; the physical executor admission accepts only
`CollisionChecked` and therefore remains closed.

## 8. RL artifact and worker flow

The v1 artifact is:

```text
manifest.toml
model.onnx
```

The manifest fixes model SHA-256/size, RTen version, exact static `f32`
input/output tensors, profile/group/order, observation sources,
normalization, action kind/scaling/bounds/slew, immutable MIT gains, inference
period/deadline, queue capacity, RTen thread count, optional recurrent state
and `WithdrawProposal` fallback.

Startup reads bounded bytes, verifies the hash, parses the single-file ONNX
graph, rejects external data/dynamic shapes/non-finite constants, loads it with
safe `Model::load`/`load_file` rather than unsafe mmap, and cross-checks RTen
node metadata before exposing readiness.

A named non-RT worker owns `rten::Model`, tensor allocation and a caller-sized
`rten::ThreadPool`. Request/result queues are bounded SPSC structures.
Each request/result carries request sequence, profile ID, group ID, model
generation and local enqueue/completion timestamps. Full queues, worker loss,
late output, wrong generation and invalid output withdraw the proposal and do
not refresh any command validity.

## 9. Compatibility, rollout and rollback

- Protobuf additions are additive in `robot.v1`; zero enum values are
  unspecified and rejected. Existing field numbers are not reinterpreted.
- Existing `JointState` remains compatible. New tagged arm state is added
  rather than relabeling its angular fields for prismatic joints.
- Existing robot YAML examples remain `arm.morphology: none` and continue to
  validate byte-for-byte.
- Existing Damiao wire layout and driver lifecycle remain authoritative.
- New crates are classified atomically in `Cargo.toml` and
  `architecture.toml`.
- If a selectively ported kernel fails review, remove that kernel and keep its
  capability unavailable; never weaken validation or import the KINETIC
  workspace to make progress.
- If RTen dependency, operator or license policy fails, the RL crate remains
  unready and no alternative runtime is silently selected.
- Rolling back either child removes only its new crates/contracts and restores
  the previous additive schema baseline; no hardware or production profile is
  mutated.

## 10. Qualification boundary

Host tests prove deterministic software contracts, numerical golden vectors,
failure behavior and architecture. They do not prove:

- real-arm geometry, calibration or transmission accuracy;
- contact/collision safety;
- torque accuracy or MIT stability;
- realtime scheduling on RK3588;
- physical safe stop, E-stop or motor timeout behavior;
- live ROS interoperability.

Those gates remain explicit future tasks.
