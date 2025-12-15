# Project-native multi-morphology arm stack — technical design

## 1. Scope and hard dependency boundary

This child delivers the project-owned arm contract and algorithm stack required
by `07-31-rl-policy-control`.

It depends on:

- `message-model` for semantic SI identities, observations, actions and
  proposals;
- `control-core` for lease/action/lifecycle semantics;
- the existing Damiao codec only through the legal `contract-codec` protocol
  boundary;
- the parent PRD/design for collision, topology and qualification limits.

The RL child must wait for this child's public contracts. This child does not
depend on the RL child.

## 2. Crate layout

```text
crates/application_module/
  arm-model/
    model.rs          # project-owned links, joints and group registry
    profile.rs        # profile compilation and SHA-256 identity
    urdf.rs           # bounded parse/validation boundary
    transmission.rs   # joint-to-rotary-actuator mapping

  arm-algorithms/
    kinematics/
    trajectory/
    dynamics/
    planning/
    reactive/
    grasp/
    task/

  arm-control/
    proposal.rs       # position/velocity/effort/MIT projection
    admission.rs      # identity/freshness/evidence/authority gates
    execution.rs      # sim and caller-owned log records
```

All three crates are classified as `application_module`. `arm-model` depends on
`message-model`; `arm-algorithms` depends on `arm-model`; `arm-control` depends
on the other two plus `control-core` and `message-model`. None imports a
hardware crate.

Protocol additions live in existing owners:

```text
crates/protocol_module/message-model/
crates/protocol_module/contract-codec/
proto/robot/v1/control.proto
crates/hardware_module/motor-damiao/   # validation API only; wire layout unchanged
```

## 3. Storage and boundedness

`MAX_JOINTS = 64` remains the repository-wide maximum. One step/proposal uses
fixed-capacity joint storage suitable for nonblocking local delivery.

Large models, planner trees and complete trajectories are non-RT values. They
may use heap storage only behind constructors that enforce:

- maximum links, joints, groups, waypoints and bytes;
- exact group DOF on every vector;
- finite values and stable order;
- checked allocation size before allocation.

A full arm trajectory is not nested as
`ArrayVec<ArrayVec<f64, 64>, 256>` because that would create an unnecessarily
large stack value. It owns a bounded heap allocation and samples into a
caller-owned fixed-capacity output.

Dynamic `nalgebra` matrices remain internal to non-RT algorithms. Public
Jacobian and mass-matrix results expose validated dimensions and bounded
storage, never an unvalidated upstream matrix.

## 4. Semantic identities and units

`message-model` adds:

- `ArmProfileId([u8; 32])`, rejecting all-zero identity;
- `KinematicGroupId`, using the existing portable ASCII identifier;
- `ArmJointKind::{Revolute, Continuous, Prismatic}`;
- profile/group-bound bounded joint state and coordinate vectors;
- tagged angular (`rad`, `rad/s`, `N·m`) and linear (`m`, `m/s`, `N`) values;
- `ArmControlProposal` variants for absolute position, position residual,
  velocity, direct rotary torque and MIT impedance;
- `RotaryMitSetpoint` and bounded aggregate;
- bounded concrete arm trajectory action types.

The stable group descriptor owns the scalar interpretation for trajectory
vectors. Wire messages include profile ID, group ID, exact joint order and
joint kinds; a point cannot be decoded without that context.

MIT is actuator-space and always uses radians, radians/second and N·m. A
prismatic kinematic joint is MIT-eligible only when its profile supplies a
validated one-to-one rotary transmission.

## 5. URDF and profile compilation

The primary parse boundary uses exact-pinned `quick-xml 0.41.0` through a
project-owned bounded event parser. `urdf-rs 0.9.0` was rejected because its
mandatory `quick-xml 0.36.2` dependency fails the advisory gate
(`RUSTSEC-2026-0194` and `RUSTSEC-2026-0195`). The dynamic parse tree remains
private. Compilation produces an immutable `ArmModel`.

Validation includes:

- input byte bound and well-formed XML;
- unique portable robot/link/joint/group names;
- exactly one root and an acyclic connected tree for every selected group;
- valid parent/child references and non-self edges;
- finite normalized joint axes;
- finite origin translation and normalized orientation;
- supported revolute, continuous, prismatic and fixed joints;
- rejection of planar/floating joints in v1;
- finite ordered position limits for revolute/prismatic joints;
- finite positive velocity, acceleration and effort limits for active joints;
- no motion limits required for fixed joints;
- continuous joints have velocity/acceleration/effort limits but no finite
  position interval;
- finite positive mass and physically plausible symmetric inertia when
  dynamics is requested;
- mimic source existence, finite multiplier/offset and no mimic cycle; mimic
  joints do not add an independent DOF;
- group base/tip existence and one deterministic open path;
- exact one-to-one logical controller role/resource binding;
- finite nonzero transmission ratio and direction.

The compiler constructs stable explicit group order. It does not use
`HashMap` iteration or auto-select the "first" group.

The profile identity hashes one canonical versioned byte representation of the
model/group/controller data. Original XML whitespace, file location and map
iteration order do not affect identity; a semantic axis, limit, ordering,
inertia or mapping change does.

## 6. Transmission and actuator projection

Each controllable joint may declare one rotary actuator binding:

```text
motor_position_rad = scale_rad_per_joint_unit * joint_position + offset_rad
motor_velocity_radps = scale_rad_per_joint_unit * joint_velocity
motor_torque_nm = joint_effort / scale_rad_per_joint_unit
```

`joint_unit` is radians for rotary joints and metres for prismatic joints.
The effort conversion is power-consistent. Scale is finite, nonzero and its
sign records direction.

Algorithm trajectories stay in joint space. MIT proposals are projected to
actuator space only by `arm-control` after profile/group/order checks. Profiles
without a complete one-to-one rotary binding reject MIT as
`UnsupportedControlMode`.

## 7. Algorithm capability design

### 7.1 Kinematics

Native surfaces include:

- chain extraction, FK/all-link FK and batch FK;
- 6×N geometric Jacobian, condition number and manipulability;
- DLS, FABRIK, SQP and Bio-IK numerical solvers;
- OPW, Paden-Kahan/subproblem 6-DOF and redundant 7-DOF analytical paths;
- geometry analysis/IKFast-compatible selection, cached IK and batch IK;
- deterministic reachability/workspace maps.

Every solver publishes `SolverCapabilities`. Automatic selection is
deterministic. Random restarts and workspace sampling use an explicit portable
seed and record consumed samples/iterations. An incompatible requested solver
returns `UnsupportedGeometry`; `Auto` may choose a compatible solver but
records which one was used.

### 7.2 Trajectory

Native surfaces include:

- trapezoidal and per-joint trapezoidal timing;
- time-optimal path parameterization;
- jerk-limited S-curve generation;
- cubic/clamped spline timing;
- trajectory blending and sequence blending;
- deviation monitoring and complete validation.

Every trajectory has exact profile/group/order identity, nonzero strictly
increasing time from start, bounded duration/points and finite
position/velocity/acceleration. Samplers write into caller-owned storage.

### 7.3 Dynamics

Surfaces include RNEA inverse dynamics, ABA forward dynamics, gravity
compensation, CRBA mass matrix, dynamic manipulability and trajectory effort
feasibility.

The primary implementation may use exact-pinned `featherstone = 0.1.0` only
after source, license, advisory, precision and API review. The adapter validates
all dimensions and converts explicitly between project `f64` values and the
backend. Missing inertia, an un-timed trajectory, a singular mass matrix or a
dimension mismatch is an error, never an empty-success result or nominal mass.

If the dependency admission gate fails, stop and return to planning for a
project-native f64 dynamics kernel; do not silently broaden `deny.toml`.

### 7.4 Planning

One project-owned planner interface covers:

- RRT-Connect, RRT*, bidirectional RRT*, BiTRRT;
- EST, KPIECE and PRM;
- GCS and IRIS region decomposition;
- CHOMP and STOMP optimization;
- Cartesian planning and constrained RRT;
- shortcutting, smoothing and pipeline composition.

Each planner config has explicit maximum samples/nodes/iterations/duration and
seed. State validity is a trait returning typed evidence. The first
implementation supplies joint-limit and explicit constraint validity only.
Because no collision provider exists, every plan is
`ArmPlan<CollisionUnchecked>`.

Coordinated dual-arm planning remains `UnsupportedCapability` even if a
selectively ported source file contains a dual-arm implementation.

### 7.5 Reactive, grasp and task

Reactive surfaces include twist, joint-jog and pose-tracking servo; EMA,
Butterworth and no-filter policies; and RMP reach, joint-limit, singularity and
damping policies. Obstacle/self-collision policies report collision capability
unavailable rather than using infinite clearance or an empty scene.

Grasp surfaces include bounded antipodal, top-down, side and suction candidates
for project-owned primitive object geometry with deterministic ranking.
Reachability may be checked; collision filtering is unavailable, so candidates
remain `CollisionUnchecked`.

The task stage engine composes bounded move, Cartesian, gripper, pick, place,
sequence and recovery stages. It produces intents compatible with the existing
mission/action lifecycle and never treats cancel acceptance as terminal
cancellation.

## 8. Control and execution admission

`arm-control` validates one proposal in this order:

1. profile ID, group ID and exact order;
2. observation header quality, source/boot/sequence and exclusive freshness;
3. proposal generation and action semantic;
4. finite values, joint/actuator bounds and configured slew;
5. transmission projection where applicable;
6. collision typestate;
7. existing lease, deadline, lifecycle and cancellation state.

Direct torque uses fresh measured actuator position, zero desired velocity,
`kp = 0`, `kd = 0` and bounded feed-forward torque. Fixed-gain MIT uses
proposal position/velocity/torque plus immutable validated gains.

`SimExecutor` and `LogExecutor` consume unchecked products without sleeping,
I/O, stdout/stderr or hardware callbacks. The log executor appends bounded
typed records into caller-owned storage. A physical executor trait accepts
only `CollisionChecked`, for which this task exposes no production
constructor.

## 9. Damiao mapping

`contract-codec` adds a pure one-setpoint mapping to
`motor_damiao::MitCommand`. `motor-damiao` exposes validated MIT construction
or range access without changing its frame layout.

The mapper requires a fake-or-real verified `RuntimeLimits` context and rejects:

- resource/profile/group mismatch;
- NaN/Inf or f64-to-f32 overflow;
- values outside application bounds;
- values outside verified `PMAX/VMAX/TMAX`;
- gains outside the Damiao protocol ranges.

The existing driver still requires Enabled state, fresh feedback, no pending
feedback, verified config and authorized bus-set generation before encoding.

## 10. Provenance

Add a project notice/provenance record listing:

- KINETIC repository and reference commit
  `3dbaa61579c4359ce4c35f246fc83e1d3110b646`;
- upstream path and purpose for each substantially derived kernel;
- local destination and material changes;
- retained Apache-2.0 notice.

Each substantially derived source file has a concise header pointing to that
record. Native contracts, admission, configuration and error code do not
claim KINETIC origin.

## 11. Compatibility and rollback

- Existing angular `JointState` is not reinterpreted for prismatic joints.
- Protobuf additions are additive and use new field numbers/types.
- Existing YAML examples and normalized hashes remain unchanged.
- Existing Damiao MIT golden frames remain unchanged.
- Removing this child removes its new crates and additive arm messages; no
  production profile or physical state is changed.
- Host completion leaves collision, physical motion, realtime, floating-base
  and coordinated dual-arm qualification explicitly open.
