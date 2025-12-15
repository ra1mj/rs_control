# Reinforcement-learning policy control runtime — technical design

## 1. Scope and explicit dependencies

This child deploys already-trained policy artifacts. It does not train,
fine-tune or construct a simulation environment.

It starts only after `07-31-kinetic-arm-stack` completes and provides:

- stable `ArmProfileId` and `KinematicGroupId`;
- bounded profile/group joint observations;
- stable joint/actuator ordering and transmission eligibility;
- project-owned position, velocity, direct-torque and MIT proposal types;
- arm-control constraint/slew and collision-evidence admission.

The crate must not create local substitutes for those types. It depends on
`arm-model`, `arm-control`, `message-model` and registry RTen crates only. It
does not depend on KINETIC, URDF parsing, `motor-damiao`, SocketCAN or a
middleware backend.

## 2. Crate and ownership

```text
crates/application_module/rl-policy-control/
  artifact.rs       # bounded manifest/model loading and identity
  manifest.rs       # strict versioned TOML contract
  graph.rs          # ONNX and RTen metadata validation
  observation.rs   # coherent snapshot encoding/normalization
  action.rs         # semantic decode, hard bounds and projection
  engine.rs         # synchronous RTen engine owned by worker
  worker.rs         # bounded SPSC request/result runtime
  evidence.rs       # typed readiness/degradation/counters
```

All file I/O, TOML/ONNX parsing, tensor allocation, graph optimization and
inference are non-RT. The control-facing API exchanges fixed-capacity semantic
snapshots/proposals and never calls RTen.

## 3. Exact dependency policy

The production backend is:

```toml
rten = "=0.24.0"
rten-onnx = "=0.24.0" # only when needed for pre-load graph inspection
```

RTen default global threading is not used for inference. The runtime constructs
`ThreadPool::with_num_threads(manifest.thread_count)` and passes it through
`RunOptions`.

Model loading uses safe bounded bytes with `Model::load` or safe
`Model::load_file`. `Model::load_mmap` is forbidden because it is unsafe and
the workspace forbids unsafe production code.

The dependency gate reviews features, licenses, advisories, duplicate versions
and the final operator set. Failure leaves the policy runtime unavailable; no
ORT, tract, Python or GPU fallback is selected.

## 4. Artifact layout and manifest

One immutable v1 artifact directory contains exactly:

```text
manifest.toml
model.onnx
```

Both must be regular files. Symlinks, extra external-weight files and path
escape are rejected. The loader bounds manifest and model bytes before reading.

The strict `deny_unknown_fields` manifest records:

```text
schema_version = 1
runtime = "rten"
runtime_version = "0.24.0"
model_format = "onnx"
model_sha256
max_model_bytes

arm_profile_sha256
kinematic_group_id
joint_order[]
actuator_order[]

inputs[] / outputs[]
  name, dtype=f32, exact static shape

observations[]
  source semantic, joint/field order, units
  mean, inverse_std, hard finite range

action
  kind
  output tensor/layout
  affine scaling
  hard reject bounds
  projected command bounds
  slew bounds
  fixed kp/kd for MIT

inference_period_ns
inference_deadline_ns
queue_capacity
thread_count
fallback = "withdraw_proposal"
optional recurrent tensor pairs and explicit reset values
```

The policy identity is SHA-256 over canonical manifest semantics plus model
SHA-256. A process-local nonzero model generation increments on every load,
including reload of identical bytes. Every request, recurrent state and result
carries both identity and generation.

## 5. Startup validation

Startup completes before publishing readiness:

1. Reject missing, non-regular, oversized or unexpected artifact files.
2. Strictly parse and version-check the manifest.
3. Parse hexadecimal hashes/identities and portable IDs.
4. Hash bounded model bytes and compare exactly.
5. Parse ONNX metadata and reject external data, dynamic/unknown dimensions,
   unsupported dtype, duplicate names and non-finite floating constants.
6. Load RTen from safe bytes; unsupported/corrupt operators fail here.
7. Cross-check RTen node IDs, names, dtypes and static shapes against the
   manifest.
8. Match the exact compiled arm profile, group, joint/actuator order and DOF.
9. Validate normalization, scaling, hard/projected bounds, slew, periods,
   deadline, queue/thread counts and recurrent tensor pairs.
10. For MIT, require a complete rotary actuator/transmission binding and
    per-joint immutable finite nonnegative `kp/kd`.
11. Validate gains and command bounds against project `arm-control` caps.
    Hardware protocol/runtime limits remain a later admission gate.

Only the resulting `ReadyPolicy` may spawn a worker.

## 6. Observation snapshot

One `PolicyObservation` contains:

- exact profile ID, group ID and stable joint/actuator order;
- source/boot/sequence identity;
- one coherent local observation time and exclusive expiry;
- semantic quality;
- bounded required state fields;
- optional explicitly declared non-joint observations.

The encoder rejects:

- missing fields or sources;
- mixed profile/group/order;
- stale, future, replayed or reordered samples;
- stale/calibrating/faulted quality and undeclared degraded quality;
- NaN/Inf or hard observation range violation.

No absent value becomes numeric zero. Normalization is deterministic:

```text
normalized = (value - mean) * inverse_std
```

All manifest coefficients are finite and `inverse_std > 0`. The encoder checks
the normalized result for finiteness before converting to `f32`.

## 7. Action decoding

The manifest chooses exactly one action kind:

| Kind | Tensor length | Decode |
|---|---:|---|
| absolute position | DOF | group joint positions |
| position residual | DOF | fresh measured position + residual |
| velocity | DOF | group joint velocities |
| direct rotary torque | actuator DOF | actuator torque in N·m |
| MIT impedance | `3 * actuator DOF` | binding-major actuator-space `(q_rad, dq_radps, tau_ff_nm)` |

MIT `kp/kd` is not in the ONNX output. It is immutable per-actuator manifest
data bound to the policy identity/generation and appended after model decode.

Each raw output follows:

1. exact tensor name/dtype/shape;
2. `f32` to `f64` conversion and finite check;
3. affine scale/offset;
4. hard reject bound;
5. projection into narrower command/profile bounds with saturation evidence;
6. previous-accepted-output slew limit;
7. exact profile/group/order/generation tagging.

An output beyond a hard bound withdraws the proposal. A value between hard and
command bounds is projected and reports typed saturation. Rate state resets on
startup, reload, timeout, invalid output, worker loss or profile change.

Every learned motion result is an
`ArmControlProposal<CollisionUnchecked>`. It may be simulated/logged but cannot
enter physical admission in this delivery.

## 8. Worker and timing model

`PolicyWorker` owns:

- `rten::Model`;
- a fixed-size RTen thread pool;
- preallocated tensor work buffers after warmup where RTen permits;
- recurrent tensors;
- one bounded request SPSC queue and one bounded result SPSC queue;
- saturating counters and typed terminal status.

The producer uses nonblocking `try_submit`; the consumer uses nonblocking
`try_receive`. Queue capacity is bounded to a small reviewed range. Full queues
return typed `Full` and never block or allocate in the control-facing call.

A request contains `std::time::Instant` enqueue time, sequence, profile/group
identity, policy identity/generation and observation. A result includes start,
finish and inference duration. The receiver rejects results when:

- request age or inference duration reaches the exclusive deadline;
- request sequence is not the expected current request;
- policy/profile/group/generation differs;
- a newer profile/model generation is active.

The production release profile aborts on panic, so the design does not pretend
to catch unwinding. Worker/channel disconnect and supervised process loss are
observed as unavailable by the consumer. Tests exercise explicit worker exit
and subprocess loss rather than claiming panic recovery inside one process.

## 9. Recurrent policies

The manifest may declare paired static `f32` recurrent input/output tensors.
Every pair has:

- exact names and equal bounded shapes;
- an explicit finite reset vector;
- a maximum total recurrent element count.

State is private to the worker. It advances only after a completely valid,
on-time inference result is accepted. It resets on startup, reload, timeout,
invalid output, queue discontinuity, worker loss and profile switch. Old state
cannot cross a generation.

## 10. Failure and fallback

The v1 fallback enum accepts only `WithdrawProposal`.

```text
missing/stale observation
queue full/disconnected
model error or invalid output
deadline reached
wrong generation
reload/restart
    -> no proposal
    -> typed reason + saturating counter
    -> existing command deadline is not renewed
    -> downstream driver/lifecycle owns safe stop
```

The RL runtime never emits a fresh zero torque, hold position or last-value
command during failure. This avoids extending authority and avoids treating
missing state as zero.

## 11. Test artifacts and qualification

Check in tiny single-file ONNX fixtures with exact manifests/hashes:

- stateless affine policy for golden normalization/action decode;
- minimal recurrent policy for state advancement and reset.

Use a small operator set supported by RTen. Fixture generation tools are
development-only and are not a production Python dependency.

Tests mutate each artifact dimension independently: bytes/hash, file kind,
schema/version, names, dtype, shapes, dimensions, constants, DOF/order,
profile/group, normalization, action bounds, gains, deadline, queue and
recurrent pairing.

Host evidence proves software behavior only. It does not establish RK3588
latency, physical torque/MIT stability, contact safety or real-arm performance.

## 12. Rollback

The RL crate is leaf-like. Rollback removes it and its test artifacts while
retaining the native arm contracts. No policy artifact is loaded during robot
YAML parsing, and no driver configuration or hardware state is changed.
