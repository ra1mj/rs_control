# Reinforcement-learning policy control runtime — implementation plan

## 1. Dependency gate

- [x] Verify the committed `07-31-kinetic-arm-stack` public contracts pass the
      full host gate required by this child.
- [x] Load `trellis-before-dev` before production edits.
- [x] Add exact `rten = "=0.24.0"` and, if required for graph inspection,
      exact `rten-onnx = "=0.24.0"` through workspace dependencies.
- [x] Disable unnecessary features and document the selected operator surface.
- [x] Run license/advisory/source checks before implementing the runtime.
- [x] Stop and return to planning if RTen cannot satisfy the dependency gate;
      do not select another backend automatically.

## 2. Crate and manifest contract

- [x] Scaffold and classify `rl-policy-control` as `application_module`.
- [x] Define strict versioned manifest structs with unknown-field rejection.
- [x] Implement bounded regular-file artifact loading with no symlink/path
      escape or external ONNX data.
- [x] Parse model/profile hashes and portable IDs.
- [x] Compute canonical policy identity and nonzero process-local generation.
- [x] Validate nonzero periods/deadlines, bounded queue/thread counts and
      `WithdrawProposal` fallback.
- [x] Add manifest good/base/bad and cross-field tests.

## 3. ONNX and RTen validation

- [x] Parse ONNX metadata before readiness.
- [x] Reject external weights, dynamic/unknown dimensions, unsupported dtype,
      duplicate tensor names and non-finite constants.
- [x] Load model from safe bounded bytes; do not call unsafe mmap.
- [x] Cross-check exact RTen input/output names, dtypes and shapes.
- [x] Match exact arm profile/group/joint/actuator order and DOF.
- [x] Validate optional recurrent tensor pairs and total element capacity.
- [x] Construct a caller-sized RTen thread pool and `RunOptions`; never rely on
      the global pool.

## 4. Observation encoder

- [x] Implement coherent profile/group-bound `PolicyObservation`.
- [x] Validate source/boot/sequence, exclusive freshness and quality.
- [x] Reject missing, mixed, future, stale, replayed and faulted inputs.
- [x] Implement exact manifest ordering and deterministic normalization.
- [x] Reject NaN/Inf, invalid inverse standard deviation and range overflow.
- [x] Prove absent values are never substituted with zero.

## 5. Action decoders

- [x] Implement absolute position decode.
- [x] Implement position residual against the same fresh observation.
- [x] Implement velocity decode.
- [x] Implement direct rotary actuator torque decode.
- [x] Implement binding-major actuator-space MIT `(q, dq, tau_ff)` decode.
- [x] Append immutable manifest `kp/kd`; reject any model tensor/layout that
      attempts to output gains.
- [x] Implement affine scaling, hard rejection, command projection and
      saturation evidence.
- [x] Implement per-field slew limits and reset state.
- [x] Tag every result with exact profile/group/order/policy generation and
      `CollisionUnchecked`.

Tests:

- [x] Golden values for all five action semantics.
- [x] Wrong shape/order/DOF/type, NaN/Inf, hard-bound and projected-bound cases.
- [x] MIT gain length/range/profile-cap mismatch.
- [x] Prismatic group rejection without complete rotary transmission.

## 6. Synchronous engine and recurrent state

- [x] Implement one synchronous `PolicyEngine` owned only by the worker.
- [x] Pre-resolve RTen node IDs and output ordering at startup.
- [x] Allocate/warm required tensors before exposing runtime readiness.
- [x] Run with the explicit RTen thread pool.
- [x] Advance recurrent state only after a valid on-time result.
- [x] Reset recurrent and slew state on every defined failure/generation edge.

## 7. Bounded worker

- [x] Implement named non-RT worker construction and clean shutdown.
- [x] Add bounded SPSC request/result queues with nonblocking submit/receive.
- [x] Carry request sequence, identities, generation and enqueue/start/finish
      timing.
- [x] Reject queue full/disconnect, late inference, stale request and wrong
      generation without refreshing validity.
- [x] Expose typed readiness/terminal reason and saturating counters without
      library stdout/stderr.
- [x] Test explicit worker exit and supervised subprocess loss.
- [x] Do not claim in-process recovery from release-profile panic abort.

## 8. Deterministic ONNX fixtures

- [x] Add a tiny stateless single-file ONNX affine policy and exact manifest.
- [x] Add a tiny recurrent ONNX policy and exact reset manifest.
- [x] Record fixture hashes and maximum bytes.
- [x] Keep any generation script development-only; CI loads checked-in
      artifacts without Python.
- [x] Mutate model/manifest identities, constants, dtype and graph dimensions
      in fail-closed tests.

## 9. Arm-control integration

- [x] Submit only fresh stable arm-stack observations.
- [x] Convert accepted inference output to project-owned arm proposals.
- [x] Verify profile/model reload invalidates pending requests, results,
      recurrent state and slew history.
- [x] Verify every policy proposal remains `CollisionUnchecked`.
- [x] Verify lease, lifecycle, cancellation and command deadline are still
      required outside the policy runtime.
- [x] Test semantic direct-torque/MIT projection with fake arm-control limits;
      do not import or construct a Damiao/CAN type.

## 10. Verification

Targeted:

```bash
cargo test -p rl-policy-control
cargo clippy -p rl-policy-control --all-targets --all-features -- -D warnings
cargo run --locked -p architecture-check -- check
cargo deny check
```

Cross-child and full:

```bash
cargo test -p arm-model -p arm-algorithms -p arm-control -p rl-policy-control
cargo metadata --no-deps --format-version 1
./tools/ci.sh
```

## 11. Completion gate

- [x] Every RPC requirement maps to accepted or rejected host evidence.
- [x] Exact RTen/model/profile identities are visible in typed readiness
      evidence.
- [x] Every failure path withdraws the proposal and cannot renew authority.
- [x] No hardware, KINETIC, Python runtime, GPU or online-training dependency
      is present.
- [x] No host result is described as physical/realtime qualification.
- [x] Commit and archive this child before the parent integration review.
