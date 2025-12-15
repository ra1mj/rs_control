# Reinforcement-learning policy control runtime — implementation plan

## 1. Dependency gate

- [ ] Verify `07-31-kinetic-arm-stack` is completed and its public contracts
      pass the full host gate.
- [ ] Load `trellis-before-dev` before production edits.
- [ ] Add exact `rten = "=0.24.0"` and, if required for graph inspection,
      exact `rten-onnx = "=0.24.0"` through workspace dependencies.
- [ ] Disable unnecessary features and document the selected operator surface.
- [ ] Run license/advisory/source checks before implementing the runtime.
- [ ] Stop and return to planning if RTen cannot satisfy the dependency gate;
      do not select another backend automatically.

## 2. Crate and manifest contract

- [ ] Scaffold and classify `rl-policy-control` as `application_module`.
- [ ] Define strict versioned manifest structs with unknown-field rejection.
- [ ] Implement bounded regular-file artifact loading with no symlink/path
      escape or external ONNX data.
- [ ] Parse model/profile hashes and portable IDs.
- [ ] Compute canonical policy identity and nonzero process-local generation.
- [ ] Validate nonzero periods/deadlines, bounded queue/thread counts and
      `WithdrawProposal` fallback.
- [ ] Add manifest good/base/bad tests for every field.

## 3. ONNX and RTen validation

- [ ] Parse ONNX metadata before readiness.
- [ ] Reject external weights, dynamic/unknown dimensions, unsupported dtype,
      duplicate tensor names and non-finite constants.
- [ ] Load model from safe bounded bytes; do not call unsafe mmap.
- [ ] Cross-check exact RTen input/output names, dtypes and shapes.
- [ ] Match exact arm profile/group/joint/actuator order and DOF.
- [ ] Validate optional recurrent tensor pairs and total element capacity.
- [ ] Construct a caller-sized RTen thread pool and `RunOptions`; never rely on
      the global pool.

## 4. Observation encoder

- [ ] Implement coherent profile/group-bound `PolicyObservation`.
- [ ] Validate source/boot/sequence, exclusive freshness and quality.
- [ ] Reject missing, mixed, future, stale, replayed and faulted inputs.
- [ ] Implement exact manifest ordering and deterministic normalization.
- [ ] Reject NaN/Inf, invalid inverse standard deviation and range overflow.
- [ ] Prove absent values are never substituted with zero.

## 5. Action decoders

- [ ] Implement absolute position decode.
- [ ] Implement position residual against the same fresh observation.
- [ ] Implement velocity decode.
- [ ] Implement direct rotary actuator torque decode.
- [ ] Implement binding-major actuator-space MIT `(q, dq, tau_ff)` decode.
- [ ] Append immutable manifest `kp/kd`; reject any model tensor/layout that
      attempts to output gains.
- [ ] Implement affine scaling, hard rejection, command projection and
      saturation evidence.
- [ ] Implement per-field slew limits and reset state.
- [ ] Tag every result with exact profile/group/order/policy generation and
      `CollisionUnchecked`.

Tests:

- [ ] Golden values for all five action semantics.
- [ ] Wrong shape/order/DOF/type, NaN/Inf, hard-bound and projected-bound cases.
- [ ] MIT gain length/range/profile-cap mismatch.
- [ ] Prismatic group rejection without complete rotary transmission.

## 6. Synchronous engine and recurrent state

- [ ] Implement one synchronous `PolicyEngine` owned only by the worker.
- [ ] Pre-resolve RTen node IDs and output ordering at startup.
- [ ] Allocate/warm required tensors before exposing runtime readiness.
- [ ] Run with the explicit RTen thread pool.
- [ ] Advance recurrent state only after a valid on-time result.
- [ ] Reset recurrent and slew state on every defined failure/generation edge.

## 7. Bounded worker

- [ ] Implement named non-RT worker construction and clean shutdown.
- [ ] Add bounded SPSC request/result queues with nonblocking submit/receive.
- [ ] Carry request sequence, identities, generation and enqueue/start/finish
      timing.
- [ ] Reject queue full/disconnect, late inference, stale request and wrong
      generation without refreshing validity.
- [ ] Expose typed readiness/terminal reason and saturating counters without
      library stdout/stderr.
- [ ] Test explicit worker exit and supervised subprocess loss.
- [ ] Do not claim in-process recovery from release-profile panic abort.

## 8. Deterministic ONNX fixtures

- [ ] Add a tiny stateless single-file ONNX affine policy and exact manifest.
- [ ] Add a tiny recurrent ONNX policy and exact reset manifest.
- [ ] Record fixture hashes and maximum bytes.
- [ ] Keep any generation script development-only; CI loads checked-in
      artifacts without Python.
- [ ] Mutate every model/manifest identity and graph dimension in tests.

## 9. Arm-control integration

- [ ] Submit only fresh stable arm-stack observations.
- [ ] Convert accepted inference output to project-owned arm proposals.
- [ ] Verify profile/model reload invalidates pending requests, results,
      recurrent state and slew history.
- [ ] Verify every policy proposal remains `CollisionUnchecked`.
- [ ] Verify lease, lifecycle, cancellation and command deadline are still
      required outside the policy runtime.
- [ ] Test semantic direct-torque/MIT projection with fake arm-control limits;
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

- [ ] Every RPC requirement maps to an accepted and rejected test.
- [ ] Exact RTen/model/profile identities are visible in typed readiness
      evidence.
- [ ] Every failure path withdraws the proposal and cannot renew authority.
- [ ] No hardware, KINETIC, Python runtime, GPU or online-training dependency
      is present.
- [ ] No host result is described as physical/realtime qualification.
- [ ] Commit and archive this child before the parent integration review.
