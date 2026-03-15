# Reinforcement-learning policy control runtime

## Goal

Add a control-layer runtime for immutable trained-policy artifacts, bounded
observation/action contracts, watchdog/fallback and host-qualified inference
without direct hardware authority.

## Requirements

- **RPC-01:** Use exact-pinned RTen 0.24.0 to load ONNX policy artifacts on a
  named non-realtime CPU worker. No native ONNX Runtime library, GPU provider,
  Python interpreter or online-training dependency is part of production.
- **RPC-02:** Validate an immutable manifest before readiness: schema/runtime
  version, model SHA-256 and byte bound, exact tensor names/dtypes/static
  shapes, observation ordering, normalization, action semantics/scaling/
  bounds, inference period/deadline and fallback.
- **RPC-02A:** Accept only bounded single-file ONNX artifacts with embedded
  weights and static `f32` tensors. Reject external-data files, dynamic
  dimensions and unsafe memory-mapped model loading.
- **RPC-03:** Reject malformed models, unsupported/dynamic tensors, hash/shape/
  type mismatch, non-finite constants and robot-DOF mismatch at startup.
- **RPC-03A:** Bind every policy manifest, observation snapshot and output to
  the immutable compiled arm-profile identity, stable joint order and DOF.
  Switching morphology is a disarmed reload that invalidates queued outputs
  and recurrent state; a proposal can never be reused across profiles.
- **RPC-03B:** Bind each policy to one exact named kinematic group in the first
  delivery. The schema does not assume a globally unique arm and reserves
  typed extension to composite dual-arm/floating-base observations; unsupported
  composite policies fail validation rather than dropping joints or base
  state.
- **RPC-04:** Inference consumes one coherent semantic observation snapshot.
  Missing, stale, replayed or faulted required input skips the policy and
  produces typed unavailable/degraded evidence rather than substituted zeros.
  The only v1 fallback is `WithdrawProposal`; no failure manufactures a fresh
  hold, zero, velocity or torque command.
- **RPC-05:** Validate finite exact-shape output and project it through
  project-owned joint and actuator-binding limits. Policy output is a proposal
  and owns no lease, local deadline, lifecycle transition or hardware command.
- **RPC-06:** Bind readiness, outputs and optional recurrent state to a model
  generation/hash. Reload is startup/disarmed only and invalidates prior state.
- **RPC-07:** Use bounded request/result queues, a configured RTen thread count
  and a local inference deadline. Full queues, late/error results,
  worker/channel or supervised process loss fail closed without refreshing
  command validity. Release-profile panic abort is not presented as recoverable
  inside the same process.
- **RPC-08:** Provide a deterministic tiny ONNX fixture and tests for valid
  normalization/inference/action decoding, every manifest/model failure,
  non-finite/wrong-shape output, bounds, timeout, queue saturation, reload and
  fallback.
- **RPC-09:** The manifest selects one exact action semantic from a closed
  allowlist: absolute joint position, joint-position residual, joint velocity,
  direct rotary-actuator torque in N·m, or MIT impedance control. MIT uses a
  project-owned SI tuple `(position_rad, velocity_radps, kp, kd,
  feed_forward_torque_nm)` in stable compiled joint-to-actuator binding order.
  The learned MIT tensor contains position, velocity and feed-forward torque
  only; validated fixed per-actuator `kp/kd` values are immutable manifest data
  tied to the model generation and cannot be changed dynamically by inference.
- **RPC-10:** Direct-torque and MIT results remain typed proposals. A policy
  cannot choose raw CAN bytes, bus, motor ID or protocol opcode, and the RL
  crate cannot depend on the Damiao hardware crate. Mapping an admitted
  proposal to `MitCommand` remains at the protocol/hardware boundary.
- **RPC-11:** Validate every torque/MIT field against finite manifest bounds,
  compiled joint-to-actuator binding limits and configured slew/rate limits.
  Bounds must be no wider than the driver's verified runtime
  `PMAX/VMAX/TMAX`, with `0 <= kp <= 500` and `0 <= kd <= 5`; the driver still
  enforces verified configuration, fresh feedback, enabled state and bus-set
  generation.
- **RPC-12:** Direct torque and MIT receive deterministic host-only
  qualification. The task makes no physical-motion, contact-stability or
  force/torque-accuracy claim.

## Dependencies

- Depends on the project-owned bounded joint/model types delivered by
  `07-31-kinetic-arm-stack`; it must not import KINETIC, URDF or upstream
  dynamic `Vec` types directly.
- Consumes the arm stack's immutable multi-morphology profile identity and
  stable joint ordering; no RL tensor layout is hard-coded to six joints.
- Depends on existing `control-core` resource lease, lifecycle and action
  cancellation semantics; it does not replace them.
- Depends on a project-owned semantic admission/mapping boundary for
  direct-torque and MIT proposals; it never imports or constructs hardware
  wire types.

## Acceptance Criteria

- [x] A checked-in manifest and deterministic fixture ONNX model load with
      exact identity and tensor-contract evidence.
- [x] Fresh semantic observations produce a bounded typed policy proposal with
      the expected golden values.
- [x] Every malformed/stale/late/non-finite/wrong-generation case yields the
      exact configured fallback and cannot refresh authority.
- [x] Golden tests decode and constrain direct-torque and MIT outputs in stable
      compiled joint-to-actuator binding order, including SI units,
      gain/effort/rate limits and rejection before raw motor framing.
- [x] Model load and inference stay outside hardware/protocol crates and the
      allocation-free control hot path.
- [x] Architecture, Clippy, tests, dependency policy and all non-OrangePi host
      CI gates pass.
