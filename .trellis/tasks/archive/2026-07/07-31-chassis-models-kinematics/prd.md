# Chassis SI models and kinematics

## Goal

Implement software-only H1/H2 body-frame SI types, constraints, feasibility/saturation evidence, and mecanum-4, omni-3/4, and swerve-4 forward/inverse kinematics with deterministic host tests.

## Requirements

- Reuse `message_model::Twist2` as the canonical finite body-frame command:
  right-handed `x` forward, `y` left and positive yaw counter-clockwise.
- Add application-owned SI geometry and wheel/module state types whose
  constructors reject non-finite, non-positive and singular inputs.
- Make feasibility explicit. Inverse kinematics must uniformly desaturate
  wheel speeds, preserve direction and report the applied scale.
- Implement mecanum-4 forward/inverse kinematics with documented wheel order
  and roller signs.
- Implement generic omni forward/inverse kinematics plus standard symmetric
  omni-3 and omni-4 layouts.
- Implement swerve-4 forward/inverse kinematics, nearest-angle drive reversal,
  steering-rate limiting, drive-speed saturation and zero-speed angle hold.
- Reject singular and near-singular geometry at construction, not on the
  realtime calculation path.
- Keep all steady-state calculations deterministic, bounded, allocation-free
  and independent of clocks, I/O and hardware.
- Keep Orange Pi and physical chassis qualification out of scope.

## Acceptance Criteria

- [x] Shared finite SI types, uniform desaturation evidence and common body
  fixture vectors are implemented and tested.
- [x] Mecanum-4 round-trips every unsaturated common fixture within tolerance.
- [x] Standard omni-3 and omni-4 round-trip every unsaturated common fixture.
- [x] Swerve-4 round-trips fixtures when steering can reach the requested angle.
- [x] Saturation preserves wheel/module ratios and reports a scale in `(0, 1)`.
- [x] Swerve reversal chooses an equivalent target within 90 degrees of the
  current angle; rate limits and zero-speed angle hold are tested.
- [x] Non-finite, non-positive, singular and near-singular inputs receive typed
  errors without panics.
- [x] Forward/inverse hot paths allocate zero heap memory.
- [x] New crates satisfy architecture policy and warnings-denied Clippy.
- [x] `tools/ci.sh` passes with hardware qualification still open.

## Notes

- Wheel outputs use radians per second; geometry uses metres; body translation
  uses metres per second; body and steering rotation use radians.
