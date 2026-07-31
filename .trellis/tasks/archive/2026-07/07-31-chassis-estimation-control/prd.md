# Chassis state estimation and SE2 control

## Goal

Implement software-only H3 local-monotonic sample alignment, wheel+IMU 2D estimation with external pose correction, body velocity/heading control, and fixed-capacity timestamped SE(2) trajectory tracking with explicit degradation.

## Requirements

- Extend `chassis-model` with validated `BodyPose2`, locally timestamped sample
  metadata and explicit body-command limits without duplicating `Twist2`.
- Accept only samples already projected onto the local monotonic clock domain;
  reject future, stale, out-of-order and excessively skewed samples.
- Implement deterministic 2D dead reckoning from wheel-derived body velocity
  and IMU yaw rate, with configured IMU blend and optional/required external
  pose correction.
- Propagate input quality and return typed `Nominal`, `Degraded` or
  `Unavailable` estimator evidence. Wheel loss must never become numeric zero.
- Implement bounded body-velocity control with explicit acceleration and
  command saturation, plus heading hold using shortest-angle error.
- Implement a fixed-capacity, strictly time-ordered SE(2) trajectory with
  interpolation, feed-forward twist and a body-frame pose-error tracker.
- Require explicit caller time and `dt`; no clock reads, I/O, blocking,
  allocation or hidden resume behavior on steady-state paths.
- Keep SLAM, mapping, global planning, Orange Pi and physical chassis
  qualification out of scope.

## Acceptance Criteria

- [x] Shared pose/time/limit types reject non-finite, invalid range and invalid
  monotonic values without exposing mutation into invalid states.
- [x] Wheel/IMU samples are age/skew/quality gated and stale or future samples
  never advance state.
- [x] Nominal wheel+IMU integration and external correction are deterministic
  and tested across yaw wrap.
- [x] IMU loss follows an explicit wheel-only degraded policy or stop policy;
  wheel loss always returns unavailable.
- [x] Optional external pose loss is reported without stopping; required loss
  is degraded according to policy.
- [x] Body velocity and heading controllers enforce speed, yaw-rate and
  acceleration limits with explicit saturation evidence.
- [x] SE(2) trajectories reject empty, duplicate, unordered and over-capacity
  points; interpolation and completion are tested.
- [x] The tracker transforms world pose error into the body frame and combines
  it with feed-forward twist.
- [x] Estimator and controller hot paths allocate zero heap memory.
- [x] Architecture policy, warnings-denied Clippy and `tools/ci.sh` pass while
  all physical chassis gates remain open.

## Notes

- Dependencies: completed H1/H2 chassis model/kinematics and existing
  lifecycle/lease safety. This task produces commands but does not grant
  authority or bypass morphology feasibility.
