# Design: chassis state estimation and SE(2) control

## Ownership

- `chassis-model`: `BodyPose2`, `TimedSample<T>`, local sample quality and
  validated body command/rate limits.
- `state-estimation-2d`: input gate, degradation policy, dead reckoning and
  external correction.
- `trajectory-se2`: fixed-capacity trajectory, sampler, body velocity
  controller, heading controller and SE(2) tracker.

Both new crates are `application_module`. They depend only on `chassis-model`;
no generated DTO, middleware backend or hardware type enters the algorithm.

## Local time and quality

`TimedSample<T>` carries a value, local monotonic sample time and semantic
quality. A protocol/composition boundary is responsible for converting
external acquisition time using synchronization evidence before construction.
H3 never directly compares independent source clocks.

Each estimator step receives `now_monotonic_ns`. The gate rejects:

- sample time in the future;
- age above the per-input maximum;
- timestamps older than the last accepted timestamp for that input;
- wheel/IMU skew above the configured bound;
- stale, calibrating or faulted quality.

Degraded samples may be consumed only when the configured policy allows them,
and the output remains degraded.

## Estimator

The estimator owns `BodyPose2`, body velocity and the last step timestamp.
Wheel velocity is required. IMU yaw rate either replaces/blends wheel yaw rate
or follows explicit `WheelOnly`/`Stop` loss policy. Translation is integrated
in the world frame using the midpoint yaw for the step.

An external pose correction is `Optional` or `Required`. A usable correction
applies configured gains to x/y and shortest yaw error; it is never an
implicit state reset. Missing optional correction is evidence but not
degradation. Missing required correction is degraded. A correction older than
the current estimator state is rejected.

The result carries `Nominal`, `Degraded(reasons)` or
`Unavailable(reason)`. Unavailable steps do not advance pose or accepted input
timestamps.

## Controllers and trajectory

`BodyCommandLimits` validates positive linear/yaw speed and acceleration
limits. The velocity controller applies proportional error around target
feed-forward, then bounds acceleration from its prior command and finally
bounds translational magnitude and yaw rate. It returns immutable saturation
flags.

The heading controller computes shortest-angle error and replaces the target
yaw rate with bounded feed-forward plus proportional correction.

`Se2Trajectory` stores at most 256 strictly increasing points in `ArrayVec`.
Each point has nonzero `time_from_start_ns`, pose and feed-forward body twist.
Sampling linearly interpolates x/y/twist and interpolates yaw on the shortest
arc. Sampling after the final point returns `Complete` with the final point.

The tracker converts world-frame x/y error to the measured body frame, applies
finite nonnegative gains, adds feed-forward twist, then passes the result
through the same velocity/rate limiter. The tracker does not own a clock,
lease, morphology or motor command.

## Failure and verification

All constructors return typed errors. Calculation overflow is explicit.
Tests cover timestamp boundary values, reorder, skew, quality, every loss
policy, integration, correction/yaw wrap, command limits, trajectory
validation/interpolation/completion, tracker frame rotation and zero
allocation. Host tests do not close lifted-chassis or real-sensor gates.
