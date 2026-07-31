# Rust Chassis Estimation and SE(2) Control

## Scenario and ownership

Use this contract for locally timed chassis samples, planar state estimation,
external pose correction, body/heading control and SE(2) tracking.

```text
chassis-model        BodyPose2, TimedSample, body command limits/evidence
state-estimation-2d  time/quality gate, wheel+IMU fusion, pose correction
trajectory-se2       fixed trajectory, velocity/heading/tracking controllers
```

These application crates consume semantic values only. They do not import
generated DTOs, concrete middleware, transports or hardware drivers.

## Local time and input gate

`TimedSample<T>` means the acquisition time has already been projected into
the consumer's local monotonic domain by an owning protocol/composition
boundary. Never compare independent source-monotonic timestamps in H3.

Every estimator call supplies `now_monotonic_ns`. Each input is rejected when:

- its timestamp is in the future;
- age exceeds its configured maximum;
- timestamp is less than or equal to the last accepted timestamp;
- quality is stale, calibrating or faulted;
- wheel/IMU skew exceeds the configured maximum.

Rejected or unavailable steps do not change pose, state time or any accepted
input timestamp. Accepted timestamps commit only after the entire estimator
step succeeds.

## Estimator and degradation

Wheel-derived `BodyTwist` is required and is never synthesized as zero when
missing. A usable IMU yaw rate is blended with wheel yaw at the configured
closed-unit weight. IMU loss either:

- advances using wheel yaw and returns degraded evidence; or
- returns unavailable without advancing.

Translation is integrated in the world frame using midpoint yaw. External
pose correction is a gain-bounded interpolation, including shortest-arc yaw;
it is not an implicit reset. Optional loss is reported but does not by itself
degrade wheel/IMU odometry. Required loss marks the result degraded.

Output is `Nominal`, `Degraded` with exact evidence, or `Unavailable` with a
typed source/reason. No degraded state may be presented as nominal.

## Control and trajectory

`BodyCommandLimits` contains finite positive translational/yaw speed and
acceleration limits. `constrain_body_command` first bounds change from the
previous command, then absolute speed. It returns immutable per-limit
evidence; silent component clipping is forbidden.

Velocity control adds proportional velocity error to feed-forward.
Heading control uses shortest-angle yaw error. The SE(2) tracker transforms
world translation error into the measured body frame, adds pose and velocity
feedback to feed-forward, then uses the common limiter.

`Se2Trajectory`:

- stores at most 256 points in fixed capacity;
- rejects empty, zero-time, duplicate or decreasing offsets;
- linearly interpolates x/y and feed-forward twist;
- interpolates yaw on the shortest arc;
- returns the final point as `Complete` at or after duration.

Controllers and trajectory sampling receive explicit `dt`/elapsed time. They
read no clock, allocate no heap memory, perform no I/O and own no lease,
lifecycle transition, morphology projection or motor command.

## Error/evidence matrix

| Condition | Required result |
|---|---|
| Future/stale/replayed sample | typed input rejection; no state change |
| Wheel unavailable | estimate unavailable |
| IMU unavailable, wheel-only policy | degraded advance |
| IMU unavailable, stop policy | unavailable; no advance |
| Optional external pose unavailable | evidence, wheel/IMU quality unchanged |
| Required external pose unavailable | degraded advance |
| Invalid/oversized trajectory | typed construction rejection |
| Speed/acceleration exceeded | bounded command plus exact limit evidence |
| Estimator/controller arithmetic overflow | typed error; no partial commit |

## Tests required

- Cover age boundaries, future/replay/skew/quality rejection and non-advance.
- Cover nominal fusion, every IMU/external policy and degraded propagation.
- Cover midpoint integration, external correction and yaw wrap.
- Cover trajectory capacity/order/interpolation/completion.
- Cover velocity, heading, body-frame tracker and all command limits.
- Measure zero allocations for estimator and controller/sampling hot paths.
- Run warnings-denied Clippy, architecture policy and `tools/ci.sh`.
- Keep real sensor timing, lifted chassis, CAN load and traction gates open
  until physical hardware evidence exists.
