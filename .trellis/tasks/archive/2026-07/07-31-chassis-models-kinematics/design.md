# Design: chassis models and kinematics

## Ownership

Create the application-layer crates already named in the parent architecture:

- `chassis-model`: canonical body alias, geometry primitives, bounded wheel
  arrays, typed errors, uniform desaturation and a validated planar
  least-squares solver.
- `chassis-mecanum`: mecanum-4 wheel-order/sign contract and analytic maps.
- `chassis-omni`: arbitrary validated omni layouts plus symmetric 3/4-wheel
  constructors.
- `chassis-swerve`: rectangular/custom four-module maps, steering
  optimization and rate limits.

The morphology crates depend only on `chassis-model`; `chassis-model` depends
on `message-model` so it can reuse the already validated `Twist2` contract.

## Coordinate and wheel conventions

The body frame is right-handed: `x` forward, `y` left, positive yaw
counter-clockwise. Wheel and module order is stable and public.

- Mecanum: front-left, front-right, rear-left, rear-right. Positive wheel
  angular velocity drives its contact tangent forward.
- Omni: each row declares module position and the positive drive-tangent angle.
- Swerve: front-left, front-right, rear-left, rear-right. A module state is
  signed drive speed plus normalized steering angle.

## Shared numerical contract

All external floats are validated once by constructors. Geometry validation
normalizes design-matrix columns before rank checking so very different
translation/rotation units do not create a false rank decision. The validated
least-squares object stores its fixed-size normalized normal-matrix inverse;
forward calculations then use only stack arrays.

Inverse calculations return `KinematicSolution<T>` with:

- `Feasible` when no wheel limit changed the result.
- `Saturated { scale }` when every drive speed was multiplied by one uniform
  scale in `(0, 1)`.

Uniform scaling preserves the requested body direction for linear
kinematics. No component-wise clipping is allowed.

## Morphology equations

Mecanum uses the standard four-row map with
`lever = half_length + half_width`, divided by wheel radius. Forward uses the
closed-form pseudoinverse and rejects a near-zero radius or lever at startup.

For an omni wheel at `(x, y)` whose positive tangent unit vector is
`(cos(theta), sin(theta))`:

```text
wheel_linear = cos(theta) * vx
             + sin(theta) * vy
             + (-cos(theta) * y + sin(theta) * x) * yaw_rate
```

Inverse evaluates these rows; forward solves the validated least-squares
system.

For a swerve module at `(x, y)`:

```text
module_vx = vx - yaw_rate * y
module_vy = vy + yaw_rate * x
```

The desired vector becomes speed/angle. If rotating the target by pi makes
the steering move shorter, drive speed is reversed. The angle delta is then
bounded by `max_steer_rate * dt`; drive speed is the desired module vector
projected onto the reachable angle. A zero desired vector holds the prior
angle exactly. Finally all drive speeds are uniformly saturated.

## Failure behavior

- Non-finite values and non-positive dimensions/limits/time step: typed error.
- Rank-deficient or near-rank-deficient design matrix: typed construction
  error; no runtime pseudo-answer.
- Zero command: zero wheel speed, and for swerve retain prior angles.
- Steering unable to reach the requested target in this update: return
  `steering_limited = true` and the least-squares achieved body twist.
- No clock reads or implicit delta time; the caller supplies a validated `dt`.

## Verification

The four crates share the same pure translation, strafe, rotation, diagonal
and combined body fixtures. Tests cover round trips, sign/reversal symmetry,
saturation ratios, zero behavior, singular/near-singular geometry, steering
optimization/rate limiting and zero heap allocation. The final gate is
warnings-denied Clippy, architecture check and `tools/ci.sh`.
