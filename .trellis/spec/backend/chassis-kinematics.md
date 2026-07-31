# Rust Chassis Models and Kinematics

## Scenario: Implement or consume planar chassis motion

### 1. Scope / Trigger

Use this contract for body/wheel SI types, chassis geometry, mecanum/omni/swerve
forward or inverse kinematics, wheel constraints, steering optimization and
the H3 estimator/controller boundary.

### 2. Ownership and signatures

```text
chassis-model     canonical SI values, validation, saturation, least squares
chassis-mecanum  mecanum-4 wheel order and analytic maps
chassis-omni     arbitrary omni maps and symmetric omni-3/omni-4 layouts
chassis-swerve   four-module maps, reversal and steering-rate limits
```

```rust
Mecanum4::inverse(body) -> Result<KinematicSolution<WheelAngularSpeeds<4>>, _>
OmniKinematics<N>::inverse(body)
    -> Result<KinematicSolution<WheelAngularSpeeds<N>>, _>
Swerve4::inverse(body, previous_modules, dt_s)
    -> Result<SwerveInverseSolution, _>
```

`message_model::Twist2` remains the canonical finite body command and is
re-exported by `chassis-model` as `BodyTwist`; do not add another planar twist
with competing validation.

### 3. Coordinate, units and order

- The body frame is right-handed: x forward, y left and positive yaw
  counter-clockwise.
- Positions and radii are metres, translation is metres/second, wheel and body
  angular speeds are radians/second, and time steps are seconds.
- Mecanum and rectangular swerve arrays are always front-left, front-right,
  rear-left, rear-right.
- Omni arrays preserve declaration order. Each declared angle is the positive
  drive-tangent direction, not the roller axis or radial direction.
- A positive swerve drive speed acts along its module steering angle.

### 4. Validation and numerical contracts

- Public constructors reject non-finite values. Dimensions, limits and
  caller-supplied delta time must be positive.
- Wheel radius, chassis radius and rectangular half-dimensions below
  `MIN_EFFECTIVE_LENGTH_M` are near singular.
- Arbitrary omni/swerve geometry is converted into a fixed design matrix.
  Matrix columns are normalized before Gram determinant checks so metre and
  dimensionless columns receive a meaningful rank test.
- Singular or near-singular geometry is rejected at construction. Runtime
  calculation never invents a partial degree-of-freedom answer.
- Least-squares matrix storage and all wheel/module arrays have compile-time
  sizes. Construction and forward/inverse hot paths allocate nothing, read no
  clock and perform no I/O.
- Finite inputs whose arithmetic overflows return `NumericalOverflow`.

### 5. Constraint evidence

Inverse mecanum/omni maps calculate unconstrained wheel angular velocities,
then apply one scale to every wheel. Swerve does the same after steering
reachability is applied. Component-wise clipping is forbidden.

`Feasibility::Feasible` means scale 1. `Feasibility::Saturated` contains
read-only `Saturation` evidence whose scale is in `(0, 1)`. Public callers
cannot construct arbitrary saturation evidence.

Forwarding the uniformly saturated mecanum or omni wheel set must recover the
requested body twist multiplied by that scale. Swerve returns its explicit
least-squares `achieved_body`, because a rate-limited steering angle may not
span the requested module vector during that update.

### 6. Morphology maps

For mecanum X rollers with `lever = half_length + half_width`, positive body
strafe produces wheel signs `[-, +, +, -]` and positive yaw produces
`[-, +, -, +]` in FL/FR/RL/RR order.

For an omni wheel at `(x, y)` with positive tangent
`(cos(theta), sin(theta))`:

```text
wheel_linear = cos(theta) * vx
             + sin(theta) * vy
             + (-cos(theta) * y + sin(theta) * x) * yaw_rate
wheel_radps = wheel_linear / wheel_radius
```

Symmetric omni layouts place modules around a common chassis radius and drive
tangentially.

For a swerve module at `(x, y)`:

```text
module_vx = vx - yaw_rate * y
module_vy = vy + yaw_rate * x
```

If adding pi to a desired module angle reduces the absolute steering delta
below 90 degrees, reverse drive direction. Bound the remaining delta by
`max_steer_rate_radps * dt_s`, project the desired module vector onto that
reachable angle, then apply uniform drive saturation. A zero desired vector
sets drive speed to zero and holds the previous normalized angle exactly.

### 7. Error and evidence matrix

| Condition | Required result |
|---|---|
| NaN/infinite geometry, state or observation | `NonFinite` |
| Zero/negative radius, dimension, limit or `dt` | `NonPositive` |
| Nonzero but ineffective geometry length | `NearSingularGeometry` |
| Rank-deficient design matrix | `SingularGeometry` |
| Poorly conditioned normalized matrix | `NearSingularGeometry` |
| Finite arithmetic exceeds numeric range | `NumericalOverflow` |
| Wheel limit exceeded | uniform scale plus immutable saturation evidence |
| Swerve angle cannot be reached this step | `steering_limited`, achieved body |
| Zero swerve vector | zero drive, exact prior angle |

### 8. Tests required

- Reuse `COMMON_BODY_FIXTURES` for every morphology's unsaturated round trip.
- Cover pure body axes, combined motion, reversal symmetry and zero.
- Verify saturation scale, output limits and preserved ratios/direction.
- Cover mecanum signs, both standard omni layouts and swerve 180-degree
  optimization/rate limiting/angle hold.
- Reject non-finite, non-positive, singular and near-singular inputs.
- Measure zero allocations on each morphology's forward/inverse path.
- Run warnings-denied Clippy, architecture policy and `tools/ci.sh`.
- Keep lifted-chassis, real wheel-order, CAN-load and traction validation open
  until physical hardware is available; host math tests do not close them.
