# Arm Payload Identification and Adaptive Gravity Compensation

## Purpose and safety boundary

This document defines the implemented process for identifying a fixed tool
payload and using it as bounded gravity feed-forward. The implementation is a
deterministic host qualification facility. It does not grant motion authority,
construct hardware frames, open ROS or device endpoints, or establish
physical-arm accuracy or stability.

Adaptive effort remains an ordinary `ArmControlProposal`. The existing lease,
deadline, lifecycle, observation freshness, fixed gain, magnitude, slew and
collision-evidence boundaries remain authoritative.

## Identified model

The nominal URDF link inertias remain immutable. The estimator identifies four
payload parameters in the configured tool frame:

```text
theta = [m, h_x, h_y, h_z]
h = m * c
```

`m` is payload mass in kilograms, `c` is center of gravity in metres and `h`
is the first moment in kg m. Payload rotational inertia is needed by the moving
simulation plant but does not affect static gravity torque and is never
estimated.

Friction, stiction, contact, cable force, compliance, actuator gain, payload
rotational inertia and broad whole-arm inertial error are outside this model.
Large tool changes require a new batch session rather than unrestricted online
learning.

## Inputs and isolation

Estimator-visible inputs are limited to the prepared payload regressor,
measured/applied joint-effort residual, exact model generation and stable joint
order, observation freshness, settled-motion evidence, actuator saturation and
contact-free status.

The simulation harness separately owns payload mass, CoG, rotational inertia,
plant gravity and exact ABA state. No estimator constructor or update method
accepts these truth values. Truth is used only after estimation to calculate
qualification metrics.

For a quasi-static eligible observation:

```text
y = tau_measured - tau_nominal(q)
y = Y_g(q) * theta + epsilon
```

`PreparedGravity` compiles link CoG and joint geometry once. Its warm nominal
gravity, `n x 4` payload regressor and payload evaluation use caller-owned
storage and stable group order. The sign convention is checked against the
independent Featherstone gravity boundary and the one-joint pendulum golden.

## Preparation

Before collecting data:

1. Load and validate the immutable `ArmModel` and selected
   `KinematicGroup`.
2. Bind `PreparedKinematics`, `PreparedGravity`, their workspaces and the
   estimator to a non-zero model generation.
3. Validate gravity, mass and CoG bounds, QR rank threshold, condition limit,
   RLS forgetting factor, covariance, innovation, parameter-step and output
   bounds.
4. Confirm the exact joint and transmission order.
5. Enter a contact-free, motor-safe calibration procedure through the normal
   control-admission path. The checked-in host qualification does not authorize
   a physical procedure.

A model generation or order change invalidates the session. Missing selected
link inertia and unsupported mimic dynamics fail closed.

## Settled pose collection

Call `start_collection` explicitly. Each pose window is accepted only when:

- generation and group identity match;
- the observation is fresh and finite;
- motion is declared settled;
- the actuator is not saturated;
- contact-free status is true; and
- bounded storage remains available.

Skipped data increments a typed bounded counter. The implementation accepts at
most 24 calibration windows and stores no unbounded history. A production
caller should form deterministic trimmed or robust window means before calling
`observe_window`; the current synthetic qualification supplies exact settled
window aggregates.

Calibration poses must vary wrist and arm orientation enough to excite all
four columns. Repeating nearby poses can increase sample count without adding
rank and must not be treated as successful calibration.

## Batch solve and observability

`solve_batch` processes weighted scalar joint rows in stable pose/joint order
with incremental Givens QR. It does not invert normal equations. All four QR
diagonals must exceed the configured rank threshold, and their bounded
condition estimate must pass.

Back substitution creates a candidate. Physical projection then:

- clamps mass to a positive configured interval;
- decodes each CoG component as `h_i / m`;
- clamps CoG to configured tool-volume bounds; and
- re-encodes the first moments.

An unobservable or non-finite solve enters `Rejected`; it cannot replace the
previous active snapshot.

## Held-out validation and activation

Calibration and validation poses are disjoint. `validate_candidate` predicts
the effort residual on held-out estimator observations and requires:

```text
RMSE / max(abs(measured payload effort)) <= configured threshold
```

The checked-in threshold is 5%. A non-zero measured peak is mandatory.
`activate` succeeds only after validation and performs a complete snapshot
promotion at the caller-selected cycle boundary. Until then, the candidate is
never evaluated as active feed-forward.

## Projected recursive refinement

After activation, `update_rls` accepts one scalar joint row at a time. It uses
a forgetting factor near one and caller-owned 4x4 covariance. Every update is
gated by identity, freshness, settled motion, saturation, contact status,
excitation and a 2,048-update ceiling.

Innovation is rejected at a hard outlier bound and clipped at the normal
update bound. Each parameter delta is rate-limited, the candidate is physically
projected, covariance is symmetrized and bounded, and the active snapshot is
committed only after all checks pass. Recoverable gate failures enter
`Frozen`: evaluation continues from the last accepted snapshot while learning
stops. A subsequent eligible update resumes `Active`.

## Control composition

The active payload effort is:

```text
tau_payload = Y_g(q) * theta_active
tau_command = tau_feedback + tau_nominal + tau_payload
```

`direct_effort_proposal` and `mit_feed_forward_proposal` map stable-order joint
effort through the compiled power-consistent transmissions. The inverse mapping
used by the host plant is `joint_effort = actuator_torque * transmission_scale`.
These helpers return ordinary proposals; they cannot create admitted commands.

## Deterministic host qualification

The `rs-arm-simulation` app uses the checked-in synthetic 6-DOF model, a finite
simulator-only terminal payload, fixed `0.0002 s` steps, ABA forward dynamics
and semi-implicit Euler integration. It performs:

- 20 disjoint calibration pose windows;
- 8 held-out validation poses;
- prepared FK and numerical IK round-trip;
- matched nominal holding;
- mismatched nominal-only holding;
- mismatched adaptive holding; and
- bounded eligible RLS refinement.

The canonical report passes only when:

- held-out payload torque RMSE is at most 5% of peak truth torque;
- mismatch holding RMS improves by at least 80%;
- matched holding RMS degrades by at most 5%;
- mass error is at most 3%;
- CoG error is at most 10 mm;
- calibration, held-out and RLS budgets pass; and
- FK/IK round-trip error passes its fixed tolerance.

Run and verify with:

```bash
tools/run-arm-simulation.sh qualify
tools/run-arm-simulation.sh verify
tools/run-arm-simulation.sh test
```

The report hash excludes only its own hash field. Timing and allocation
measurements are deliberately excluded from deterministic evidence because
they depend on the host scheduler.

## Failure handling and rollback

- Stale, moving, saturated, contact or weak-excitation samples freeze or skip
  learning without changing the last accepted estimate.
- Non-finite values, dimension mismatches, invalid configuration and missing
  inertia fail closed.
- A rejected candidate never replaces the active snapshot.
- Detaching/changing a tool, changing the model generation or changing joint
  order requires estimator reset and a new batch calibration.
- Removing adaptive compensation means omitting its feed-forward contribution;
  nominal gravity, feedback and every admission gate remain unchanged.

The host evidence is necessary regression evidence, not authorization to reuse
the same bounds or gains on a physical robot. Physical deployment requires a
separate restrained-arm procedure, independent payload measurement, collision
review, stability analysis, actuator/thermal limits, watchdog validation and
hardware-specific sign verification.
