# Arm Simulation and Adaptive Gravity Compensation — Design

## 1. Purpose and safety boundary

This delivery adds a deterministic, motor-disabled articulated-arm simulator
and a bounded payload-gravity estimator. The estimator improves a feed-forward
proposal; it does not become a controller, grant authority, change a lease or
deadline, bypass collision typestate, or write a hardware frame.

The existing nominal link model remains immutable. The only adaptive state is
the four-parameter payload vector:

`theta = [m, h_x, h_y, h_z]`, where `h = m*c`.

The project-native simulator is a qualification composition root. Its results
qualify deterministic host behavior only and make no physical-arm accuracy or
stability claim.

## 2. Architecture and data flow

```text
checked-in model + simulator-only payload truth
              |                    |
              |                    +--> ArmDynamics plant backend (ABA)
              |                              |
              v                              v
 PreparedKinematics / PreparedGravity    q, qd, applied effort
              |                              |
              +--> nominal gravity ----------+--> ArmJointObservation
              |                                      |
              +--> payload regressor ----------------+--> estimator
                                                         batch QR -> candidate
                                                         held-out validation
                                                         projected gated RLS
                                                                  |
                                                                  v
                                                     bounded adaptive effort
                                                                  |
 target + PD + nominal effort + adaptive effort                    |
              |                                                   |
              +------------------- arm-control proposal <----------+
                                      |
                       lease/lifecycle/freshness/limit/slew admission
                                      |
                                SimExecutor record
                                      |
                          simulated MIT actuator response
                                      |
                              next fixed plant step
```

Simulation truth and estimator input use different Rust types and are owned by
different components. No estimator constructor or update method accepts plant
payload truth, exact plant gravity torque, injected pre-observation noise, or
hidden plant acceleration.

## 3. Repository boundaries and file ownership

| Boundary | Planned files | Responsibility |
| --- | --- | --- |
| Workspace | `Cargo.toml`, `Cargo.lock`, `architecture.toml` | Register the motor-disabled `rs-arm-simulation` app and any already-locked serialization dependency. |
| Fast kinematics | `crates/application_module/arm-algorithms/src/kinematics/ik/prepared.rs`, `src/kinematics/ik.rs` | Add a public allocation-free prepared FK evaluation using the existing generation-bound `IkWorkspace`; retain every current prepared IK API and latency gate. |
| Prepared gravity and payload plant model | `crates/application_module/arm-algorithms/src/dynamics/mod.rs`, new `src/dynamics/gravity.rs`, new `src/dynamics/payload.rs` | Compile nominal link CoM geometry, evaluate nominal gravity and the four-column payload regressor without warm allocations, and optionally append a validated simulator-only payload inertia to the existing Featherstone plant backend. |
| Adaptive estimator | new `crates/application_module/arm-algorithms/src/adaptive_gravity/` module | Own configuration, identity, collection window, weighted batch QR, observability checks, physical projection, candidate validation, gated projected RLS, state transitions and bounded evidence. |
| Feed-forward proposal composition | new `crates/application_module/arm-control/src/feed_forward.rs`, `src/lib.rs` | Convert stable-order joint effort and joint targets through `RotaryTransmission` into direct-torque/MIT proposals. Admission remains the sole owner of authority, freshness, magnitude and slew checks. |
| Transmission inverse mapping | `crates/application_module/arm-model/src/transmission.rs` | Add the power-consistent actuator-torque-to-joint-effort inverse needed by the simulated actuator. |
| Simulation composition root | new `apps/rs-arm-simulation/` | Own fixed-step integration, deterministic sensor formation, synthetic authority, calibration/hold scenarios, qualification metrics, report rendering and CLI. It has no hardware dependency or physical executor implementation. |
| Operator scripts and README | new `tools/run-arm-simulation.sh`, `tools/ci.sh`, `README.md` | Provide a repository-root-safe host-only wrapper, deterministic report verification in CI, and copyable usage/safety documentation. |
| Tests and evidence | arm algorithm/control tests, `apps/rs-arm-simulation/tests/`, `evidence/arm-adaptive-gravity-simulation.json` | Mathematical, failure, isolation, determinism, accuracy, allocation and Release latency gates. |
| User documentation | `docs/arm-payload-identification.md` | Publish the complete identification process from the reviewed task research, updated to exact implemented type, state and test names. |

`arm-control::SimExecutor` keeps its current semantics and public API: it records
admitted setpoints and never becomes a dynamics engine. The new composition root
feeds those recorded setpoints into its own articulated plant. This preserves
existing callers and proves that adaptive output traverses the real admission
boundary.

## 4. Prepared kinematics and gravity

### 4.1 Prepared FK

`PreparedKinematics::forward(...)` will copy tagged positions into the existing
fixed `[f64; MAX_JOINTS]` scratch, call the current prepared frame evaluator, and
return an `ArmPose`. It reuses `IkWorkspace`, so profile, group, generation,
dimension, unit and joint-limit checks are identical to prepared IK. No new
solver or cache is introduced.

### 4.2 Prepared nominal gravity

`PreparedGravity` is compiled once from a `KinematicChain`, non-zero model
generation and finite gravity vector. Preparation rejects missing selected-link
inertia and mimic-joint dynamics with the existing `DynamicsUnavailable`
contract. It stores only immutable path transforms, joint bindings, link masses
and link-frame CoGs.

`GravityWorkspace` preallocates path transforms, active joint frames and fixed
joint/output arrays. One evaluation walks base-to-tip once, then accumulates the
gravity effort of each link CoG into every upstream active joint:

- revolute/continuous: the joint axis dotted with the CoG moment;
- prismatic: the joint axis dotted with the gravity force.

The sign is fixed by the existing `ArmDynamics::gravity_compensation` pendulum
golden test. It is never inferred from residual behavior.

The current Featherstone-backed `ArmDynamics` remains the independent f32
reference and the general RNEA/ABA/CRBA boundary. Its public methods are not
silently redefined. The prepared f64 geometric evaluator is the control-loop
surface because the upstream `gravity_compensation`/RNEA implementation clones
and allocates internal dynamic vectors on every call.

### 4.3 Payload regressor

The same prepared frame pass produces a row-major bounded `n x 4` regressor.
For each joint row, the mass column is the tip-origin gravity term and each
first-moment column is the moment induced by one tool-frame unit axis rotated
into the base frame. Evaluation returns finite values in stable group order and
uses no plant payload state.

The analytical regressor is checked independently against the gravity-torque
difference between two Featherstone plant models, one without and one with a
validated attached payload, over the pendulum and multiple 6-DOF poses.

### 4.4 Simulator-only attached payload

`AttachedPayloadInertia` validates positive finite mass, finite tool-frame CoG
and a physically valid positive-definite inertia tensor about the payload CoG.
`ArmDynamics::new_with_payload(...)` appends it as a fixed terminal body after
the selected group tip. `ArmDynamics::new(...)` delegates to the no-payload
case and preserves exact behavior.

Rotational inertia is plant-only data required for moving forward dynamics. It
is never included in the estimator parameter vector and does not contribute to
the static payload-gravity regressor.

## 5. Estimator contract

### 5.1 Immutable configuration and identities

Preparation binds profile ID, group ID, generation, exact joint order, source
ID, boot ID, gravity, timestep/freshness limits, mass/CoG/output bounds,
collection capacities and numerical thresholds. Zero or non-finite bounds,
incompatible dimensions, missing inertia and invalid covariance settings fail
before `Ready`.

An `AdaptiveSampleContext` explicitly carries caller monotonic time, model
generation, contact-free status and admission/saturation/slew evidence. Joint
acceleration used for gating is a bounded finite difference of consecutive
estimator-visible velocities and timestamps. Hidden simulator acceleration is
not accepted by the API.

### 5.2 State machine

The public states are:

`Unprepared -> Ready -> Collecting -> Solving -> Candidate -> Active`

`Frozen` temporarily stops RLS updates while retaining the last active
snapshot. `Rejected` requires an explicit collection reset. `Faulted` requires
full reset/re-preparation. Every transition and skipped update has a bounded
typed reason counter; no hot-path logging or I/O occurs.

### 5.3 Settled-window collection

Each configured pose has a fixed-capacity window. Freshness, identity, quality,
finite-value, quasi-static velocity/derived-acceleration, saturation, contact,
residual and variance gates run before a pose sample is accepted. The final
pose row uses a deterministic trimmed mean. Repeated poses that do not improve
the information floor are skipped with `InsufficientExcitation` evidence.

The checked-in fixture admits at most 24 calibration pose windows and reserves
at least 8 different poses exclusively for validation.

### 5.4 Weighted batch solve

Accepted joint rows are processed in pose then stable-joint order by a bounded
Givens/Householder QR update. Weights are positive finite values derived from
configured joint effort noise and settled-window variance. Optional prior
regularization is represented as additional QR rows; the primary solve never
inverts normal equations.

A fixed 4x4 SVD/eigendecomposition is used only to report singular values and
enforce rank, minimum singular value, condition number and dominance gates.
Back substitution yields the unprojected candidate. All matrices and pose rows
live in preparation-time/fixed-capacity storage.

### 5.5 Physical projection and activation

Every candidate is decoded as mass and CoG, checked against finite minimum mass,
projected into configured mass/CoG bounds, re-encoded as first moments, and
rejected if the required projection exceeds its configured maximum. Parameter,
covariance and predicted output bounds are checked before publication.

Candidate validation consumes only held-out observed position/effort windows.
It requires finite predictions, bounded residuals and configured improvement
over nominal-only prediction. Activation is a two-phase snapshot swap at a
caller-declared control-cycle boundary; a failed candidate cannot change the
previous active snapshot.

### 5.6 Projected gated RLS

After activation, eligible scalar joint rows update the 4x4 covariance in
stable joint order. The forgetting factor is close to one. Innovation clipping,
per-update/per-second parameter rate limits, covariance symmetrization and
eigenvalue bounds, physical projection and adaptive torque limits are applied
before commit. A failed check retains the previous parameter and covariance
state.

Weak excitation, stale data, excessive motion, saturation, contact or a
recoverable outlier enters `Frozen`; recovery resumes from the same accepted
snapshot. Identity or numerical-integrity failures enter `Faulted`. The
qualification small-mismatch case must converge within 2,048 accepted updates;
large payload changes require a new explicit batch session.

## 6. Feed-forward and admission path

The estimator evaluates only `tau_payload = Y_g(q) * theta_active`. The caller
applies adaptive per-joint magnitude and slew bounds, then adds it to the
prepared nominal gravity and feedback effort.

`arm-control` provides additive proposal builders that:

1. require exact config profile/group/generation and joint order;
2. map joint positions, velocities and efforts through the existing
   power-consistent transmissions;
3. use the config's immutable MIT gains; and
4. return an ordinary `ArmControlProposal`.

The proposal still needs an existing `CommandEnvelope`, lease cache, running
lifecycle, executing action and fresh observation. `ArmControlAdmission` then
enforces magnitude and slew limits exactly as it does today. No adaptive type
can construct `AdmittedArmCommand` or `CollisionChecked` evidence.

## 7. Fixed-step simulation

`rs-arm-simulation` is a new application composition root with no hardware or
ROS dependencies. Its plant owns an `ArmDynamics` instance, tagged `q/qd/qdd`,
deterministic monotonic sequence, model/transmission identity and a bounded
sensor-noise generator.

For each step:

1. `ArmControlAdmission` validates an MIT or direct-torque intent.
2. `SimExecutor` records the admitted actuator setpoints.
3. The simulated actuator evaluates MIT PD plus feed-forward torque, applies
   its configured effort bound, records saturation, and maps power-consistently
   back to joint effort.
4. Payload-augmented ABA computes acceleration.
5. Semi-implicit Euler updates velocity then position at the explicit fixed
   timestep; non-finite values or a joint-limit crossing fail the run.
6. A new coherent `ArmJointObservation` is formed from post-step position,
   velocity and applied effort with deterministic bounded noise.

The simulator has an explicit maximum step count and never reads wall time,
sleeps, opens a device, emits a hardware frame or implements
`PhysicalArmExecutor`.

The qualification runner performs matched-model, mismatched nominal-only and
mismatched adaptive runs from byte-identical initial states and command
schedules. It also exercises prepared FK, the existing prepared IK solver, a
moving admitted trajectory and multiple static holds. Calibration and held-out
poses are checked in, deterministic, within limits and explicitly carry no
collision claim.

## 8. Accuracy metrics and release gates

For held-out pose/joint rows `i`, let `t_i` be simulator-truth payload gravity
and `p_i` be estimator prediction:

- `torque_rmse = sqrt(sum((p_i - t_i)^2) / N)`;
- `peak_payload_torque = max(abs(t_i))`, which must exceed a declared non-zero
  fixture floor;
- release requires `torque_rmse / peak_payload_torque <= 0.05`;
- diagnostic mass error is `abs(m_hat - m_true) / m_true <= 0.03`;
- diagnostic CoG error is `norm(c_hat - c_true) <= 0.010 m`.

Holding RMS is calculated over every held sample and joint in joint SI units.
The nominal-only and adaptive comparisons use identical plant/config/seed,
initial state, targets and step counts:

- mismatch reduction:
  `(nominal_rms - adaptive_rms) / max(nominal_rms, 1e-6) >= 0.80`;
- matched degradation:
  `max(0, adaptive_rms - nominal_rms) / max(nominal_rms, 1e-6) <= 0.05`.

The estimator never receives any value used only on the `t_i`, mass/CoG or true
holding-error side of these calculations.

## 9. Evidence and observability

The app emits canonical JSON with stable field order and a schema version. A
SHA-256 wrapper hashes the deterministic functional report excluding the hash
field itself. Two identical in-process runs must produce byte-equal canonical
bytes, and `--verify` compares a fresh run with the checked-in evidence file.

The deterministic report contains identities, complete configuration, hidden
truth clearly marked as oracle-only, state/counter evidence, observability,
candidate/projection/covariance values, fit/held-out metrics, holding metrics,
convergence budgets and all functional pass/fail gates.

Allocation counts and measured P95 latency are host/toolchain evidence and are
therefore deliberately separate from the deterministic report hash. Ignored
single-threaded Release tests print them and enforce these limits:

- prepared FK: P95 <= 50 us;
- prepared nominal gravity: P95 <= 50 us;
- payload regressor/adaptive evaluation: P95 <= 50 us;
- one eligible RLS update: P95 <= 50 us;
- existing IK: analytical/cache <= 50 us, numerical <= 500 us.

Allocation-counter tests require exactly zero warm allocations for prepared FK,
nominal gravity, regressor, adaptive evaluation and RLS update. Rendering,
batch report assembly and offline plant qualification are not realtime paths.

## 10. Operator script and README contract

`tools/run-arm-simulation.sh` resolves the repository root from its own location,
uses `set -eu`, and exposes only these bounded commands:

- `qualify`: run the locked Release qualification and optionally write the
  canonical report to an explicitly supplied path;
- `verify`: read-only comparison against the checked-in report;
- `test`: run the app's unit/integration qualification tests.

The wrapper does not discover devices, source platform environment files, open
ROS/CAN interfaces, or accept a hardware target flag. Unknown commands fail
before Cargo is invoked. `tools/ci.sh` calls only `verify`, so CI cannot rewrite
evidence. README examples use the wrapper rather than duplicating raw Cargo
argument details and state the host-only/non-physical qualification boundary.

## 11. Compatibility, dependencies and rollback

- No protocol, Protobuf, ROS or robot-profile schema changes are required.
- `ArmDynamics::new`, all existing FK/IK/control admission APIs and
  `SimExecutor` retain their behavior. New APIs are additive.
- The new app is classified as an `app` composition root. It may depend on
  application/protocol crates but never on a hardware crate.
- The simulator may use `serde`/canonical `serde_json`, `sha2`, `clap` and the
  existing deterministic RNG dependencies outside hot paths. Estimator and
  prepared evaluation code do no I/O and use fixed-capacity storage.
- Removing the new app, prepared/adaptive modules and additive proposal helpers
  restores the prior system without data migration. The checked-in nominal
  model and existing admission behavior require no rollback step.

The highest-risk compatibility points are the prepared IK file and existing
dynamics constructor. Changes there stay additive: FK is a wrapper over the
existing evaluator, and `ArmDynamics::new` delegates to the exact no-payload
constructor path. Golden and Release IK tests guard both rollback points.

## 12. Explicitly rejected alternatives

- Full-link or full payload-inertia identification: unnecessary for static
  gravity compensation and less observable.
- EKF/UKF, RTLS/IMU or neural residual learning: more tuning, sensors and
  qualification surface than the four-parameter problem requires.
- Controller-wide Slotine-Li adaptation: changes the feedback/stability boundary
  instead of improving a bounded feed-forward proposal.
- Replacing `SimExecutor` with a dynamics engine: breaks its established record
  semantics and couples admission tests to plant behavior.
- Making latency part of the byte-stable report: host timing is intentionally
  nondeterministic and would invalidate the evidence contract.
