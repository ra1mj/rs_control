# Payload Identification and Adaptive Gravity Compensation Process

## 1. Purpose

This document defines the complete, deterministic process for identifying an
end-effector payload and using the result as bounded adaptive gravity
compensation.

The process is designed first for the project-native host simulation. It is not
a physical-arm calibration or stability qualification. The estimator produces
a feed-forward proposal only and cannot grant motion authority, bypass control
admission, alter leases/deadlines, or exceed actuator limits.

## 2. Scope

The first release identifies only the gravitational effect of a rigid tool
payload attached at the configured group tip:

`theta = [m, h_x, h_y, h_z]`

where:

- `m` is payload mass in kilograms;
- `h = m*c` is the first moment in kilogram-metres;
- `c = [c_x, c_y, c_z]` is payload center of gravity in the tip/tool frame.

Nominal robot-link inertial parameters remain fixed. Payload rotational inertia,
link inertial errors, friction, stiction, contact, cable forces, compliance, and
actuator gain identification are outside this estimator.

## 3. Success definition

The primary success criterion is compensation quality, not merely parameter
similarity. A parameter estimate can be physically close yet produce a poor
torque prediction if the calibration poses are weak, and some parameter errors
can partially cancel within a narrow pose set.

The qualification gates for this delivery are:

- payload-induced gravity-torque RMSE no greater than 5% of the peak true
  payload-gravity torque over held-out poses;
- at least 80% reduction in RMS holding error compared with nominal-only
  gravity compensation under the configured mismatched payload;
- no more than 5% holding-error degradation with a matched nominal plant;
- diagnostic payload mass error no greater than 3%;
- diagnostic CoG Euclidean error no greater than 10 mm;
- all safety, identity, determinism, allocation, and performance gates pass.

The torque and holding-error gates are release gates. Mass and CoG error are
diagnostic gates for the deterministic simulator, because physical controllers
generally cannot provide independent payload ground truth at runtime.

## 4. Required inputs and identities

An identification session is created only after the following immutable inputs
have been validated:

- arm profile ID and model generation;
- kinematic group ID, base link, tip link, and exact joint order;
- prepared nominal kinematics and gravity evaluator;
- gravity vector and frame convention;
- sample period and maximum observation age;
- payload mass and CoG bounds;
- joint velocity, acceleration, saturation, residual, and noise gates;
- batch sample/window limits and observability thresholds;
- RLS forgetting factor, covariance bounds, innovation limit, and rate limits;
- deterministic session seed/config identity for simulation evidence.

Any model generation or exact joint-order change invalidates the session. No
estimate or covariance can be carried across an identity change.

## 5. Estimator inputs versus validation truth

### Estimator-visible inputs

The estimator may use only:

- measured joint position in exact group order;
- measured joint velocity and acceleration, or bounded estimates used solely
  for eligibility gating;
- measured/applied joint effort in exact group order;
- observation timestamp, source/boot identity, sequence, quality, and fault
  state;
- actuator saturation/admission state;
- known nominal gravity model output;
- configured contact-free/calibration mode state.

### Simulation-only validation truth

The following values are available to the qualification harness but must never
enter the estimator:

- plant payload mass and CoG;
- plant gravity torque;
- injected sensor noise/outlier values before observation formation;
- exact plant acceleration or other hidden simulator state not exposed by the
  observation contract.

The test harness compares estimates and outputs with this truth only after an
estimator update/evaluation has completed.

## 6. Mathematical observation model

The payload gravity torque is linear in mass and first moments:

`tau_payload(q) = Y_g(q) * theta`

where `Y_g(q)` is an `n x 4` payload-gravity regressor for an `n`-joint group.

For an eligible quasi-static observation:

`y = tau_measured - tau_nominal_gravity(q)`

and:

`y = Y_g(q) * theta + epsilon`

`epsilon` contains measurement noise and residual effects not represented by
the payload gravity model.

The implementation defines the compensation sign from the existing
`ArmDynamics::gravity_compensation` convention and validates it against an
independent analytical fixture. The estimator must not infer or flip the sign
from residual trends at runtime.

## 7. Payload-gravity regressor construction

The prepared regressor must:

- use the existing project kinematic chain, frames, gravity vector, and joint
  order;
- perform no allocation after preparation;
- return finite `n x 4` values in caller-owned fixed-capacity storage;
- be generation-bound and fail on identity mismatch;
- be independently checked against payload gravity-torque differences across
  multiple poses.

The implementation uses the analytical strategy: derive the four columns from
the prepared tip pose/Jacobian and gravity wrench. This avoids physically
invalid zero-mass/first-moment basis models and keeps the warm path allocation
free. It must match an independent finite-difference or analytical oracle within
the existing dynamics precision. Regressor construction cannot use the
simulated plant model.

## 8. Identification state machine

The session uses explicit states:

1. `Unprepared`: no model-bound workspace exists.
2. `Ready`: workspace and immutable configuration are valid; no samples exist.
3. `Collecting`: eligible settled observations are accumulated.
4. `Solving`: the bounded batch problem is evaluated once enough information
   exists.
5. `Candidate`: a physically bounded estimate exists but is not active.
6. `Active`: the accepted estimate may contribute bounded feed-forward torque.
7. `Frozen`: evaluation continues from the last accepted estimate but online
   learning is temporarily disabled by eligibility or excitation gates.
8. `Rejected`: the current batch/candidate failed a validation gate; collection
   may restart under the same identity after an explicit reset.
9. `Faulted`: a non-recoverable configuration, identity, numerical, or bound
   violation requires full session reset/re-preparation.

State changes are deterministic and produce typed evidence. A skipped update is
not a fault and must record its reason.

## 9. Calibration pose and sample collection

### Pose selection

Calibration must cover sufficiently different arm orientations so that the
stacked four-parameter regressor has full numerical rank. Repeating many samples
at the same pose does not improve observability.

The simulator uses a fixed checked-in pose sequence with:

- positions inside joint limits and away from configured singular/boundary
  margins;
- varied tip orientations relative to gravity;
- no collision claim;
- deterministic order and bounded length;
- a separate held-out validation-pose set not used for fitting.

### Settling

At each pose, collection begins only after a fixed settle window passes all
eligibility gates. The position and effort used for one calibration row are
formed from a bounded fixed-size window using deterministic robust aggregation,
such as a median or trimmed mean. Window variance must remain below configured
limits.

### Per-observation eligibility gates

Reject or skip a sample when any of the following is true:

- timestamp is stale, in the future, duplicated, or non-monotonic;
- source, boot, profile, group, generation, or joint order differs;
- position, velocity, acceleration, effort, or regressor contains NaN/Inf;
- velocity or acceleration exceeds quasi-static thresholds;
- actuator effort is saturated, slew-limited, or rejected by admission;
- the observation reports a fault, protective stop, or contact condition;
- the residual exceeds the configured hard plausibility bound;
- settle-window variance is too high;
- the sample contributes insufficient new information.

Every rejection increments a bounded reason counter. Raw high-rate logging is
not performed inside the estimator hot path.

## 10. Batch initialization

### Stacked system

Accepted pose observations form a bounded stacked system:

`A * theta = b`

where each pose contributes selected joint rows from `Y_g(q)` and its measured
residual vector. Rows with structurally negligible regressor magnitude may be
excluded deterministically before the solve.

### Weighting

Weights may account for:

- per-joint effort-noise variance;
- settle-window variance;
- known effort-sensor scale differences;
- deterministic robust down-weighting of bounded residual outliers.

Weights must be finite, positive, bounded, and prepared without dynamic storage.

### Observability gate

Before solving, the weighted information matrix must satisfy all of:

- numerical rank equals four;
- minimum singular value exceeds a configured floor;
- condition number is below a configured maximum;
- each accepted pose/sample count is within configured bounds;
- no single pose or joint dominates the information beyond a configured ratio.

Failure returns `InsufficientExcitation` evidence and does not publish a
candidate.

### Solve

Use a deterministic weighted Givens-QR least-squares solve, optionally with a
small documented regularization term toward a configured prior:

`min ||W(A*theta-b)||^2 + rho*||L(theta-theta_prior)||^2`

Normal-equation inversion is not the primary implementation. Solver iteration,
matrix sizes, and workspace are bounded at preparation time.

### Physical projection

Decode the solution as mass and CoG:

1. require finite mass above the configured minimum;
2. clamp/reject mass outside the configured interval according to policy;
3. compute `c = h/m`;
4. project/reject CoG outside the configured tool-frame box/radius;
5. re-encode `h = m*c`;
6. record every active bound.

An estimate requiring excessive projection is rejected instead of silently
accepted.

## 11. Candidate validation and activation

A batch solution becomes `Candidate`, not immediately `Active`.

Evaluate it on the independent held-out pose set and require:

- finite predicted torques;
- torque RMSE and maximum error below configured limits;
- improvement over nominal-only prediction by the configured minimum ratio;
- no parameter/rate/output bound violation;
- stable residual sign and magnitude across poses, not improvement at only one
  pose;
- optional mass/CoG diagnostic accuracy in simulation.

Activation is a two-phase operation:

1. validate and seal candidate evidence;
2. atomically replace the active parameter snapshot at a control-cycle
   boundary.

The previous active snapshot remains available until replacement succeeds. A
rejected candidate cannot modify active compensation.

## 12. Online projected RLS refinement

After activation, eligible observations may refine the same four parameters.
Process the selected joint rows in stable joint order. For one scalar row
`phi` and residual observation `z`:

`innovation = z - phi^T * theta`

`gain = P*phi / (lambda + phi^T*P*phi)`

`theta_next = theta + gain*innovation`

`P_next = (P - gain*phi^T*P) / lambda`

where `lambda` is close to one and `P` is the bounded covariance/information
state.

Each update must:

- clip or reject excessive innovation;
- enforce a maximum parameter delta per observation and per second;
- symmetrize covariance and bound its eigenvalues/diagonal representation;
- project parameters through the mass/CoG physical bounds;
- reject non-finite or non-positive covariance state;
- retain the previous state on any failed check;
- contribute to a rolling excitation/information window.

Weak excitation, excessive motion, saturation, stale data, or contact moves the
session to `Frozen`. `Frozen` is reversible when gates recover. A generation or
numerical integrity failure moves it to `Faulted`.

## 13. Adaptive torque evaluation

For every eligible control evaluation:

1. verify active snapshot identity;
2. compute `tau_adaptive = Y_g(q) * theta_active`;
3. enforce finite values and configured per-joint adaptive torque limits;
4. apply configured output slew/rate limits;
5. add the result to nominal gravity feed-forward as a proposal;
6. pass the combined proposal through existing arm-control admission and final
   actuator limits.

Evaluation does not imply learning. The estimator may remain `Frozen` while the
last accepted parameters continue to be evaluated.

## 14. Reset and payload-change behavior

An explicit reset clears samples, candidate state, covariance, counters, and
active adaptive output. Reset is mandatory after model-generation or exact
joint-order changes.

A payload step change under the same model identity is handled by one of two
configured policies:

- start a new bounded calibration session; or
- allow gated RLS tracking only if innovation and rate limits can accommodate
  the change without repeated clipping.

The MVP qualification uses explicit new-session calibration for large payload
changes and tests RLS only for small/slow mismatch refinement.

## 15. Determinism and realtime constraints

- Fixed timestep, pose order, sample windows, row order, solver order, and
  seeded noise/outlier sequence are part of the evidence identity.
- Prepared FK, regressor evaluation, adaptive torque evaluation, and RLS update
  allocate nothing after warm-up.
- All storage has compile-time or validated preparation-time capacity.
- No estimator path reads the wall clock, logs to stdout/stderr, performs I/O,
  sleeps, or takes an unbounded lock.
- Batch solving may run outside the control hot path, but remains deterministic,
  bounded, and identity-bound.

## 16. Required output evidence

Each completed run emits a bounded structured report containing:

- report schema version;
- simulator/config/seed/model-generation/profile/group/joint-order identity;
- nominal and plant payload truth for simulation evidence, clearly marked as
  unavailable to the estimator;
- all estimator thresholds and bounds;
- attempted, accepted, and rejected sample counts by reason;
- calibration and validation pose IDs;
- singular values, numerical rank, condition number, and information metrics;
- unprojected candidate, projected candidate, active bounds, and final estimate;
- initial/final covariance or confidence metrics;
- nominal and adaptive torque RMSE/max error on fit and held-out sets;
- nominal and adaptive RMS/max holding error;
- convergence sample/time index;
- RLS accepted/skipped/rejected update counts;
- saturation, clipping, freeze, reset, reject, and fault events;
- allocation and Release P95 latency evidence in a separate host-performance
  attachment (timing is not part of the byte-stable functional report);
- deterministic report hash or byte-equality comparison result.

The deterministic functional report is canonicalized and hashed without the
self-referential hash field. Host timing/allocation measurements are emitted by
single-threaded Release tests as a separate attachment, because wall-clock
latency is not byte-deterministic. The CLI/observer layer may render either
artifact and live state, but rendering is not part of the estimator and cannot
affect its state.

## 17. Qualification test matrix

### Mathematical correctness

- 1-DoF analytical pendulum with a known point payload.
- Multiple 6-DoF poses comparing regressor prediction with independent plant
  payload-gravity differences.
- RNEA/ABA round-trip and nominal gravity golden tests remain unchanged.
- Coordinate-frame and gravity-sign variants.

### Batch identification

- noiseless full-rank pose set recovers mass/CoG and torque within thresholds;
- deterministic sensor noise remains within release/diagnostic thresholds;
- bounded outliers are down-weighted/rejected without corrupting the result;
- repeated/degenerate poses fail `InsufficientExcitation`;
- held-out validation catches overfit or wrong-frame candidates;
- unphysical or excessive-projection solutions are rejected.

### Online adaptation

- small payload mismatch converges within a fixed accepted-update budget;
- matched model does not drift or materially degrade holding error;
- weak excitation freezes learning;
- stale, saturated, fast-motion, faulted, contact, non-finite, and excessive
  innovation samples do not update state;
- covariance and parameter bounds hold under long deterministic runs;
- large payload change requires explicit recalibration according to MVP policy.

### Closed-loop simulation

- nominal-only and adaptive runs start from byte-identical initial state and
  command sequence;
- static holds cover several poses and gravity leverage conditions;
- trajectory motion remains finite and admitted under nominal and mismatched
  payloads;
- adaptive compensation meets torque and holding-error improvement gates;
- output limits and slew prevent torque discontinuities at activation/reset.

### Failure and identity handling

- invalid timestep, gravity, payload bounds, estimator config, and capacity fail
  before readiness;
- missing link inertia preserves `DynamicsUnavailable`;
- profile/group/generation/order/source/boot mismatches fail closed;
- reset clears adaptive output and cannot reuse stale covariance;
- no candidate or failed solve can alter active compensation.

### Performance and allocation

- zero warm allocations for prepared FK, regressor, nominal/adaptive gravity,
  and RLS update/evaluation;
- Release P95 latency gates for each prepared surface;
- existing analytical/cached/numerical IK performance gates continue to pass;
- warnings-denied Clippy and `tools/ci.sh` pass.

## 18. Completion rule

The feature is accepted only when the checked-in deterministic report proves
all primary accuracy, safety, identity, determinism, allocation, and performance
gates. Passing parameter-error diagnostics alone is insufficient. No result is
presented as physical-arm accuracy or stability qualification.
