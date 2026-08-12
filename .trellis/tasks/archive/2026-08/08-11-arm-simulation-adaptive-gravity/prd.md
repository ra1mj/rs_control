# Arm Simulation and Adaptive Gravity Compensation

## Goal

Create a deterministic host-side articulated-arm simulation that exercises the
existing project-native model, FK/IK, dynamics, trajectory and control
admission surfaces, then add bounded adaptive gravity compensation that reduces
steady holding error when the actual arm/load differs from the nominal URDF
inertial model.

The simulator and estimator are qualification tools only. They grant no motion
authority, perform no hardware I/O and make no physical-arm stability claim.

## Background

- `arm-algorithms` already provides validated FK, prepared analytical/numerical
  IK, RNEA inverse dynamics, ABA forward dynamics, CRBA and nominal gravity
  compensation.
- The existing Release IK gate passes on this host with P95 values of 5.6 us
  analytical, 0.44 us cached and 2.0 us numerical, all below current limits.
- `arm-control::SimExecutor` currently records admitted actuator setpoints but
  does not integrate articulated dynamics, model sensors, payload mismatch or
  closed-loop tracking.
- Existing gravity compensation is accurate for the nominal pendulum golden
  fixture but allocates temporary vectors per call; it is not yet a prepared
  control-loop surface.
- Missing inertial data already fails closed as `DynamicsUnavailable`; the new
  simulation and adaptive path must preserve that contract.
- Literature and public industrial interfaces converge on tool-payload
  identification as the practical gravity-mismatch scope. ABB RobotWare 8,
  Universal Robots, FAIRINO and a public AUBO SDK workflow expose payload mass
  and center of gravity as identified/configured model data.
- Payload gravity torque is linear in four physical parameters: mass and the
  three first moments. Payload rotational inertia does not affect static
  gravity torque, and classic experimental work reports mass and center of
  mass as easier to identify accurately than inertia.
- The 2021 BIRDy survey recommends IDIM-IV/DIDIM for robust offline whole-arm
  identification, but notes that recursive OLS/WLS are simpler choices for
  online rough inertial estimation. Kalman methods require sensitive tuning;
  ordinary TLS is seldom used and did not outperform adequately filtered
  OLS/WLS in the cited comparisons.

## Product decisions

- Keep the validated nominal link inertial model fixed.
- Make the MVP mismatch class a changing tool payload mass and center of
  gravity, represented as `[m, m*c_x, m*c_y, m*c_z]`.
- Initialize from bounded weighted batch least squares across sufficiently
  diverse eligible poses, then allow slow projected RLS refinement only when
  freshness, motion, saturation, residual and excitation gates pass.
- Keep friction, contact and broad whole-arm model error outside this estimator.
- See `research/adaptive-gravity-methods.md` for the comparison, industrial
  evidence and primary citations.
- This scope and the quantitative release gates below were approved by the user
  on 2026-08-11.

## Requirements

- SIM-01: Add a deterministic fixed-step host simulation for one validated
  `ArmModel`/`KinematicGroup`, with explicit timestep, initial state, gravity,
  actuator effort input and bounded run length.
- SIM-02: Integrate joint position and velocity through the existing articulated
  dynamics boundary and expose coherent simulated position, velocity and effort
  observations in stable joint order.
- SIM-03: Support a nominal model and a deliberately different plant payload or
  inertial condition so compensation robustness can be measured without
  changing production robot profiles.
- SIM-04: Exercise prepared FK, existing prepared IK, nominal gravity
  compensation and admitted torque/MIT control paths without hardware or ROS.
- AGC-01: Add an online adaptive gravity correction with explicit preparation,
  estimator state, update gates, parameter bounds, rate limits and reset/model
  generation identity.
- AGC-02: Learning must be disabled for stale/faulted observations, excessive
  velocity/acceleration, saturation, non-finite residuals or identity mismatch.
- AGC-03: The adaptive output is a bounded feed-forward proposal only. It never
  grants authority, changes a lease/deadline or bypasses the existing final
  arm-control admission and actuator limits.
- AGC-04: The estimator identifies only payload mass and first moments. It
  starts from a rank/conditioning-checked bounded batch fit and uses projected
  RLS for optional online refinement.
- AGC-05: Estimator covariance, innovation, parameter delta, mass, center of
  gravity and output torque are bounded. Weak excitation freezes learning but
  does not invalidate evaluation from the last accepted estimate.
- PERF-01: Warm prepared FK, nominal/adaptive gravity evaluation and estimator
  update paths allocate nothing. Release performance gates report P95 latency;
  existing IK performance limits remain enforced.
- TEST-01: Golden nominal-model tests retain the current gravity result, while
  mismatch simulations compare nominal-only and adaptive compensation using
  deterministic error metrics.
- DOC-01: Add a complete identification-process document covering estimator
  inputs versus simulation-only truth, parameterization, regressor contract,
  calibration state machine, sample eligibility, observability, batch solve,
  candidate activation, projected RLS, resets, evidence and the qualification
  test matrix. The approved process is recorded in
  `research/payload-identification-process.md`.
- OPS-01: Add a repository-root-safe `tools/run-arm-simulation.sh` wrapper with
  bounded `qualify`, `verify` and `test` commands. It must be host-only, refuse
  hardware/ROS arguments, use locked Cargo commands and make no implicit
  destructive changes. `tools/ci.sh` must invoke its read-only `verify` path.
- DOC-02: Update the root README with copyable build/run/verify commands for
  the simulation and explicit warnings that its evidence is host-only and does
  not qualify physical-arm accuracy, stability or motor output.

## Acceptance Criteria

- [x] A checked-in deterministic simulation holds and moves the synthetic 6-DOF
      arm using existing semantic types, with repeatable byte-equal evidence for
      the same model, seed/config and command sequence.
- [x] The checked-in calibration fixture uses no more than 24 accepted settled
      pose windows, reserves at least 8 separate held-out poses, and any online
      refinement converges within 2,048 accepted RLS updates.
- [x] Across all held-out pose/joint rows, payload-gravity torque RMSE is at most
      5% of the maximum absolute simulation-truth payload-gravity torque in that
      held-out set. Simulation diagnostics also show payload-mass error at most
      3% and CoG Euclidean error at most 10 mm.
- [x] In the configured mismatched-payload hold scenario, adaptive compensation
      reduces joint-position RMS holding error by at least 80% relative to the
      byte-identical nominal-only run.
- [x] In the matched nominal-plant scenario, enabling the adaptive path increases
      RMS holding error by no more than 5% relative to the nominal-only run. The
      ratio normalizer is `max(nominal_rms, 1e-6)` in joint SI units.
- [x] Invalid timestep, missing inertia, wrong joint order/generation, stale or
      non-finite observations and estimator-bound violations fail closed.
- [x] Allocation tests prove zero warm allocations for prepared FK, gravity
      compensation and adaptation update/evaluation.
- [x] On the same isolated Release host gate used by the current IK benchmark,
      prepared FK, nominal gravity, payload-regressor evaluation, adaptive
      gravity evaluation and one eligible RLS update each have P95 latency no
      greater than 50 us. Existing IK limits remain 50 us for analytical/cache
      solves and 500 us for numerical solves.
- [x] Warnings-denied Clippy, architecture checking and `tools/ci.sh` pass.
- [x] `bash -n tools/run-arm-simulation.sh` passes, its `verify` command matches
      the checked-in deterministic report, and the README commands are kept in
      sync with the wrapper's actual subcommands.
- [x] The complete identification-process document is consistent with the
      implemented state machine, configuration, structured evidence and test
      names; no estimator test consumes simulation-only plant truth.

## Out of scope

- Physical motor actuation or physical-arm accuracy/stability qualification.
- Gazebo/Isaac/MuJoCo integration, ROS visualization and GPU simulation in the
  MVP; the first simulator is project-native and deterministic.
- Full-link or payload rotational-inertia identification, friction/stiction,
  contact/cable-force estimation and actuator-gain identification.
- Online neural-network training or an estimator that can directly command
  actuators without the normal admission path.
