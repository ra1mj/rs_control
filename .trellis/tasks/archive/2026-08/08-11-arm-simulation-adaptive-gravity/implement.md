# Arm Simulation and Adaptive Gravity Compensation — Implementation Plan

## Delivery order

### 0. Re-enter development with a clean planning gate

- After the user approves `prd.md`, `design.md` and this file, run
  `python3 ./.trellis/scripts/task.py start .trellis/tasks/08-11-arm-simulation-adaptive-gravity`
  using the Trellis development workflow.
- Re-read the target package specs with `trellis-before-dev`, capture the current
  dirty worktree, and do not alter unrelated existing changes.
- Confirm the workspace is still on the planning task and that no new protocol,
  hardware or ROS requirement has appeared.

Rollback point: before this step no production source or workspace manifest is
changed.

### 1. Add the simulation composition root and bounded report shell

Files owned:

- `Cargo.toml`, `Cargo.lock`, `architecture.toml`;
- `apps/rs-arm-simulation/Cargo.toml`;
- `apps/rs-arm-simulation/src/main.rs`, `src/lib.rs`;
- `apps/rs-arm-simulation/tests/qualification.rs`;
- new `tools/run-arm-simulation.sh`;
- `README.md`.

Tasks:

- Register `rs-arm-simulation` as an `app` composition root with no hardware or
  ROS dependency.
- Add a library API so tests can run two qualifications in-process without
  stdout, wall-clock reads or filesystem access.
- Define the report schema, canonical serialization, deterministic functional
  hash and `qualify --verify <path>` CLI command. Keep timing/allocation samples
  outside the deterministic report.
- Add the repository-root-safe `qualify`, read-only `verify` and bounded `test`
  wrapper commands; reject unknown commands before invoking Cargo.
- Update README with the exact wrapper commands, report location and host-only
  safety boundary.
- Add the fixed synthetic 6-DOF profile, stable joint order, seed, timestep,
  calibration pose IDs, held-out pose IDs, target schedule and explicit bounds
  as checked-in configuration values.

Validation:

```sh
cargo fmt --all -- --check
cargo test --locked -p rs-arm-simulation --lib
cargo test --locked -p rs-arm-simulation --test qualification
bash -n tools/run-arm-simulation.sh
```

Rollback point: remove only the new app and its workspace/architecture entries;
all existing crates must compile unchanged.

### 2. Add allocation-free prepared FK and analytical gravity geometry

Files owned:

- `crates/application_module/arm-algorithms/src/kinematics/ik.rs`;
- `crates/application_module/arm-algorithms/src/kinematics/ik/prepared.rs`;
- `crates/application_module/arm-algorithms/src/dynamics/mod.rs`;
- new `crates/application_module/arm-algorithms/src/dynamics/gravity.rs`;
- `crates/application_module/arm-algorithms/tests/kinematics.rs`;
- `crates/application_module/arm-algorithms/tests/dynamics.rs`;
- new `crates/application_module/arm-algorithms/tests/gravity_prepared.rs`;
- new `crates/application_module/arm-algorithms/tests/gravity_performance.rs`.

Tasks:

- Add `PreparedKinematics::forward` as a thin public wrapper over the existing
  generation-bound evaluator and prove order/limit/generation failures.
- Implement `PreparedGravity` and `GravityWorkspace` with fixed-capacity or
  preparation-time capacity only. Compile every selected link's CoM and active
  joint frame; reject missing inertia/mimic dynamics as before.
- Implement nominal gravity and `n x 4` payload regressor evaluation with a
  fixed sign convention and caller-owned output storage.
- Replace only the hot gravity surface used by the new estimator/simulator with
  the prepared evaluator. Keep the current Featherstone method as a reference
  and preserve its public result.
- Add the analytical pendulum and multi-pose Featherstone-difference oracle.

Validation:

```sh
cargo test --locked -p arm-algorithms --test dynamics
cargo test --locked -p arm-algorithms --test gravity_prepared
cargo test --locked -p arm-algorithms --test kinematics
```

Risk: `prepared.rs` is shared by every IK kernel. Keep changes additive and
run the full existing kinematics suite before touching estimator code.

### 3. Add simulator-only payload augmentation to the dynamics boundary

Files owned:

- `crates/application_module/arm-algorithms/src/dynamics/mod.rs`;
- new `crates/application_module/arm-algorithms/src/dynamics/payload.rs`;
- `crates/application_module/arm-algorithms/tests/dynamics.rs`;
- new payload cases in `apps/rs-arm-simulation/src/plant.rs`.

Tasks:

- Validate mass, tool-frame CoG and a positive-definite plant-only inertia
  tensor in `AttachedPayloadInertia`.
- Add `ArmDynamics::new_with_payload` by appending one fixed terminal body;
  make `ArmDynamics::new` delegate to the no-payload path.
- Preserve profile/group/order identity and `DynamicsUnavailable` behavior.
- Add tests showing payload gravity differences agree with the prepared
  regressor and that missing nominal inertia still fails closed.

Validation:

```sh
cargo test --locked -p arm-algorithms --test dynamics
cargo test --locked -p arm-algorithms --test gravity_prepared
```

Rollback point: if backend payload insertion changes any nominal golden or
RNEA/ABA round trip, remove only the new constructor/body append and retain the
prepared analytical gravity path; no caller needs a migration.

### 4. Implement bounded batch identification and candidate activation

Files owned:

- new `crates/application_module/arm-algorithms/src/adaptive_gravity/mod.rs`;
- new `.../batch.rs`, `.../rls.rs`, `.../evidence.rs`;
- `crates/application_module/arm-algorithms/src/lib.rs`;
- new `crates/application_module/arm-algorithms/tests/adaptive_gravity.rs`.

Tasks:

- Define validated config, model/order/source/boot identity, parameter bounds,
  fixed-capacity pose windows, observation context and typed state/reason enums.
- Form `tau_measured - tau_nominal` strictly from estimator-visible samples.
- Add deterministic settled-window aggregation and per-reason counters.
- Implement weighted QR batch solve, fixed 4x4 rank/conditioning checks,
  bounded back substitution, physical mass/CoG projection and held-out
  candidate validation.
- Implement atomic active-snapshot activation and reset behavior.
- Implement projected gated covariance RLS with innovation clipping, rate,
  covariance and excitation bounds. Evaluation remains available while frozen.
- Expose only bounded array/snapshot views; no logging, wall clock, dynamic
  storage growth or simulator truth access.

Validation:

```sh
cargo test --locked -p arm-algorithms --test adaptive_gravity
cargo test --locked -p arm-algorithms --test gravity_prepared
cargo test --locked -p arm-algorithms --lib
```

Risk: numerical conditioning and projection are the main correctness risks.
Keep a failing candidate separate from the active snapshot and add explicit
degenerate-pose, outlier, non-finite, stale, saturation and identity tests
before integrating the simulator.

### 5. Add feed-forward proposal composition without changing admission

Files owned:

- `crates/application_module/arm-model/src/transmission.rs`;
- new `crates/application_module/arm-control/src/feed_forward.rs`;
- `crates/application_module/arm-control/src/lib.rs`;
- `crates/application_module/arm-control/tests/control.rs`.

Tasks:

- Add the inverse transmission effort conversion used only by the host plant.
- Add bounded builders for joint-effort direct-torque and fixed-gain MIT
  proposals. They must validate exact config identity/order and finite units,
  then return ordinary `ArmControlProposal` values.
- Do not change `ArmControlAdmission`, lease checks, lifecycle checks, output
  limits, slew checks, collision typestate or `SimExecutor` record semantics.
- Test that adaptive effort still fails when any existing admission gate fails.

Validation:

```sh
cargo test --locked -p arm-model
cargo test --locked -p arm-control --test control
cargo clippy --locked -p arm-control --all-targets --all-features -- -D warnings
```

### 6. Implement deterministic plant, sensor contract and qualification runs

Files owned:

- `apps/rs-arm-simulation/src/lib.rs`;
- `apps/rs-arm-simulation/tests/qualification.rs`.

Tasks:

- Integrate payload-augmented ABA with explicit fixed timestep and bounded
  semi-implicit Euler updates.
- Simulate MIT actuator response, effort saturation and power-consistent
  transmission back to joint effort.
- Form only semantic `ArmJointObservation` values with deterministic bounded
  noise and monotonically increasing synthetic headers.
- Build synthetic leases/envelopes and send every proposal through
  `ArmControlAdmission` and `SimExecutor`.
- Run matched, mismatched nominal-only, mismatched calibrated/adaptive and
  small-mismatch RLS scenarios from byte-identical initial states and commands.
- Run FK/IK round trips and moving admitted commands as separate qualification
  stages; retain oracle-only plant payload/gravity in the harness.

Validation:

```sh
cargo test --locked -p rs-arm-simulation --test qualification -- --test-threads=1
tools/run-arm-simulation.sh verify
```

Risk: a stiff MIT configuration can make a fixed-step host plant numerically
unstable. Use the checked-in timestep, gains, target spacing and actuator limits
as validated configuration; fail the run on non-finite or limit-crossing state
instead of silently clipping positions.

### 7. Publish the process document and deterministic evidence

Files owned:

- new `docs/arm-payload-identification.md`;
- `.trellis/tasks/08-11-arm-simulation-adaptive-gravity/research/payload-identification-process.md`;
- `evidence/arm-adaptive-gravity-simulation.json`;
- `apps/rs-arm-simulation/src/lib.rs`;
- `tools/run-arm-simulation.sh`;
- `tools/ci.sh`;
- `README.md`.

Tasks:

- Copy the approved process into the user-facing docs path and replace
  implementation-neutral alternatives with the selected analytical regressor,
  state names, configuration fields and test names.
- State clearly which report fields are simulation-only oracle values and that
  timing/allocation evidence is a separate host attachment.
- Generate the canonical report, hash it, and check in the expected output.
- Make `tools/ci.sh` call the wrapper's read-only `verify` command after the
  workspace/application checks; it must never regenerate or overwrite evidence.

Validation:

```sh
tools/run-arm-simulation.sh verify
python3 ./.trellis/scripts/task.py validate .trellis/tasks/08-11-arm-simulation-adaptive-gravity
```

### 8. Run the complete quality gate

Warm allocation checks:

```sh
cargo test --locked -p arm-algorithms --test gravity_prepared
cargo test --locked -p arm-algorithms --test adaptive_gravity
cargo test --locked -p rs-arm-simulation --test qualification
```

Release latency checks (isolated and single-threaded):

```sh
cargo test --locked --release -p arm-algorithms --test gravity_performance -- --ignored --nocapture --test-threads=1
cargo test --locked --release -p arm-algorithms --test ik_performance -- --ignored --nocapture --test-threads=1
```

Repository gates:

```sh
cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
cargo test --locked --workspace --all-features
cargo run --locked -p architecture-check -- check
tools/ci.sh
```

The last command repeats several checks intentionally; its result is the final
repository gate. Do not claim acceptance if any primary accuracy, allocation,
identity, determinism or admission test is skipped.

## Risky files and rollback points

| Risk | Guard | Rollback |
| --- | --- | --- |
| `arm-algorithms/src/kinematics/ik/prepared.rs` shared by all IK kernels | Run all kinematics/IK tests after additive FK wrapper | Revert only the wrapper and its tests; keep old prepared IK. |
| `arm-algorithms/src/dynamics/mod.rs` existing constructor/golden path | `ArmDynamics::new` delegates to the exact no-payload path; run pendulum and RNEA/ABA tests | Remove payload constructor/append; keep nominal dynamics and prepared gravity. |
| QR/RLS numerical projection | Independent analytical fixture, Featherstone oracle, degenerate/outlier/long-run tests | Disable activation/RLS and retain nominal gravity; no active snapshot migration. |
| `arm-control` proposal helper | Existing admission tests plus new identity/limit/slew tests | Remove helper; callers can continue constructing existing proposals manually. |
| New simulation app/report | Architecture check, byte-equality test, `--verify` against checked-in JSON | Remove app/evidence only; production crates retain additive APIs. |
| User-facing process document | Test names and state/config cross-check in review | Retain task research document and defer publication without altering code. |

## Pre-start checklist

- [x] User has reviewed and approved all three planning artifacts.
- [x] PRD contains no resolved/open brainstorm questions or “recommended”
      wording.
- [x] Every requirement ID maps to a design section and an implementation/test
      step.
- [x] OPS-01 and DOC-02 map to the wrapper, CI verify hook and README examples.
- [x] No estimator API accepts simulation-only truth.
- [x] No hardware/ROS dependency is introduced by the simulation app.
- [x] Existing nominal gravity, RNEA/ABA, FK/IK and admission contracts remain
      covered by their current tests.
