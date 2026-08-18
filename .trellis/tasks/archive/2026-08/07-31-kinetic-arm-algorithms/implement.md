# Project-native arm algorithms and policy control — implementation plan

## 1. Execution strategy

This parent coordinates two independently verifiable children. Do not
implement the parent as one monolithic change.

| Order | Task | Exit condition |
|---|---|---|
| 1 | `07-31-kinetic-arm-stack` | Stable project-owned profile/group/joint/proposal contracts and full host-tested native arm surfaces |
| 2 | `07-31-rl-policy-control` | ONNX/RTen policy runtime consumes child-1 contracts and emits bounded typed proposals |
| 3 | Parent integration | Cross-child invalidation, full architecture/schema/dependency/host gates pass |

The RL child has an explicit hard dependency on child 1. Tree position alone
does not satisfy this dependency.

## 2. Global implementation gates

- [x] Keep the task in planning until the user approves `prd.md`, `design.md`
      and `implement.md`.
- [x] Start and finish the arm-stack child before starting RL implementation.
- [x] Load `trellis-before-dev` before changing production files in inline
      execution mode.
- [x] Classify every new crate in root `Cargo.toml` and `architecture.toml` in
      the same change.
- [x] Do not add a KINETIC dependency, submodule, copied workspace or public
      KINETIC namespace.
- [x] Add exact provenance comments/records for substantially derived kernels.
- [x] Keep generated Protobuf Rust in `OUT_DIR`; change only `.proto`,
      semantic types and codec mappings.
- [x] Keep all synthetic arm and ONNX artifacts non-production and explicitly
      host-only.
- [x] Do not run hardware, CAN transmit, Orange Pi, camera or live ROS tests.

## 3. Child 1 — native arm stack

Execute the detailed plan in
`../07-31-kinetic-arm-stack/implement.md`.

Parent review points:

- [x] Protocol arm identities and tagged angular/prismatic values are stable.
- [x] MIT proposal types contain no bus, motor ID, opcode or frame bytes.
- [x] Profile identity includes group order, limits and controller mapping.
- [x] Multi-group and unsupported future-topology behavior is tested.
- [x] Every KAA-10 surface exists and reports typed bounded evidence.
- [x] `CollisionUnchecked` cannot reach physical executable admission.
- [x] No missing inertia is replaced with a nominal mass.
- [x] Existing YAML, driver and chassis behavior remains compatible.

Rollback point: revert only child-1 additions. Do not weaken architecture,
profile identity or collision admission to unblock RL.

## 4. Child 2 — RL policy control

Start only after child 1 is completed and its public contracts pass review.
Execute the detailed plan in
`../07-31-rl-policy-control/implement.md`.

Parent review points:

- [x] `rten = "=0.24.0"` is the only production inference backend.
- [x] Model loading uses bounded safe reads, not unsafe mmap.
- [x] Exact profile/group/order and model-generation checks occur before
      action decoding.
- [x] Position, residual, velocity, torque and fixed-gain MIT actions all have
      golden decoding and rejection tests.
- [x] MIT `kp/kd` is immutable artifact data and never a learned output.
- [x] Queue overflow, late inference, invalid output, worker loss and reload
      withdraw proposals without refreshing authority.
- [x] RL code imports no hardware crate and constructs no raw motor command.

Rollback point: remove the RL crate/artifacts and retain the stable native arm
stack. Do not substitute another ML runtime automatically.

## 5. Parent integration checks

- [x] Test plan/profile/policy identity mismatch across at least two
      morphologies and two named groups.
- [x] Test old output and recurrent-state rejection after both model reload and
      profile switch.
- [x] Test direct-torque and MIT proposal flow through project constraints to
      the semantic Damiao mapper with fake verified runtime limits.
- [x] Prove the same proposals cannot reach physical admission while carrying
      `CollisionUnchecked`.
- [x] Test cancellation, lease expiry and command deadline without allowing a
      late policy result to renew validity.
- [x] Confirm existing `arm.morphology: none` configurations normalize and hash
      exactly as before.
- [x] Review every changed boundary against
      `.trellis/spec/backend/{directory-structure,robot-contracts,error-handling,logging-guidelines,quality-guidelines}.md`.

## 6. Validation commands

Targeted checks as crates appear:

```bash
cargo test -p message-model -p contract-codec -p motor-damiao
cargo test -p arm-model -p arm-algorithms -p arm-control
cargo test -p rl-policy-control
cargo clippy -p arm-model -p arm-algorithms -p arm-control -p rl-policy-control --all-targets --all-features -- -D warnings
cargo run --locked -p architecture-check -- check
```

Schema and cross-language checks:

```bash
tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb
cargo test -p proto-generated -p contract-codec -p proto-compat-check
```

Dependency and full host gates:

```bash
cargo metadata --no-deps --format-version 1
cargo deny check
./tools/ci.sh
```

Use the actual checked-in wrapper syntax if `tools/bufw --help` differs; do not
bypass the project wrappers.

## 7. Final review and delivery

- [x] Run the PRD/implementation acceptance matrix and record any explicitly
      open physical/collision gates.
- [x] Run `trellis-check` in inline mode, then the full host gate.
- [x] Use `trellis-update-spec` for durable arm/profile/RL contracts learned
      during implementation.
- [x] Commit only task-owned files; preserve unrelated Trellis/environment
      files already present in the worktree.
- [ ] Push the approved commits to the configured branch.
- [x] Archive children independently, then archive the parent only after the
      cross-child integration review passes.

Physical arm, collision, floating-base, coordinated dual-arm, Orange Pi and
live ROS gates remain open after this software delivery.
