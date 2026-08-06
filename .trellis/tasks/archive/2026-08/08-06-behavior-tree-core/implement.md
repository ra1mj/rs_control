# Behavior-tree core — implementation plan

1. [x] Read the relevant application, mission, logging and quality specifications.
2. [x] Add the `behavior_tree` facade and public, fixed-capacity snapshot values in
   `crates/application_module/control-core`.
3. [x] Extend the existing reducer to retain per-node latest terminal outcomes and
   emit action-level deadline effects without changing its no-clock/no-I/O
   contract.
4. [x] Add unit/replay/allocation tests for the facade, snapshot and deadline
   behavior.
5. [x] Add `apps/btctl`, its workspace/architecture registration and process-level
   CLI tests for deterministic output and invalid invocation behavior.
6. [x] Run targeted `control-core`/`btctl` tests, Clippy/format/architecture checks,
   then the full host CI gate.

## Verification evidence

- `cargo test -p control-core -p btctl`
- `cargo clippy -p control-core -p btctl --all-targets -- -D warnings`
- `cargo run -p architecture-check -- check`
- `./tools/ci.sh`

## Rollback points

- The facade is additive; revert it without renaming existing `Mission*` APIs.
- The only reducer behavior change is the previously missing action deadline
  effect. Tests must prove that no deadline is armed twice for one activation.
