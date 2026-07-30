# Architecture Module Naming Migration Implementation Plan

## 1. Preconditions

- [x] Preserve unrelated working-tree files and Trellis bootstrap material.
- [x] Record the current `cargo metadata --no-deps` package/path graph for
      before/after comparison.
- [x] Confirm the SocketCAN/Damiao task declares this task as a prerequisite.

## 2. Atomic rename

- [x] Move `crates/lower` to `crates/hardware_module` and rename
      `lower-core` to `hardware-module-core`.
- [x] Move `crates/middle` to `crates/protocol_module`.
- [x] Move `crates/upper` to `crates/application_module`.
- [x] Update root Cargo member/dependency paths, the hardware core package name
      and resulting lockfile package metadata.
- [x] Rename architecture class keys, allowed dependency sets, member classes
      and path prefixes without changing graph reachability.
- [x] Rename all three forbidden-edge fixture files and update their package
      paths, class values, test function names and expected diagnostics.
- [x] Update source docs, README, platform docs, specifications, active tasks,
      archived task artifacts and journal architecture vocabulary.

## 3. Static checks

- [x] Run targeted `rg` checks for the retired paths, class tokens,
      `lower-core`, fixture names and architecture-layer phrases; classify any
      remaining lexical hit before changing it.
- [x] Run `cargo metadata --no-deps --format-version 1` and compare package and
      dependency identities with the pre-migration graph.
- [x] Run `cargo run -p architecture-check -- check`.
- [x] Run `cargo test -p architecture-check` to prove the renamed negative
      fixtures still reject all forbidden directions.

## 4. Regression gate

- [x] Run `cargo fmt --all -- --check`.
- [x] Run `tools/ci.sh`.
- [x] Review `git diff --check`, `git status --short` and the final rename-aware
      diff for accidental semantic edits.
- [x] Commit and push the naming migration independently before adding the new
      SocketCAN or Damiao crates.

## 5. Rollback point

If metadata, architecture reachability or external schemas differ beyond the
declared rename, stop and revert the isolated migration commit rather than
adding compatibility shims.
