# Implementation plan: lightweight three-level node observability CLI

## Ordered checklist

1. Add the bounded transport diagnostic contract in `navigation-ipc`.
   - Define semantic channel/counter snapshot types and strict constructors.
   - Define fixed wire representation and codec with freshness/identity checks.
   - Add channel name, layout creation, publisher and read-only observer aliases.
   - Add protocol tests for round-trip, malformed padding/counts/duplicates and
     read-only observation.
2. Publish transport diagnostics from `rs-navigation-e2e`.
   - Attach the new publisher after layout creation.
   - Aggregate existing endpoint counters/SHM diagnostics into bounded entries.
   - Publish best-effort at the existing participation cadence.
   - Keep all publishing outside authority/reducer decisions and motor output.
3. Extend `btctl`.
   - Add one `observe` command that presents tree, nodes and transport sections
     together, with optional section filtering and interval/sample options.
   - Use an in-place three-pane dashboard on an attached terminal; keep
     deterministic key=value records for pipes and explicit `--plain` mode.
   - Factor absolute namespace validation and freshness/error reporting.
   - Read participation once per sample for both tree and node rendering.
   - Render tree hierarchy and participant states without claiming leases.
   - Render transport counters with stable key=value records and derive a
     bounded dashboard delta log outside the producer/realtime path.
   - Preserve `example` and `inspect` byte-stable behavior.
4. Add process-level tests in `apps/btctl/tests/cli.rs`.
   - Build a temporary namespace and publish participation/transport samples.
   - Assert each level's stdout, status and empty stderr.
   - Assert watch mode emits the requested finite sample count.
   - Assert stale/missing/incompatible data returns nonzero without mutating
     producer counters.
   - Assert the dashboard renderer contains the fixed tree/node/transport panes
     and recognizes all exit keys.
5. Run formatting, focused tests, clippy and the full host quality gate.

## Validation commands

```text
cargo fmt --all -- --check
cargo test -p navigation-ipc
cargo test -p btctl
cargo test -p rs-navigation-e2e
cargo clippy -p navigation-ipc -p btctl -p rs-navigation-e2e --all-targets -- -D warnings
tools/ci.sh
```

## Risk and rollback points

- **Wire contract risk:** keep the payload additive, explicitly versioned and
  bounded; if codec tests fail, stop before changing runtime publishers.
- **Runtime coupling risk:** publish only after normal endpoint work and ignore
  publish failures; a regression can be rolled back by removing the publisher
  call without touching reducer logic.
- **CLI compatibility risk:** leave existing command branches and golden output
  untouched; add new tests before refactoring shared helpers.
