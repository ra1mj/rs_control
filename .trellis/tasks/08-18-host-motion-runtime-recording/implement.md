# Host motion runtime and telemetry recording implementation plan

## Preparation

- [x] Load trellis-before-dev and directory, robot-contract, error, logging,
      realtime, and quality specifications before source edits.
- [x] Inspect mcap 0.25.0 resolved features, license, and source policy. Record
      the admission decision before adding production imports.
- [x] Add telemetry-recorder and motion-rt workspace and
      architecture entries atomically.

## Bounded host motion runtime

- [x] Implement fixed-capacity runtime inputs, decisions, diagnostics, and fake
      sink records.
- [x] Compose preallocated latest/ordered ports, exact lease enforcement, and
      separate synthetic I/O, control, and drain threads.
- [x] Keep control as injected-tick step_at. Reject stale state, wrong lease,
      stale sequence, deadline expiry, and peer loss without waiting.
- [x] Test warm allocations, channel full/disconnect, worker loss, ordered
      shutdown, and fake-sink SafeStop behavior.

## Non-RT diagnostics

- [x] Adapt existing typed health/fault values into a fixed snapshot.
- [x] Drain compact diagnostics outside control without adding a logging runtime.
      export only there.
- [x] Test drain absence, saturation, and recorder failure without changing
      authority, deadlines, or sink behavior.

## MCAP recorder and replay

- [x] Implement bounded records, priority queues, and coalesced gaps from
      original Protobuf bytes and descriptor-set schema data.
- [x] Implement create-new staging, chunking, rotation, quota, sealing, and
      interrupted-output recovery through the admitted MCAP API.
- [x] Implement strict replay validation and a test-only bounded input return.
- [x] Test valid/malformed data, pressure, quota/failing storage, interruption,
      quota, and deterministic replay. Prove replay cannot form output/driver
      commands.

## Integration and review

- [x] Verify no hot path allocates, formats, logs, reads a clock, performs I/O,
      or blocks after preparation.
- [x] Add least-privilege systemd examples with explicit motor-disabled host
      mode. Do not claim platform qualification.
- [x] Run focused tests, warnings-denied Clippy, architecture, metadata,
      dependency/source policy, and tools/ci.sh.
- [x] Update durable specifications; commit and archive after final diff review.
      only after the full host gate.

## Validation

    cargo fmt --all -- --check
    cargo test -p telemetry-recorder -p motion-rt
    cargo clippy -p telemetry-recorder -p motion-rt --all-targets --all-features -- -D warnings
    cargo run --locked -p architecture-check -- check
    cargo metadata --locked --no-deps --format-version 1
    cargo deny check
    tools/ci.sh

## Rollback

- If dependency admission fails, stop before runtime imports or wire-format
  work.
- If diagnostics affect control, remove the diagnostic integration and retain
  standalone SafeStop behavior.
- If recovery cannot prove non-overwrite behavior, retain failed staging output
  and do not expose it to replay.
