# Host verification

Date: 2026-08-18

All evidence below is Linux x86_64 host-only. It does not qualify Orange Pi or
RK3588 scheduling, storage, CAN, ROS, motor enablement, or physical stopping.

## Passed

- `cargo test -p control-core -p motion-rt -p telemetry-recorder`
- `cargo clippy -p control-core -p telemetry-recorder -p motion-rt --all-targets --all-features -- -D warnings`
- `cargo tree -p telemetry-recorder -e features | rg 'mcap|zstd|lz4'`
  confirmed `mcap v0.25.0` with neither default compression feature enabled.
- `cargo run --locked -p architecture-check -- check`
- `cargo metadata --locked --no-deps --format-version 1`
- `cargo deny check`
- `tools/ci.sh`

## Behavioral coverage

- Host role separation, warm zero allocations, exact lease/sequence/deadline
  admission, wrong-lease rejection, bridge/diagnostic/recorder/worker loss and
  ordered shutdown all reach only the motor-disabled fake sink.
- MCAP retains original bytes and typed metadata, creates a new staging output,
  seals a manifest, recovers complete staging, retains malformed staging,
  represents pressure with gaps, degrades on quota, and rejects malformed or
  descriptor-incompatible replay.
