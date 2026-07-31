# Diagnostics and CLI Output

## Current boundary

The workspace does not currently standardize on a `log` or `tracing` runtime.
Library crates under `crates/` do not write to stdout/stderr and do not make
formatted text part of safety behavior. They return typed errors and expose
bounded diagnostic snapshots for a non-RT owner to publish, record, or render.

Source-backed examples:

- `hardware-module-core::TransportHealth` and
  `transport-socketcan::SocketCanHealth` report link state and saturating
  counters without formatting in the transport hot path.
- `middleware-api::ChannelCounters`, `middleware-shm::ShmDiagnostics`, and
  `middleware-zenoh::BridgeCounters` expose copyable snapshots rather than
  logging each delivery or drop.
- `message-model::{ComponentHealth, Fault}` owns the semantic health/fault
  representation that crosses a protocol boundary.

Do not add a logging dependency to one library ad hoc. Introducing a runtime
subscriber, persistent recorder, or log-level policy is an architecture change
and must define the non-RT drain boundary first.

## Runtime diagnostics

- Represent machine decisions with enums, numeric fault codes, typed errors,
  lifecycle state, counters, and explicit availability. Never parse log text
  to decide whether motion is allowed.
- Counter updates on a hot path are bounded and saturating. An unavailable
  metric is `MetricValue::Unavailable`, not a measured zero.
- Preserve clock domains next to timestamps. Diagnostic realtime/hardware
  timestamps are not interchangeable with `MonoTime`.
- Preserve raw evidence needed for later classification, such as CAN error
  bits and bytes; a rendering layer may add human-readable context without
  discarding the typed/raw value.
- High-rate success events are counted or sampled outside the RT path. Do not
  format one line per frame, control cycle, middleware sample, or retry.

## CLI stdout and stderr

Command-line executables keep presentation at the process boundary:

```rust
match run(Cli::parse()) {
    Ok(()) => ExitCode::SUCCESS,
    Err(error) => {
        eprintln!("robotctl: {error}");
        ExitCode::FAILURE
    }
}
```

`apps/robotctl/src/main.rs` and `tools/architecture-check/src/main.rs` are the
reference shape:

- `run` performs fallible work and returns a typed/boxed error to `main`.
- Successful, script-consumable output goes to stdout.
- A terminal failure writes one program-prefixed diagnostic to stderr and
  returns non-zero.
- Stable output is deterministic. `apps/robotctl/tests/config_cli.rs` asserts
  byte-identical normalized stdout and empty stderr on success.

Avoid emitting progress chatter to stdout. If an output becomes a machine
contract, prefer a versioned structured record over prose and add a process
integration test for status, stdout, and stderr.

## Qualification and evidence output

Platform qualification commands keep human console output separate from
immutable evidence. Structured JSON, environment manifests, raw tool output,
and `SHA256SUMS` files under `platform/orangepi5plus/` are the review input;
console lines are not qualification evidence.

- Never overwrite or append to an existing evidence directory.
- Record missing measurements explicitly and retain raw observations.
- A sealed manifest hashes the complete expected file set and offline
  verification re-derives the verdict.
- Host research remains labeled as host research even when its console output
  looks successful.

## Verification

When changing diagnostics or CLI output:

- test counter saturation, unavailable values, combined fault flags, and
  snapshot consistency at the owning library boundary;
- add or update a process-level test for stable stdout/stderr and exit status;
- run `tools/ci.sh`;
- run the applicable target evidence gate as an additional check when a claim
  concerns physical timing, hardware, or realtime qualification.
