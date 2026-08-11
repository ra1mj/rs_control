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

## Scenario: Expose live diagnostics to a read-only CLI

### 1. Scope / Trigger

Use this contract when an operator CLI must follow live state or delivery
counters from a local realtime-adjacent process. It applies to behavior-tree,
participant, middleware and transport observation.

### 2. Signatures

```rust
ShmNamespace::attach_latest_observer<T, C>(name, config, codec)
    -> Result<ShmLatestObserver<T, C>, ShmError>
ShmLatestObserver::try_observe_latest(&mut self) -> Option<LatestSample<T>>
```

```text
btctl observe --ipc-root <absolute-root>
    [--section all|tree|nodes|transport]
    [--interval-ms <u64>] [--samples <nonzero-u64>] [--plain]
```

### 3. Contracts

- The runtime owner publishes a fixed-capacity latest diagnostic snapshot at a
  bounded cadence. Publishing is best effort and its result is never an input
  to authority, limiting, cancellation, output or restart decisions.
- High-rate success is represented by saturating counters. Do not publish one
  formatted or ordered event for every frame, control cycle or retry.
- The CLI attaches through `ShmLatestObserver`, never through the normal
  subscriber endpoint. Observation does not claim a subscriber lease, advance
  consumption, acknowledge delivery or update shared decode/race counters.
- Every snapshot carries source, boot, sequence, process epoch and explicit
  local-monotonic freshness. Command-correlated diagnostics retain the command
  ID. Missing or expired data is unavailable, not a zero-valued healthy sample.
- An attached terminal may use an alternate-screen dashboard for human
  observation. It must restore raw mode, cursor visibility and the prior screen
  on `q`, Escape, Ctrl-C, a finite sample limit or an error. Tree, participant
  and transport views remain read-only and update in place rather than appending
  progress chatter.
- A rolling human transport log is derived only in the non-RT CLI by diffing
  successive counter snapshots. Do not add a producer-side formatted event
  stream to make the dashboard look live.
- Redirected/piped output automatically uses stable key=value stdout;
  `--plain` forces the same mode on a terminal. One terminal error goes to
  stderr with a nonzero exit status; plain watch output is flushed after each
  complete sample.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Relative, missing or non-directory namespace | reject before attach; nonzero CLI exit |
| Missing diagnostic region from an older runtime | explicit unavailable error; no fallback to control subscription |
| Wrong schema/version/size/padding/count | typed SHM/codec rejection; no partial snapshot |
| Snapshot at or beyond exclusive local expiry | unavailable/stale error; never reuse as current |
| Zero process epoch, empty or duplicate entry set | semantic constructor rejection |
| `--samples 0` | CLI argument error in every mode |
| Plain/non-terminal multiple samples with zero interval | CLI argument error |
| Interactive zero interval | use the documented dashboard refresh default |
| Dashboard snapshot expires after attach | render explicit unavailable state and keep polling read-only |
| Terminal setup/draw/input failure | restore terminal state, emit one prefixed error and exit nonzero |
| Observer reads a valid sample | producer counters and subscriber lease state remain unchanged |

### 5. Good / Base / Bad Cases

- Good: a producer aggregates typed middleware counters every 50 ms into one
  latest snapshot; an attached-terminal dashboard renders tree/node panes and
  derives a bounded transport delta log without changing those counters.
- Base: a one-shot observer prints the newest fresh generation once and exits successfully.
- Bad: tail process stdout, parse text to infer safety state, attach as the sole
  subscriber, emit one log line per message from the producer hot path, or send
  ANSI dashboard frames into a pipe that expects stable records.

### 6. Tests Required

- Protocol tests round-trip the fixed payload and reject stale, malformed,
  padded, empty and duplicate-entry values.
- SHM tests publish through the diagnostic bypass, observe through a read-only
  mapping and assert publisher counters are byte-for-byte unchanged by observation.
- Process CLI tests assert stable stdout, empty stderr on success, nonzero
  missing/stale/incompatible errors and exact finite watch sample counts.
- Dashboard renderer tests assert the fixed tree/node/transport pane structure,
  exit-key handling and bounded counter-delta formatting. A real pseudo-terminal
  smoke test verifies in-place refresh and terminal restoration.
- Run warnings-denied Clippy and `tools/ci.sh`.

### 7. Wrong vs Correct

#### Wrong

```rust
// Steals the normal SPSC consumer capability and changes delivery accounting.
let mut diagnostics = namespace.attach_latest_subscriber(name, config, codec)?;
println!("{:?}", diagnostics.try_read_latest());
```

#### Correct

```rust
// Read-only diagnostics never participate in the channel's delivery contract.
let mut diagnostics = namespace.attach_latest_observer(name, config, codec)?;
let sample = diagnostics
    .try_observe_latest()
    .ok_or(DiagnosticUnavailable)?;
render_outside_realtime(sample);
```

#### Wrong

```rust
// Makes machine output terminal-dependent and pushes formatted events upstream.
println!("\x1b[2Jchannel={channel} sent={count}");
producer.publish_formatted_log(...);
```

#### Correct

```rust
// Select presentation only at the process boundary. The producer publishes
// typed counters; the observer derives human deltas locally.
if stdout.is_terminal() && !options.plain {
    run_read_only_dashboard(observers)?;
} else {
    print_stable_records(observers)?;
}
```

## Qualification and evidence output

Platform qualification commands keep human console output separate from
immutable evidence. Structured JSON, environment manifests, raw tool output,
and `SHA256SUMS` files under `platform/orangepi5plus/` are the review input;
console lines are not qualification evidence.

- Never overwrite or append to an existing evidence directory.
- Record missing measurements explicitly and retain raw observations.
- Record the workload phase and the exact event that opened sampling. Startup,
  idle and active-control snapshots are distinct evidence and must not be
  relabeled as one another.
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
