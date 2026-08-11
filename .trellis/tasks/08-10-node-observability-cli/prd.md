# Lightweight three-level node observability CLI

## Goal

Add a small read-only CLI for operator diagnosis at three levels:

1. **Tree level:** follow the live behavior-tree shape and aggregate runtime state.
2. **Node level:** inspect every behavior-tree/navigation participant and its typed state/reason.
3. **Transport level:** inspect a fixed-capacity diagnostic channel containing real-time send counters.

Observation failure is diagnostic-only. The CLI must never acquire authority, mutate
shared-memory control state, consume a normal subscriber lease, or change the motion
safety path.

## Confirmed facts

- `apps/btctl` is the existing operator-facing behavior-tree utility with no hardware access.
- `control-core::behavior_tree` already exposes bounded `BehaviorTreeSnapshot` values, exact activation states, and deterministic text renderers.
- `navigation-ipc` already exposes a read-only `ParticipationObserver` for
  `NavigationParticipationSnapshot`, including participant ID/state/reason,
  command ID, sequence, process epoch and freshness metadata.
- `middleware-shm` already provides fixed-layout latest-value channels and
  best-effort read-only observers. Endpoint implementations expose saturating
  `ChannelCounters` and SHM diagnostics to their owners.
- There is no generic live behavior-tree IPC snapshot or unified transport-event
  log. The existing navigation participation snapshot is the current live state boundary;
  the new transport diagnostics snapshot will be the lower-level boundary.
- The logging contract requires deterministic stdout, one process-prefixed stderr
  error on failure, typed/unavailable metrics instead of inferred zeros, and no
  formatted logging in realtime paths.

## Requirements

- R1: Extend `btctl` with one primary `observe` entry point that displays the
  `tree`, `nodes` and `transport` sections together by default, while preserving
  `example` and `inspect`. An optional section filter may narrow the output.
- R2: Tree-level output must show a stable navigation tree hierarchy, current
  aggregate state, and the current state/reason of tree participants. It must
  support one-shot and bounded-interval watch modes.
- R3: Node-level output must show every participant from the latest
  `NavigationParticipationSnapshot`, together with generation, source, boot,
  sequence, process epoch, command and explicit stale/unavailable errors.
- R4: Transport-level output must read a new fixed-capacity latest-value diagnostic
  channel. Each entry identifies a channel and reports published/consumed/
  overwritten/full/disconnected counters plus SHM diagnostic counters where available.
- R5: All three levels are read-only. Watch mode repeatedly samples without claiming
  a subscriber lease, consuming data, or updating shared diagnostics.
- R6: An attached terminal defaults to a fixed three-pane live dashboard with the
  tree above node states and a rolling transport-send log below. State is color
  coded, the screen is updated in place, and `q`, Escape or Ctrl-C restores the
  terminal and exits cleanly.
- R6a: Non-terminal output and explicit `--plain` mode retain stable key=value
  records. Errors return non-zero and go to stderr; a finite sample limit exits
  either mode cleanly.
- R7: Missing, stale, malformed, incompatible or empty snapshots are rendered as
  explicit unavailable/invalid diagnostics and never as healthy zero values.
- R8: Add process-level CLI tests and protocol tests for success, watch sampling,
  invalid paths, stale/incompatible snapshots and preservation of producer counters.

## Acceptance criteria

- [x] `cargo run -p btctl -- observe --ipc-root ...` displays all three sections
      in one sample, and
      `example`/`inspect` retain their existing output and semantics.
- [x] Tree and node views are deterministic and parseable for the existing
      motor-disabled navigation participation stream.
- [x] The transport view reads the new diagnostic channel and reports actual
      counter values; absent data is a non-zero, explicit unavailable error.
- [x] Watch mode repeats complete three-section samples at the requested interval
      and remains non-mutating.
- [x] An 80x24 terminal shows a fixed tree pane, all current participant rows and
      a tailing transport-counter delta log without scrolling the whole screen.
- [x] Pipes, redirection and `--plain` preserve deterministic key=value output.
- [x] New tests and the relevant workspace quality gates pass.

## Scope boundaries

- Do not modify Nav2's internal behavior tree, add ROS dependencies, add a GUI,
  or turn the CLI into a control/debug-write tool.
- Do not format per-frame logs on a realtime thread. The new channel carries a
  bounded latest snapshot of counters; it is best-effort and never an input to
  authority, limiting or output decisions.

## Resolved planning decision

The lower-level source is a new read-only fixed-capacity latest-value diagnostic
channel rather than stdout/log parsing. This keeps the contract stable, avoids
log rotation/format coupling, and preserves the existing non-blocking middleware
semantics at the cost of one additive protocol payload and publisher call.
