# Design: lightweight three-level node observability CLI

## Boundary and ownership

The feature stays within the existing `btctl` composition root and the
`navigation-ipc` protocol-module boundary:

```text
rs-navigation-e2e (best-effort owner-side aggregation)
    |-- navigation.participation.v1 ---------+
    `-- navigation.transport-diagnostics.v1 -+--> btctl observe
```

`control-core` remains the owner of behavior-tree semantics. The CLI does not
reconstruct or execute a reducer. The protocol crate owns the new semantic
diagnostic snapshot, fixed representation, codec, channel layout and read-only
observer alias. `btctl` owns only validation, polling, aggregation presentation
and process exit behavior.

## CLI contract

Existing commands remain unchanged:

```text
btctl example
btctl inspect --ipc-root <absolute-root>
```

The additive primary command is:

```text
btctl observe --ipc-root <absolute-root>
    [--section all|tree|nodes|transport]
    [--interval-ms <n>] [--samples <n>] [--plain]
```

When stdout is an attached terminal and `--plain` is absent, `observe` enters an
alternate-screen dashboard. It refreshes in place every 250 ms by default and
uses three fixed vertical panes: behavior tree, participant table and rolling
transport-counter delta log. State values are color coded. `q`, Escape and
Ctrl-C restore the original terminal. `--samples` can bound a dashboard run.

When stdout is redirected/piped, or when `--plain` is provided, `interval-ms`
defaults to one-shot (`0`). A positive interval enables text watch mode. Every
plain sample is prefixed with one stable `sample=` record followed by
`section=tree`, `section=nodes` and `section=transport` blocks so a consumer can
delimit complete updates. `section` defaults to `all` in both modes.

The tree and node sections share one read of the existing navigation
participation snapshot so they describe the same generation. Tree view uses
that snapshot as the live
runtime boundary and renders the stable two-level navigation tree:

```text
tree state=active generation=... sequence=... source=... command=...
node id=tree.root state=...
  node id=tree.navigate_to_pose state=...
```

Node view prints the full participant list and all snapshot metadata. The
transport section reads its separate diagnostic snapshot before any output is
emitted, so a failed complete sample does not leave partial stdout. A stale
or invalid sample returns a typed CLI error instead of silently printing zeros.

The plain transport view prints diagnostic metadata followed by one line per
channel, including all common and SHM-specific counters. The dashboard derives
a bounded operator-side log from counter differences between successive
snapshots; this does not add an event stream or formatted logging to the
producer.

## Semantic diagnostic payload

`navigation-ipc::NavigationTransportDiagnosticsSnapshot` is a latest-value,
fixed-capacity semantic value:

- nonzero process epoch and command-correlated `Header` metadata;
- 16 bounded channel entries (`AsciiId<MAX_ID_LEN>` IDs, unique and nonempty);
- each entry has `ChannelCounters` and `ShmDiagnostics` fields.

The wire representation is `Pod + Zeroable` with an explicit schema ID/version,
zeroed padding, bounded count and strict trailing-entry validation. Decoding
reuses the existing header freshness validator and rejects malformed IDs,
unknown flags, duplicate channels and invalid counts.

The channel is named `navigation.transport-diagnostics.v1`, uses the existing
latest config, and exposes publisher/observer aliases. The observer never claims
the subscriber endpoint and never updates shared consumption or decode counters.

## Producer aggregation

`rs-navigation-e2e` creates the diagnostic layout and publishes a snapshot at
the same bounded 50 ms cadence as participation. It aggregates counters from
the already-attached navigation endpoints (authority, action request/response,
proposal, odometry, dynamic TF, health and participation). The diagnostic
publisher is intentionally excluded from its own report. Publishing is
best-effort and its result is ignored; failure cannot affect the reducer,
authority admission, final limiter or motor-disabled sink.

## Compatibility and failure behavior

- Existing IPC layouts remain byte-compatible; the new layout is additive.
- Existing `create_layouts` callers create the extra file automatically.
- A namespace created by an older process simply has no diagnostic region; the
  CLI reports `unavailable` and exits nonzero for the transport level.
- Watch mode polls only the read-only observer. It sleeps outside any realtime
  path and never consumes the normal participation subscriber.
- Reverting the feature removes the extra channel and publisher while leaving
  behavior-tree execution and authority behavior unchanged.
