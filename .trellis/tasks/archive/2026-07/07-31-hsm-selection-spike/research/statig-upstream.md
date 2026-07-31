# statig upstream baseline

Recorded: 2026-07-31

## Primary sources

- Documentation: https://docs.rs/statig/0.4.1/statig/
- Repository: https://github.com/mdeloof/statig
- Published crate: https://crates.io/crates/statig/0.4.1
- Manifest:
  https://raw.githubusercontent.com/mdeloof/statig/main/statig/Cargo.toml

## Verified package contract

`cargo info statig@0.4.1` and the published crate manifest report:

- version `0.4.1`;
- Rust 2021 with declared MSRV 1.66;
- MIT license;
- default feature `macro`;
- optional `async`, `serde` and `bevy` features;
- `macro` depends on `statig_macro`.

The final candidate dependency is exact-pinned with default features disabled
and no optional features. An initial macro prototype proved functional, but
Rust 1.97 reported its `proc-macro-error2 2.0.1` dependency as
future-incompatible. The comparison was therefore rerun against hand-written
`IntoStateMachine`, `State` and `Superstate` implementations so the optional
macro did not leave a future compiler failure in the workspace.

## Relevant behavior

The official documentation describes:

- hierarchical leaf states and superstates;
- synchronous `Handled`, `Transition` and `Super` outcomes;
- state-local and shared storage;
- entry/exit actions;
- caller-owned mutable context;
- before/after dispatch and transition callbacks;
- an optional async mode that is not needed here.

The docs explain that transitions walk the hierarchy and execute exit then
entry actions along the shortest path. This makes synchronous cancellation
mechanically possible, but its exact observable ordering still needs a local
test.

Source inspection found two additional transition-visibility costs in 0.4.1:

- `SuperstateExt::same_state` uses an internal unsafe discriminant
  `transmute`; the project would inherit that external unsafe path for
  hierarchy transitions.
- `Inner::transition` passes `(target, source)` to `before_transition` even
  though the public trait documents `(source, target)`. It passes the expected
  old/new order to `after_transition`. The local candidate includes a
  regression test proving this observed reversal.

## Graph/report limitation to test

A search of the complete published `statig-0.4.1` source for `graphviz`,
`dot` and `mermaid` found no generator API. The word `graph` appears only in
the conceptual README explanation (apart from development lockfile entries).
This is evidence that the crate does not ship first-party graph generation,
not proof that a black-box project adapter cannot generate a useful report.
The spike will test whether all state/event pairs can be enumerated without a
second hand-maintained transition table and record any limitation explicitly.

## Prior project history

`trellis mem search "statig HSM state machine lifecycle FSM"` returned no
matches for this repository. There is no prior selection decision to preserve.
