# Lifecycle HSM selection

Decision date: 2026-07-31

## Decision

Use a direct exhaustive enum reducer for the production G2 lifecycle HSM.
Retain `statig` only in the development-only comparison harness as
reproducible selection evidence. No production crate will depend on `statig`.

## Test evidence

Both candidates passed one shared conformance suite:

- eight leaf states and twelve events;
- 96 state/event report rows;
- valid, invalid and duplicate event traces;
- identical stable transition reports and DOT graphs;
- global E-stop from every state except completed shutdown;
- exact `CancelMission`, then `ReleaseResources` ordering when leaving
  `Running`;
- reconstruction of a machine always returns to motion-inhibited `Boot`;
- zero allocations for warmed synchronous initialization/dispatch paths.

The representative workload produced the same checksum for both candidates:

```text
8439712902066387584
```

## Host size evidence

Command: `tools/hsm-spike/measure.sh`

```text
measurement_scope=host_research
rustc=1.97.0 (2d8144b78 2026-07-07)
host=x86_64-unknown-linux-gnu
profile=workspace release (thin LTO, panic abort, debuginfo stripped)

candidate     file bytes   text     data    bss    text+data+bss  source lines
direct enum      430576   322254   12216   3370       337840          108
statig           432224   323062   12296   2474       337832          343
```

The runtime static-size totals are effectively tied. The direct candidate has
1,648 fewer file bytes and 808 fewer text bytes; size is supporting evidence,
not the deciding safety argument.

## Feature and toolchain evidence

`cargo tree --locked -p hsm-spike -e features` showed `statig 0.4.1` with no
optional features or transitive dependencies. There is no async, Serde, Bevy,
executor, futures, Tokio or proc-macro path in the final harness.

An initial macro-enabled run pulled `statig_macro` and
`proc-macro-error2 2.0.1`. Rust 1.97 reports the latter as
future-incompatible because it publicly re-exports a private `proc_macro`
extern crate. The candidate was rewritten against the no-macro core before
final CI, removing the warning rather than accepting it.

Published-source inspection and a local regression test also found that
`statig` 0.4.1 calls `before_transition` with new/old states despite documenting
source/target order; `after_transition` observes old/new. Its hierarchy
comparison also contains an internal unsafe discriminant transmute. Neither
issue changed the tested state/effect projections, but both increase the audit
surface of production transition observation.

## Comparison matrix

| Criterion | Direct enum | statig | Result |
|---|---|---|---|
| Heap allocation | zero | zero | tie |
| Synchronous cancellation | explicit ordered batch | ordered exit/entry actions | tie |
| Hierarchy expression | manual grouping in exhaustive reducer | native superstate delegation | `statig` advantage |
| Transition visibility | one exhaustive state/event match | spread across eight handlers plus superstate | direct advantage |
| Graph/report | shared black-box generator works; pure reducer remains project-owned | shared generator works, but crate has no native generator and its before hook reverses documented arguments | direct advantage |
| Arbitrary-state testing | project can expose a test-only constructor | safe API starts only at initial state; mutation API is unsafe | direct advantage |
| Event coverage | adding state/event breaks exhaustive match | exhaustive matches can enforce coverage, but across handlers | direct advantage |
| Async leakage | none and no async facility | async feature disabled; no executor leakage | pass both |
| Release size | smaller file/text in this fixture | 1,648-byte larger file | slight direct advantage |
| Source/adapter size | 108 lines | 343 lines for no-macro traits/adapter and hook regression | direct advantage |
| Dependency/toolchain risk | no new dependency/unsafe path | core has no transitive dependencies, but hierarchy comparison contains external unsafe; optional macro path is future-incompatible | direct advantage |

## Rationale

`statig` successfully proves that its synchronous hierarchy and entry/exit
mechanics can satisfy the spike. It is rejected for production G2 because the
current top-level lifecycle does not gain enough from its one shared
superstate to offset:

1. transitions split across handlers/traits rather than one reviewable table;
2. no first-party graph/transition-table export;
3. the documented before-transition observation order is reversed in 0.4.1;
4. safe arbitrary-state inspection requiring reachability replay rather than
   direct construction;
5. substantially more source for the no-macro core path, while the optional
   macro path has a pinned-compiler future-incompatibility;
6. an external unsafe hierarchy-comparison path that the direct reducer does
   not require.

The direct reducer keeps state/event coverage, effect order and graph
projection under project ownership. G2 should preserve these requirements:
exhaustive matches without wildcard arms, fixed-capacity typed effects,
serialized dispatch, graph/report generation from the same reducer, and
restart construction that cannot restore an active state.

## Qualification boundary

This is host selection evidence only. It does not implement G2 policy, physical
E-stop actuation, process supervision, Orange Pi behavior or realtime timing.
