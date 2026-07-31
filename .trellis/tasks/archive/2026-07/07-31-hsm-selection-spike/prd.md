# HSM selection spike

## Goal

Select one synchronous Rust implementation strategy for the robot lifecycle
HSM by comparing `statig` with a minimal direct-enum reducer under the same
host-only safety scenario. Preserve reproducible evidence for the decision
without making either prototype a hardware or production-runtime integration.

## Background

The parent design requires compiled Rust states/events, deterministic
serialized handling, a globally preemptive E-stop, synchronous cancellation
when leaving `Running`, bounded resources, graph/transition visibility and
restart into motion-inhibited `Boot`. Parent G1 explicitly requires a
`statig` prototype and a direct-enum reference before G2 implements the
production lifecycle.

Repository and upstream evidence establish:

- `control-core` is the application-layer owner and already uses
  fixed-capacity, allocation-tested reducers.
- `statig` 0.4.1 supports synchronous hierarchical state machines,
  state-local data, entry/exit actions, context and transition callbacks. Its
  macro feature is optional; async support is a separate feature.
- `statig` does not expose a first-party transition-table or graph generator;
  graph visibility must therefore be evaluated rather than assumed.
- No earlier project session records a `statig` versus direct-enum decision.

## Requirements

- **HSS-01:** Add a development-only comparison harness. It must not be linked
  by `control-core`, an app, middleware, driver or target image.
- **HSS-02:** Implement the same leaf states in both candidates: `Boot`,
  `Standby`, `Armed`, `Running`, `Degraded`, `Fault`, `EStop` and `Shutdown`.
- **HSS-03:** Implement the same event scenario in both candidates, including
  boot completion, arm/disarm, run/stop, degradation/recovery, fault,
  E-stop/reset and shutdown.
- **HSS-04:** Both candidates must make E-stop preempt every non-shutdown
  state and must synchronously emit `CancelMission` before `ReleaseResources`
  whenever a transition leaves `Running`.
- **HSS-05:** The prototypes must start in motion-inhibited `Boot`, reach
  `Standby` only through reconstructed-ready input, and require a fresh arm
  event before `Running`. Reconstructing either candidate must not restore a
  prior active state.
- **HSS-06:** Feed identical event traces to both implementations and assert
  identical state/effect projections for valid, invalid, duplicate and
  preemptive inputs.
- **HSS-07:** Measure zero heap allocations after warm-up for synchronous
  dispatch, including hierarchy/global-event and `Running` cancellation paths.
- **HSS-08:** Compare release binary text/data size on the pinned host
  toolchain and inspect enabled Cargo features/dependencies for async or
  executor leakage.
- **HSS-09:** Compare transition visibility, deterministic DOT/report
  generation, hierarchy expression, cancellation ordering and test
  ergonomics using explicit evidence rather than preference.
- **HSS-10:** Record a scored decision with rejection reasons and select
  exactly one strategy for production G2. Keeping both candidates inside the
  explicitly development-only spike harness is allowed for reproducibility;
  production crates must depend on neither until G2 implements the winner.

## Acceptance Criteria

- [x] Both candidates compile and pass the same conformance vectors.
- [x] Tests prove global E-stop, exact `Running` exit-effect order, invalid
      event no-op semantics, restart inhibition and deterministic replay.
- [x] Allocation instrumentation observes zero hot-path allocations for both.
- [x] A repeatable release build records each candidate's file/text/data size,
      Rust target/toolchain and enabled feature tree.
- [x] A stable transition report and DOT graph are generated or the candidate
      receives an evidence-backed graph-visibility limitation.
- [x] The decision report selects one candidate across allocation, size,
      transition visibility, graph generation, async leakage, cancellation and
      test ergonomics.
- [x] `cargo run --locked -p architecture-check -- check` and `tools/ci.sh`
      pass.
- [x] Only parent G1 is marked complete. G2 lifecycle production code,
      Orange Pi, target timing and hardware gates remain open.

## Out of Scope

- Production lifecycle policy, full state/event table or G2 implementation.
- Mission DSL/runtime, lease-grant priority or service supervision.
- Physical E-stop actuation, motor enable/control, systemd integration,
  Orange Pi deployment or realtime qualification.
- Selecting a library from popularity alone or enabling async execution.

## Traceability

- Parent plan: `.trellis/tasks/07-29-rust-robot-framework/implement.md` G1.
- Parent requirements: R49-R56 and R84-R88 in the parent `prd.md`.
- Parent design: sections 9.1 and 9.3 in the parent `design.md`.
