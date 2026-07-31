# HSM selection spike design

## Comparison boundary

Add `tools/hsm-spike`, classified as a development tool by the architecture
policy. It contains two modules behind one shared fixture contract:

```text
shared State/Event/Effect + conformance traces
             |                    |
     direct_enum reducer     statig adapter
             |                    |
             +-- identical projections
             +-- allocation tests
             +-- separate release binaries
             +-- transition/graph evidence
```

No production crate depends on the tool or on `statig`. The harness may retain
both prototypes after selection so the measurements remain reproducible; G2
will implement only the selected strategy in `application_module`.

## Shared semantic fixture

The comparison uses:

```rust
enum LifecycleState {
    Boot, Standby, Armed, Running, Degraded, Fault, EStop, Shutdown
}

enum LifecycleEvent {
    ReconstructionReady,
    Arm,
    Start,
    Stop,
    Disarm,
    CapabilityLost,
    CapabilityRestored,
    FaultDetected,
    ClearFault,
    EmergencyStop,
    ResetEmergencyStop,
    ShutdownRequested,
}

enum LifecycleEffect {
    MotionInhibited,
    CancelMission,
    ReleaseResources,
}
```

Each dispatch returns an observable state and fixed-capacity ordered effects.
Invalid or duplicate events are handled deterministically as no-ops. Creation
always yields `Boot` plus motion-inhibit state; serialized event history is
never accepted as a restoration input.

`EmergencyStop` is a shared/global transition for every state except
`Shutdown`. Any source `Running` transition first emits `CancelMission`, then
`ReleaseResources`, including E-stop, fault, degradation, stop and shutdown.
The spike tests transition mechanics, not the final G2 policy table.

## Candidate A: direct enum

Use exhaustive `match (state, event)` logic and fixed-capacity effects. Arrays
of all states and events drive a transition report and DOT projection from the
same reducer, so adding an enum variant creates a compile error until the
match is updated. No macro, executor or state-machine dependency is required.

## Candidate B: statig

Pin `statig = "=0.4.1"` with default features disabled and no optional
features. Implement its synchronous `IntoStateMachine`, `State` and
`Superstate` traits directly, including a superstate for global
E-stop/shutdown and entry/exit actions for bounded effects. Do not enable the
macro, async, Serde or Bevy paths.

The macro path was tested first but pulled `proc-macro-error2 2.0.1`, which
Rust 1.97 marks future-incompatible. Removing the optional macro makes the
comparison fair to the dependency's synchronous core and keeps the final
workspace warning-free.

Because the traits do not expose a static edge table, graph/report generation
is evaluated through black-box enumeration of all state/event pairs. Any
inability to initialize arbitrary states or distinguish guarded paths without
mirrored metadata is recorded as a limitation rather than hidden with a
second hand-maintained table.

## Measurement

- Warm both machines before `allocation_counter::measure`.
- Execute the same fixed trace repeatedly and require zero total allocations.
- Build two minimal release binaries with the workspace release profile.
- Record `rustc -Vv`, target, Cargo feature tree, file bytes and platform
  `size` text/data/bss output.
- Generate candidate transition reports twice and compare bytes.
- Score each candidate as pass/fail plus evidence for:
  allocation, binary size, transition visibility, graph generation, async
  leakage, cancellation ordering, hierarchy and test ergonomics.

Safety semantics and inspectability are mandatory; binary size is a
tiebreaker, not permission to weaken explicit transition coverage.

## Decision and rollback

The decision lives in task research and is promoted into the FSM spec. Parent
G1 closes only when one strategy is named and the other has explicit
rejection/trade-off reasons.

Rollback removes the tool member, architecture-policy entry, `statig`
workspace dependency and lockfile additions. It does not change
`control-core`, messages, middleware, apps, target scripts or hardware state.
