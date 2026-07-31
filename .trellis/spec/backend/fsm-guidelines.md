# Rust Lifecycle FSM

## Scenario: Implement or change the robot lifecycle HSM

### 1. Scope / Trigger

Use this contract when adding a lifecycle state/event, guard, transition,
entry/exit effect, state graph, restart behavior or mission-cancellation
boundary in `application_module`. Mission DSL nodes remain a separate reducer
owned by the legal `Running` state.

The G1 host spike selected a direct exhaustive enum reducer over `statig`
0.4.1. Both candidates were synchronous and allocation-free, but the direct
reducer kept transitions and graph projection project-owned, used less source
and text size, allowed safe arbitrary-state testing and avoided both the
core hierarchy's external unsafe comparison path and an optional proc-macro
dependency that Rust 1.97 reports as future-incompatible. The no-macro core
was the final measured candidate.

### 2. Signatures

The production G2 contract should retain this shape:

```rust
LifecycleMachine::new() -> LifecycleMachine
LifecycleMachine::startup_projection(&self) -> LifecycleProjection
LifecycleMachine::dispatch(event: LifecycleEvent) -> LifecycleProjection
LifecycleMachine::state(&self) -> LifecycleState
lifecycle_transition_report() -> String
lifecycle_dot_graph() -> String
```

`LifecycleState`, `LifecycleEvent`, `LifecycleEffect` and
`LifecycleProjection` are application-owned semantic types. The exact G2
event set may expand from the spike, but every added variant follows the same
coverage and restart rules.

### 3. Contracts

- Production lifecycle code uses one direct enum reducer with exhaustive
  `match (state, event)` handling. Do not use wildcard arms that let a new
  event compile as an implicit no-op, and do not link `statig` into a
  production crate.
- Construction always starts in `Boot` and returns typed motion-inhibit
  evidence. It accepts no serialized active state, checkpoint or prior
  mission as restoration input.
- Events are dispatched synchronously by one serialized owner. One ordered
  event history yields one deterministic sequence of states and typed effects.
- `EmergencyStop` preempts every state except completed `Shutdown`. E-stop is
  latched until the explicit physical/software reset contract is satisfied.
- Any transition leaving `Running` synchronously emits `CancelMission` before
  `ReleaseResources`; safe-state/motion-inhibit effects follow them. Publishing
  an event or requesting cancellation is not proof that the effect completed.
- Transition effects use fixed-capacity storage on hot paths. Dispatch performs
  no heap allocation, blocking, logging, serialization, clock read or I/O.
- The checked transition report and DOT graph are generated from the same
  reducer across explicit arrays of every state and event. A second
  hand-maintained edge table is not authoritative.
- `statig` may remain only in the development comparison harness. Reconsidering
  the selection requires a new measured spike and explicit spec update, not a
  second production implementation.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| New state/event omitted from reducer | compile failure from exhaustive match |
| Event invalid in current state | explicit deterministic no-op/rejection projection |
| E-stop in any non-shutdown state | transition to latched `EStop`; cancel/release first when source is `Running` |
| E-stop after completed shutdown | remain `Shutdown` |
| Transition leaves `Running` | `CancelMission`, then `ReleaseResources`, then any inhibit effect |
| Process restart or stale checkpoint | construct fresh motion-inhibited `Boot`; never resume |
| Reconstruction missing/conflicting/timeout | remain inhibited and enter the defined G2 `Fault` path |
| Graph/report differs across identical builds | fail deterministic artifact test |
| Dispatch allocates or waits | fail the hot-path quality gate |
| Production Cargo graph contains `statig` | fail architecture/review gate |

### 5. Good / Base / Bad Cases

- Good: a `Running` E-stop event produces ordered cancel/release/inhibit
  effects and enters latched `EStop`; a restarted process independently begins
  in inhibited `Boot`.
- Base: a duplicate `Arm` or irrelevant recovery event is represented by an
  explicit state/event arm and leaves state/effects unchanged.
- Bad: `_ => Handled`, deserialize `Running`, emit release before cancellation,
  infer an edge from a separate diagram, perform cancellation asynchronously
  after dispatch returns, or keep direct and `statig` production paths.

### 6. Tests Required

- Iterate every state/event pair and assert an explicit projection, complete
  report row and safe exit reachability.
- Assert global E-stop from every state, completed-shutdown behavior and exact
  `Running` exit-effect ordering.
- Cover good, invalid, duplicate, reordered, stale, timeout, fault,
  reconstruction and restart traces. Replaying the same ordered events must
  produce byte-identical projections.
- Generate report/DOT twice and compare bytes. Assert critical edges and that
  row count equals `states * events`.
- Warm construction/dispatch and measure zero allocations on all transition
  classes.
- Run warnings-denied Clippy, architecture policy and `tools/ci.sh`. Host
  success does not qualify physical E-stop or target timing.

### 7. Wrong vs Correct

#### Wrong

```rust
match event {
    LifecycleEvent::EmergencyStop => spawn_cancel_and_enter_estop(),
    _ => {}
}
```

#### Correct

```rust
let target = transition(self.state, event); // exhaustive state/event match
let effects = ordered_effects(self.state, target);
// Running exit: CancelMission, ReleaseResources, then MotionInhibited.
self.state = target;
LifecycleProjection { state: target, effects }
```
