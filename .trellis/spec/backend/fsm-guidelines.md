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
LifecycleMachine::new(boot_id: Id128) -> LifecycleMachine
LifecycleMachine::startup_projection(&self) -> LifecycleProjection
LifecycleMachine::dispatch(event: LifecycleEvent) -> LifecycleProjection
LifecycleMachine::state(&self) -> LifecycleState
lifecycle_transition_report() -> String
lifecycle_dot_graph() -> String
```

`LifecycleState`, `LifecycleEvent`, `LifecycleEffect` and
`LifecycleProjection` are application-owned semantic types. The process
composition root supplies a new nonzero `Id128` on every start. Live-health
reconstruction and operator-authority events carry that identity; Arm and
operator commands also carry strictly increasing command sequences.

### 3. Contracts

- Production lifecycle code uses one direct enum reducer with exhaustive
  `match (state, event)` handling. Do not use wildcard arms that let a new
  event compile as an implicit no-op, and do not link `statig` into a
  production crate.
- Construction always starts in `Boot` and returns typed motion-inhibit
  evidence. It accepts only the newly generated process `boot_id`, never a
  serialized active state, checkpoint or prior mission as restoration input.
- Live-health reconstruction progress and completion must match the current
  process `boot_id` and strictly increase the reconstruction revision.
  `Ready` reaches inhibited `Standby`; missing, conflicting, timed-out or
  faulted completion reaches inhibited `Fault`. No completion leaves the
  machine inhibited in `Boot`.
- Arm must match the current `boot_id` and strictly advance the operator
  command sequence. Start must additionally reference the currently accepted
  Arm sequence. Fault clear and E-stop reset use the same current-boot,
  increasing-command contract. Every fault occurrence advances a reducer-owned
  generation that its clear event must match.
- Events are dispatched synchronously by one serialized owner. One ordered
  event history yields one deterministic sequence of states and typed effects.
- `EmergencyStop` preempts every state except completed `Shutdown`. E-stop is
  latched until release is observed and a later explicit current-boot software
  reset is accepted. Every assertion advances a reducer-owned generation;
  release and reset must match it. Reassertion clears prior release observation
  so a delayed old release cannot clear a newer assertion.
- E-stop reset preserves prerequisites: it returns to `Boot` if assertion
  interrupted reconstruction, to `Fault` if a fault was preempted or observed
  while stopped, and otherwise to `Standby`. It never clears fault or
  reconstruction state implicitly.
- Any transition leaving `Running` synchronously emits `CancelMission` before
  `ReleaseResources`; safe-state/motion-inhibit effects follow them. Publishing
  an event or requesting cancellation is not proof that the effect completed.
- Capability loss leaves `Running`, cancels the mission and enters inhibited
  `Degraded`. Capability recovery reaches `Armed`, never `Running`; a new
  explicit Start is required and the cancelled mission is not restored.
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
| E-stop reset before release observation | remain inhibited and latched in `EStop` |
| E-stop release/reset for an old assertion generation | typed rejection; remain inhibited and latched |
| E-stop interrupted Boot or Fault | reset returns to inhibited Boot or Fault, not Standby |
| Fault clear for an old fault generation | typed rejection; remain inhibited in Fault |
| Transition leaves `Running` | `CancelMission`, then `ReleaseResources`, then any inhibit effect |
| Process restart or stale checkpoint | construct fresh motion-inhibited `Boot`; never resume |
| Prior-boot reconstruction/Arm/Start/clear/reset | typed `StaleBoot` rejection; no authority change |
| Duplicate/reordered reconstruction or operator sequence | typed `StaleSequence` rejection |
| Start does not reference current Arm | reject; remain out of `Running` |
| Reconstruction missing/conflicting/timeout | remain inhibited and enter the defined G2 `Fault` path |
| Graph/report differs across identical builds | fail deterministic artifact test |
| Dispatch allocates or waits | fail the hot-path quality gate |
| Production Cargo graph contains `statig` | fail architecture/review gate |

### 5. Good / Base / Bad Cases

- Good: a `Running` E-stop event produces ordered cancel/release/inhibit
  effects and enters latched `EStop`; release observation plus a current-boot
  reset returns to inhibited `Standby`; a restarted process independently
  begins in inhibited `Boot`.
- Base: a duplicate `Arm` or irrelevant recovery event is represented by an
  explicit state/event arm and leaves state/effects unchanged.
- Bad: `_ => Handled`, deserialize `Running`, emit release before cancellation,
  accept an old `boot_id`, reset E-stop without release observation, infer an
  edge from a separate diagram, perform cancellation asynchronously after
  dispatch returns, or keep direct and `statig` production paths.

### 6. Tests Required

- Iterate every state/event pair and assert an explicit projection, complete
  report row and safe exit reachability.
- Assert global E-stop from every state, completed-shutdown behavior and exact
  `Running` exit-effect ordering.
- Cover good, invalid, duplicate, reordered, stale-boot, timeout, fault,
  reconstruction and restart traces. Restart from every leaf state must return
  to inhibited `Boot`; replaying the same ordered events must produce
  byte-identical projections.
- Assert Start requires the current Arm sequence, capability recovery does not
  resume a cancelled mission, and E-stop reset requires release observation
  plus current boot/assertion generation. Assert E-stop reset preserves Boot
  reconstruction and Fault clearing prerequisites.
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
