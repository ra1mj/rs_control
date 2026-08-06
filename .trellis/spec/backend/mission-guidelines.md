# Rust Mission IR and Runtime

## Scenario: Implement or change automatic mission orchestration

### 1. Scope / Trigger

Use this contract for mission DSL nodes, graph validation, action activation,
retry/loop/timeout behavior, guard branching, parallel work, action
correlation, cancellation, or the public behavior-tree facade. The mission
runtime is owned by `application_module` and may run only while the lifecycle
machine legally permits `Running`.

### 2. Signatures

```rust
MissionBuilder::validate(self, registry: &MissionRegistry)
    -> Result<MissionIr, MissionBuildError>
MissionRuntime::new(ir: &MissionIr) -> MissionRuntime
MissionRuntime::dispatch(event: MissionEvent) -> MissionStep
BehaviorTreeRuntime::advance(event: BehaviorTreeEvent) -> BehaviorTreeStep
BehaviorTreeRuntime::snapshot() -> BehaviorTreeSnapshot
BehaviorTree::render_tree() -> String
representative_automatic_project() -> Result<MissionIr, MissionBuildError>
```

`MissionIr` has no public mutation API. Typed `Action<Input, Output>` markers
are erased only after action identity, exclusive resources and timeout are
captured in the bounded IR.

### 3. Contracts

- Supported nodes are Action, Sequence, Selector, ParallelAll, ParallelAny,
  Timeout, Retry, Loop, Guard, CancelScope and named Subflow.
- Builders, child/resource lists, registries, runtime slots, command history
  and effect batches have fixed maximum capacities. Action definitions may use
  bounded startup-time `Box` indirection to keep node enums compact; runtime
  construction and dispatch allocate nothing.
- Validation resolves every reference and rejects duplicate IDs/names,
  unknown actions/guards, empty composites, duplicate direct children or
  resources, unreachable nodes and graph cycles.
- Every action has a nonzero timeout. Retry and loop counts are present,
  nonzero and finite. Validation computes worst-case action invocations and
  simultaneous activation frames and rejects graphs over runtime capacity.
- Entering every Action emits exactly one `ArmDeadline` effect with that
  activation ID and the action's declared timeout. An explicit `Timeout`
  wrapper retains its own deadline effect; the embedding scheduler returns the
  exact matching `DeadlineElapsed` event for either one.
- Parallel validation collects exclusive resources transitively per direct
  branch and rejects any intersection before runtime.
- One serialized reducer owns runtime state. It reads no clock, performs no
  I/O and generates no random identity. Deadlines and externally allocated
  command IDs return as recorded events, making replay deterministic.
- Activation IDs include slot generation. Action dispatch, goal
  acknowledgement, cancel decision, terminal result, guard result and deadline
  events require exact active correlation. Command IDs cannot be reused within
  a run.
- A cancel request emitted before command binding/goal acceptance is retained
  as intent. The reducer waits for `ActionDispatched` and goal acceptance before
  emitting the protocol cancel request, preventing orphaned actions and invalid
  action-lifecycle order.
- Cancel acceptance is nonterminal. Timeout, scope/global cancellation and
  parallel sibling cancellation retain activation state until correlated
  terminal action evidence arrives. Cancellation rejection emits explicit
  evidence and cannot be reported as successful cancellation.
- `ReleaseActionResources` is emitted only after goal rejection or terminal
  action evidence, and before parent advancement or mission completion.
- Sequence advances on success. Selector tries later children after retryable
  failure. ParallelAll/Any cancel live siblings after their outcome is known
  and wait for terminal evidence. Capability wrappers never create a second
  executor.
- Leaving lifecycle `Running` must synchronously deliver `CancelAll`; process
  restart constructs a new runtime and never restores active slots or a prior
  mission checkpoint.
- `BehaviorTree*` names are an additive facade over the same immutable IR and
  serialized reducer. They must not add a second scheduler, dynamic callback
  executor, clock read, I/O path or runtime-loaded tree format.
- Snapshots use fixed-capacity storage and allocate nothing. They contain every
  static node's participation/latest outcome plus every live exact activation,
  including phase, parent and command ID where applicable. Reactivating a node
  clears its prior retained terminal outcome. Text renderers are deterministic
  non-realtime diagnostics.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Unknown/unreachable/cyclic node | typed build rejection |
| Empty composite or duplicate child | typed build rejection |
| Action missing timeout | typed build rejection |
| Action activation enters | one correlated `ArmDeadline` using its declared timeout |
| Retry/loop absent, zero or exceeds total run budget | typed build rejection |
| Transitive parallel resource overlap | typed build rejection naming resource |
| Worst-case active/invocation budget exceeds 64/256 | typed build rejection |
| Old activation generation | `StaleActivation`; no state advance |
| Reused command ID | `DuplicateCommand`; no state advance |
| Wrong command/phase/status | typed event rejection |
| Cancel before command/goal acceptance | retain intent; wait, then request cancel |
| Cancel accepted without terminal result | remain Running/Canceling |
| Cancel rejected | emit `CancellationRejected`; never claim Cancelled |
| Timeout followed by another cancel source | preserve TimedOut outcome |
| Action terminal evidence | release resources before advancing parent |
| Node is reactivated after a terminal outcome | clear that node's latest retained outcome |
| Snapshot is requested | bounded node/activation view with no reducer state change or allocation |
| Runtime defensive capacity/invariant failure | fail closed with `RuntimeFault` |

### 5. Good / Base / Bad Cases

- Good: an action dispatch is bound to one command ID, accepted, later
  cancelled, and releases resources only after a correlated terminal Cancelled
  result.
- Good: an action entry emits its own finite deadline; a deadline received
  before goal acceptance is retained as timeout intent and produces a cancel
  request once the goal is accepted.
- Base: a selector observes a failed child and deterministically activates the
  next child; a duplicate old activation event is rejected.
- Bad: deserialize active runtime state, generate random IDs inside dispatch,
  omit the action-level deadline while merely validating its timeout, infer
  terminal cancellation from cancel acceptance, release resources when a
  cancel request is published, use an unbounded retry, or allow parallel
  branches to share an exclusive resource.

### 6. Tests Required

- Cover every validator rejection plus one representative accepted graph.
- Test Sequence, Selector, Guard, ParallelAll, ParallelAny, Timeout, Retry,
  Loop, CancelScope and Subflow propagation.
- Test command binding, goal rejection, nonterminal cancel acceptance,
  cancellation rejection and terminal correlation.
- Test scope isolation, global cancellation, pre-binding cancellation,
  timeout/cancel ordering, stale activation generations and command reuse.
- Assert every action entry produces exactly one correlated deadline and that
  its deadline preserves `TimedOut` through asynchronous cancel completion.
- Assert snapshot transitions for dispatch, acknowledgement, running,
  cancellation, terminal outcomes and stale events; measure zero allocations
  for snapshot creation after validation.
- Test static tree and live snapshot render output for deterministic bytes; CLI
  tests assert status, stdout and stderr without a hardware boundary.
- Replay the same event log and compare complete ordered effect batches.
- Assert resource release precedes next dispatch/mission completion.
- Measure zero allocations across runtime construction and dispatch after IR
  validation.
- Run warnings-denied Clippy, architecture policy and `tools/ci.sh`.

### 7. Wrong vs Correct

#### Wrong

```rust
// Validation alone does not make a leaf timeout enforceable.
activation.phase = WaitingDispatch;
step.push(MissionEffect::DispatchAction { activation, node_id, action });
```

#### Correct

```rust
activation.phase = WaitingDispatch;
step.push(MissionEffect::DispatchAction { activation, node_id, action });
step.push(MissionEffect::ArmDeadline {
    activation,
    timeout_ns: action.timeout_ns(),
});
```
