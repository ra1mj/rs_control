# Rust Mission IR and Runtime

## Scenario: Implement or change automatic mission orchestration

### 1. Scope / Trigger

Use this contract for mission DSL nodes, graph validation, action activation,
retry/loop/timeout behavior, guard branching, parallel work, action
correlation or cancellation. The mission runtime is owned by
`application_module` and may run only while the lifecycle machine legally
permits `Running`.

### 2. Signatures

```rust
MissionBuilder::validate(self, registry: &MissionRegistry)
    -> Result<MissionIr, MissionBuildError>
MissionRuntime::new(ir: &MissionIr) -> MissionRuntime
MissionRuntime::dispatch(event: MissionEvent) -> MissionStep
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

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Unknown/unreachable/cyclic node | typed build rejection |
| Empty composite or duplicate child | typed build rejection |
| Action missing timeout | typed build rejection |
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
| Runtime defensive capacity/invariant failure | fail closed with `RuntimeFault` |

### 5. Good / Base / Bad Cases

- Good: an action dispatch is bound to one command ID, accepted, later
  cancelled, and releases resources only after a correlated terminal Cancelled
  result.
- Base: a selector observes a failed child and deterministically activates the
  next child; a duplicate old activation event is rejected.
- Bad: deserialize active runtime state, generate random IDs inside dispatch,
  infer terminal cancellation from cancel acceptance, release resources when a
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
- Replay the same event log and compare complete ordered effect batches.
- Assert resource release precedes next dispatch/mission completion.
- Measure zero allocations across runtime construction and dispatch after IR
  validation.
- Run warnings-denied Clippy, architecture policy and `tools/ci.sh`.
