# Mission DSL and deterministic runtime design

## Ownership and boundaries

`control-core` owns mission graph semantics. `message-model` continues to own
wire-facing action status and command identities. The mission reducer consumes
semantic events and emits typed effects; adapters allocate command IDs, perform
I/O and return correlated events.

## Bounded IR

The builder accepts at most 64 named nodes. Each composite has at most eight
children, each action declares at most eight exclusive resources, and action
and guard registries are fixed-capacity.

Node kinds:

- typed `Action<Input, Output>` erased only after its registered action name,
  resources and nonzero timeout are captured;
- `Sequence`, `Selector`, `ParallelAll`, `ParallelAny`;
- `Timeout`;
- bounded `Retry` and bounded `Loop`;
- `Guard` with true/false children;
- named `CancelScope`;
- named `Subflow` call.

Validation resolves all references first, then rejects duplicate identities,
unknown registry entries, empty composites, graph cycles/unreachable nodes,
missing timeouts and unbounded repetition. For each direct parallel branch it
collects the transitive exclusive-resource set and rejects intersections.

The resulting `MissionIr` owns private nodes and exposes slices/getters only.

## Runtime model

The runtime uses a fixed set of activation slots. An activation ID includes
slot and generation so an event for a reused slot is stale. Entering composite
nodes deterministically creates child activations in declared order.

Action flow:

```text
DispatchRequested(activation)
  -> ActionDispatched(activation, command_id)
  -> ActionAcknowledged(command_id, accepted)
  -> ActionTerminal(command_id, outcome)
```

Cancel flow:

```text
CancelScope / CancelAll / DeadlineElapsed
  -> RequestActionCancel(command_id)
  -> CancelDecision(command_id, accepted)
  -> ActionTerminal(command_id, Cancelled)
```

Cancel acceptance is nonterminal. Slots and parent composites remain active
until terminal action evidence arrives. Wrong activation/command IDs are
typed rejections.

## Composite semantics

- Sequence advances after success and propagates other outcomes.
- Selector tries the next child after failure/rejection/timeout, succeeds on
  the first success, and propagates cancellation.
- ParallelAll succeeds when all children succeed; a failed child cancels live
  siblings and completes only after they terminate.
- ParallelAny succeeds on the first success, cancels live siblings, and
  completes only after they terminate; all failures fail it.
- Timeout cancellation reports `TimedOut` only after descendants terminate.
- Retry repeats retryable terminal failures up to its nonzero bound.
- Loop repeats successful completion up to its nonzero bound.
- Guard waits for a correlated boolean result before activating one branch.
- CancelScope and Subflow pass their child result through.

## Determinism

The reducer reads no clock and generates no random identity. Deadlines arrive
as explicit activation-correlated events. External command IDs become part of
the recorded event log. Stable slot selection, node order and effect order make
replay byte-equivalent.

## Qualification boundary

Host tests validate semantics and allocation bounds. They do not prove remote
action execution, physical cancellation, lease enforcement completion, target
timing or Orange Pi behavior.
