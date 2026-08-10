# Behavior-tree navigation participation — design

## Action lifecycle bridge

The existing `robot.v1.ActionAck` schema is mapped into a semantic bounded
acknowledgement value and a new `NavigationActionResponseKind::Acknowledgement`
variant. The bridge publishes it immediately after Nav2 accepts or rejects the
correlated ROS goal. A rejected acknowledgement is sufficient to complete the
outer leaf as rejected; an accepted acknowledgement permits later feedback,
cancel decisions and a terminal result.

```text
tree DispatchAction
  -> app successfully publishes request
  -> ActionDispatched(command_id)
  -> bridge Acknowledgement(accepted/rejected)
  -> ActionAcknowledged(command_id, accepted)
  -> feedback / cancel decision / terminal result
```

This fixes the present invalid case where a goal that aborts before its first
feedback cannot legally advance an outer action from awaiting acceptance.

## Participant snapshot

`navigation-ipc` gains an additive latest channel with a fixed payload and
schema ID. The snapshot contains a validated header, harness process epoch,
optional active command ID and at most sixteen entries. Entry IDs and state/
reason enums are bounded and machine-readable; no arbitrary log strings are
used in the shared-memory value.

The harness emits at a low fixed observation cadence and immediately on state
transition. It is a best-effort copy after the safety decision, never an input
to the decision. The default view includes:

```text
tree.<node>                 outer tree activation state
navigation.action           request/ack/result boundary
navigation.bridge           health/peer observation
control.authority           active or unavailable lease assertion
control.proposal            current, expired or peer-lost proposal
control.final_limiter       constrained body command or stopped
control.mecanum             projection result or stopped
output.fake_sink            motor-disabled final observation
```

## Inspection CLI

`btctl inspect --ipc-root /absolute/path` attaches only as a latest subscriber
and prints a stable nested tree followed by the downstream stages. It checks
header freshness and exits nonzero for missing, stale or incompatible layouts.
It does not create layouts, publish, grant a lease or open any hardware
transport.

## Manifest and process boundary

The shared navigation manifest declares the additive status channel and its
layout policy. Root and peer copies must remain byte-identical and their schema
and contract tests must be updated together. `InitIpc` creates its layout with
the existing namespace. No Nav2 launch parameter or behavior-tree file changes
are required.

## Failure semantics

Snapshot publication failure is diagnosed only by absence/staleness to the CLI.
Bridge loss, expired proposal and lost authority continue to take their existing
local fail-closed paths; the snapshot records the typed reason after that
decision. It cannot suppress, delay or reinterpret a stop.
