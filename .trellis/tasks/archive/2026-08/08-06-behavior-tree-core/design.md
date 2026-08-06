# Behavior-tree core — design

## API shape

`control-core::behavior_tree` exposes a thin additive facade over the existing
`mission` implementation:

```text
BehaviorTreeBuilder -> validated BehaviorTree
BehaviorTreeRuntime::advance(BehaviorTreeEvent) -> BehaviorTreeStep
BehaviorTreeRuntime::snapshot() -> BehaviorTreeSnapshot
BehaviorTree::render_tree(writer)
```

The facade reuses the existing node, action, guard, effect, outcome and
activation identities. It does not translate to dynamic trait objects or
allocate executable leaf callbacks. Registered action and guard identities
remain startup-time validation inputs; adapters execute effects outside the
reducer.

## Snapshot model

`BehaviorTreeSnapshot` uses an `ArrayVec` bounded by the existing maximum node
count. Each entry contains stable node ID/name, static node kind, current
execution state, optional activation generation, optional command ID and last
terminal outcome. States distinguish inactive, waiting for a guard/children/
dispatch/acknowledgement, running, cancel-requested, canceling and terminal.

The runtime retains only the latest terminal outcome per node in a fixed array.
Reactivation clears the old terminal outcome. Thus a retry visibly changes a
failed leaf back to live rather than retaining a misleading old result.

## Deadline behavior

When an `Action` activation enters, the reducer emits `DispatchAction` and an
`ArmDeadline` for that action's declared timeout. The embedding scheduler
records the activation/deadline externally and later calls `advance` with the
exact `DeadlineElapsed` event. Wrapper timeouts retain their present behavior.
No clock is introduced into the core.

## CLI foundation

The new `btctl` composition-root application depends only on `control-core`
and `clap`. Its initial `example` command builds a checked navigation-shaped
tree from registered abstract leaf names, prints the static hierarchy and
shows deterministic state transitions. It has no IPC or hardware access in
this child; the navigation child adds its read-only live-inspection command.

## Tests and rollback

New unit tests cover snapshots, action deadline scheduling, timeout/cancel
ordering and rendering. Existing mission tests remain the compatibility suite.
Removing the facade/CLI leaves the original mission API untouched.
