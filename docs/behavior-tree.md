# Project-native behavior tree

`control-core::behavior_tree` is the public behavior-tree facade for robot
orchestration. It reuses the immutable, bounded mission IR and the one
serialized reducer; it is not a second executor.

Trees are defined in Rust and validated during startup. V1 intentionally has
no YAML/JSON parser, runtime plugin ABI, graphical editor, or untrusted tree
loading. A tree uses fixed capacities for nodes, activations, action command
history and reducer effect batches.

## Event-driven execution

`BehaviorTreeRuntime` does not read a clock, perform I/O, allocate command
identities, or invoke actuator callbacks. It returns effects for an embedding
owner to execute and accepts only correlated events in return:

- `DispatchAction` is followed by an externally allocated `ActionDispatched`
  event and then an `ActionAcknowledged` event.
- Every action and explicit timeout wrapper emits `ArmDeadline`. The external
  scheduler later delivers the matching `DeadlineElapsed` event.
- Guard, cancellation and terminal action evidence follow the same exact
  activation or command correlation rules.

This makes replay deterministic and prevents the behavior tree from becoming
a high-rate control loop.

## Inspecting participation

`BehaviorTreeRuntime::snapshot()` returns fixed-capacity static node entries
and exact live activation records without allocating. Node entries show whether
the node is inactive, active or terminal; live records provide activation
generation, command identity when bound, parent activation and precise phase.
The latest terminal outcome is cleared when that node is activated again.

`BehaviorTree::render_tree()` and `BehaviorTreeSnapshot::render()` provide
stable human-readable views for non-realtime tools.

Run the self-contained demonstration with:

```text
cargo run -p btctl -- example
```

It constructs a validated abstract navigation-shaped tree and replays startup
and route-guard events only. It opens no ROS, IPC, actuator or hardware
boundary. Nav2's internal behavior tree remains out of scope; a later adapter
will translate external navigation action evidence into this runtime.
