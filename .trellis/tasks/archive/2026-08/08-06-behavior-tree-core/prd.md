# Build project-native behavior tree core

## Goal

Turn the existing bounded mission IR/runtime into a clear, public,
project-native behavior-tree framework. It must be event driven, replayable,
safe for robot orchestration and easy to inspect without adding a second
scheduler or a configuration language.

## Confirmed facts

- `control-core::mission` already validates bounded trees and executes every
  required composition type through a clock-free, I/O-free reducer.
- It already validates every action's nonzero timeout, but it currently emits
  deadline effects only for explicit `Timeout` wrapper nodes.
- The current API exposes effects and aggregate runtime state but not a
  bounded per-node activation view suitable for a behavior-tree inspector.
- No composition root currently uses the mission runtime, so an additive
  facade can be introduced without migration of deployed callers.

## Requirements

- C1: Add a documented `BehaviorTree*` public facade over the existing
  mission IR/runtime. Preserve `Mission*` compatibility during V1.
- C2: Keep all existing fixed-capacity and validation guarantees; runtime
  construction, event progression and snapshot creation allocate nothing.
- C3: Expose a deterministic bounded snapshot containing the tree state and
  every node's current activation/participation state, command correlation
  where applicable, and the latest terminal outcome when available.
- C4: Make each leaf action's declared finite timeout emit one correlated
  `ArmDeadline` effect, so an external scheduler can return the exact
  `DeadlineElapsed` event. The reducer itself remains clock-free.
- C5: Provide a stable, human-readable static-tree and live-snapshot renderer
  suitable for a command-line tool. Rendering is outside a realtime path.
- C6: Provide a small `btctl` foundation that demonstrates build, validation,
  deterministic render and event-driven progression without hardware access.

## Acceptance criteria

- [x] A code-defined tree using sequence, selector, parallel, guard, timeout,
      retry, loop, cancel scope and subflow validates and renders
      deterministically through the public behavior-tree API.
- [x] Snapshot tests prove exact transitions for dispatch, acknowledgement,
      execution, cancellation, terminal completion and stale events.
- [x] An action leaf emits an action-level deadline effect and a matching
      deadline event produces the correct cancel/terminal behavior.
- [x] The framework preserves resource-conflict and bounded-capacity rejection
      behavior and passes allocation checks after validation.
- [x] `btctl` output is stable, exit-status tested and does not access hardware.

## Scope boundaries

- No YAML/JSON tree parser, graphical editor, plugin ABI, runtime loading or
  untrusted user definitions in V1.
- No Nav2, ROS, IPC, actuator or hardware dependency is added to `control-core`.
- Navigation action binding belongs to the dependent navigation child.
