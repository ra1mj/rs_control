# Project behavior-tree framework and motion participants

## Goal

Complete a project-native behavior-tree framework that can orchestrate robot
work safely and deterministically, starting with a chassis-navigation leaf.
It must make each active motion explainable: which outer project-tree nodes,
bridge, authority, control and output stages currently participate, and why
any stage is blocked or stopped.

## Confirmed facts

- `control-core::mission` already provides a bounded, validated tree-shaped
  IR with `Action`, `Sequence`, `Selector`, `ParallelAll`, `ParallelAny`,
  `Timeout`, `Retry`, `Loop`, `Guard`, `CancelScope`, and `Subflow` nodes.
  Its deterministic event reducer already correlates action dispatch,
  acknowledgement, cancellation and terminal results.
- The existing mission engine has a fixed maximum of 64 nodes and 256 action
  invocations. Repository usage is currently limited to its implementation,
  representative example and tests; no application composes it into the
  navigation runtime.
- Every action definition already requires a nonzero timeout, but the current
  reducer emits `ArmDeadline` only for an explicit `Timeout` wrapper. A leaf
  action's own declared timeout is therefore validated but not scheduled; the
  framework delivery must close this gap.
- Nav2 remains a separate non-RT process group. Its enabled inner behavior
  tree is `PipelineSequence(ComputePathToPose, FollowPath)` and is selected
  through a digest-bound launch profile. The Rust bridge permits only the
  semantic identity `navigate_to_pose_fail_closed` and correlates the
  canonical command ID with the ROS action UUID. Its internal BT is explicitly
  deferred and will not be changed or observed by this delivery.
- A controller proposal crosses bounded local IPC to the motor-disabled Rust
  harness. The harness independently performs authority admission, final
  limiting, mecanum projection and fake-sink output. Existing health snapshots
  lack command-correlated per-participant state.

## Requirements

- R1: Establish one project-native behavior-tree framework as the authoritative
  high-level orchestrator for robot work. It must build on the existing
  bounded mission IR/runtime rather than introduce a competing scheduler.
- R2: Provide a stable public framework API for validated tree construction,
  registered leaf actions/conditions, deterministic event-driven progression,
  resource/cancellation semantics, outcomes and bounded execution state.
- R3: Define explicit integration boundaries: project-tree leaves invoke
  services such as navigation by canonical command ID; Nav2 remains an
  isolated action provider and never gains motion authority.
- R4: Provide command-correlated, versioned motion-participation snapshots
  covering outer tree activations, the navigation action boundary, ROS bridge,
  authority admission, final limiter, mecanum projection and output sink.
- R5: Treat the Nav2 action lifecycle as one external leaf boundary. Do not
  instrument, modify or infer individual Nav2 BT/planner/controller nodes in
  this delivery.
- R6: Preserve fail-closed authority and realtime constraints: no framework
  path may mint authority, refresh command freshness, block the final motion
  loop, allocate unbounded state, or create physical motor/CAN output.
- R7: First vertical slice is a motor-disabled `NavigateToPose` project-tree
  leaf. The framework must remain reusable for future arm, perception and
  multi-resource leaves without pretending those adapters are already built.
- R8: V1 tree definitions use typed Rust builders and a registered leaf
  catalog. The framework emits a deterministic, human-readable tree and live
  participant view; it does not parse a declarative tree file yet.
- R9: Every action's declared finite timeout produces a correlated deadline
  effect for an external scheduler. The reducer remains clock-free and must
  receive the resulting deadline as an explicit event.

## Acceptance criteria

- [ ] A public, bounded project behavior-tree API executes the existing core
      composition set with deterministic, testable outcomes and cancellation.
- [ ] A validated example tree dispatches `NavigateToPose` through the existing
      navigation action boundary and receives correlated lifecycle results.
- [ ] During that action, an inspection snapshot identifies active outer tree
      nodes, the external navigation leaf lifecycle, and every downstream
      motion stage through the fake sink.
- [ ] Planning failure, proposal silence, bridge loss, authority rejection and
      local stop produce explicit participant states and typed reasons.
- [ ] The framework and adapter are documented, have unit/integration tests,
      and preserve full host CI plus motor-disabled safety checks.

## Scope boundaries

- The framework is project-native Rust. It will adapt to Nav2 actions rather
  than replace Nav2 or embed ROS/BehaviorTree.CPP in the control runtime.
- V1 uses code-defined trees plus a textual inspection view because it reuses
  the existing validated builder and is faster to make safe and understandable
  than introducing a second configuration language.
- V1 is event-driven: action/condition/cancellation/result events and
  externally scheduled deadlines advance the tree. No periodic behavior-tree
  loop is placed on the motion-control path.
- Nav2's XML remains the unmodified implementation behind a navigation leaf;
  the framework observes only its existing action lifecycle.
- This delivery does not add arm/perception leaf implementations, a graphical
  editor, production hardware output, Orange Pi work, runtime loading of
  untrusted tree definitions, or any Nav2 internal BT instrumentation.

## Child deliverables

- `08-06-behavior-tree-core` owns the public framework facade, bounded live
  tree state, action deadline scheduling, renderer and CLI foundation.
- `08-06-behavior-tree-navigation-participation` depends on the core task. It
  owns the motor-disabled navigation leaf adapter, correlated action
  acknowledgement, bounded status channel and live participant CLI command.
