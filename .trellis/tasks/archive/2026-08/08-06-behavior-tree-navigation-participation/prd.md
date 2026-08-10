# Integrate behavior tree navigation participation

## Goal

Use the completed project behavior-tree framework to run the motor-disabled
`NavigateToPose` leaf and expose a read-only, command-correlated answer to
which project-tree and downstream motion stages currently participate.

## Dependency

This task starts only after `08-06-behavior-tree-core` exposes its public
event-driven facade and fixed-capacity snapshot API. It must use that API and
must not duplicate scheduling, node-state or rendering logic.

## Confirmed facts

- `rs-navigation-e2e` currently builds and publishes its navigation action
  request directly, then manually tracks feedback/result/cancel state.
- The bridge correlates canonical and ROS IDs, but its local response channel
  currently has feedback, cancel decision and terminal result only. A Nav2 goal
  can be accepted then terminate before feedback, so an exact acceptance event
  is needed for correct outer-tree lifecycle progression.
- The harness already observes bridge health, authority, proposal admission,
  limiter/projection and fake-sink results. It can compose a status snapshot
  without touching Nav2's internal BT.

## Requirements

- N1: Add an exact, correlated action acknowledgement response at the existing
  navigation IPC boundary. Accepted and rejected goals must be distinguishable
  before any feedback/result and obey the generic action lifecycle.
- N2: Replace the direct E2E action loop with a `BehaviorTreeRuntime` driving
  one registered `NavigateToPose` leaf. IPC publication, acknowledgement,
  cancellation, result and deadline events must map exactly to the core API.
- N3: Add a fixed-capacity latest participation snapshot channel. The sole
  publisher is the motor-disabled harness; readers are unprivileged and
  read-only.
- N4: The snapshot must carry source/boot/process identity, optional canonical
  command ID, freshness and typed states for outer tree nodes, navigation
  action boundary, bridge, authority, proposal, final limiter, mecanum and
  fake sink.
- N5: Extend `btctl` with a read-only live-inspection command that attaches to
  an explicit absolute IPC namespace, rejects missing/stale data and prints a
  deterministic tree/stage view.
- N6: Status publication is best effort and independent of authority. A failed
  publisher/reader may never alter proposal admission, limiter execution, stop
  timing, hardware gates or Nav2 behavior.

## Acceptance criteria

- [x] Goal acceptance, rejection before feedback, feedback, cancellation and
      terminal result advance the outer tree only with exact command IDs.
- [x] A running motor-disabled navigation action displays a current tree leaf
      plus bridge, authority, proposal, limiter, mecanum and fake-sink stages.
- [x] Proposal expiry, bridge loss, authority unavailability and terminal
      navigation failure have unambiguous states/reasons in the snapshot.
- [x] `btctl inspect` can read a live snapshot and rejects stale/missing or
      wrong-layout data without writing to the namespace.
- [x] Existing fake-Nav2 and real-Nav2 motor-disabled success/cancel/fault
      checks continue to pass. No Nav2 XML or internal BT code changes.

## Scope boundaries

- No Nav2 internal behavior-tree instrumentation, XML modification, ROS
  BehaviorTree.CPP plugin or Groot integration.
- No physical motor/CAN output, Orange Pi work or production deployment claim.
- The snapshot is a local inspection contract; UI/dashboard and external wire
  publication are later work.
