# Project behavior-tree framework — design

## Purpose and boundary

The project gets one high-level, project-native Rust behavior-tree framework.
It is an event-driven orchestration layer, not a realtime controller and not
a replacement for Nav2. Its first leaf requests `navigate_to_pose_v1` through
the existing bounded navigation action boundary.

```text
btctl (read-only) <--- latest participation snapshot --- rs-navigation-e2e
                                                          |
BehaviorTreeRuntime <--- action lifecycle events --------+
        |
        +-- NavigateToPose leaf --> navigation.action-request.v1
                                      |
                                 rs-navigation-bridge
                                      |
                                  Nav2 action

rs-navigation-e2e --> authority / final limiter / mecanum / fake sink
```

Nav2's internal behavior tree is deliberately out of scope. The outer leaf
sees only the existing action lifecycle and has no filesystem path, ROS node
handle, lease-minting ability or motor-output ability.

## Deliverable ownership

The parent owns cross-child contracts and integration review. The core child
must finish first. The navigation child depends on its public API and must not
copy its scheduler or state model.

| Child | Owns | Depends on |
|---|---|---|
| `behavior-tree-core` | bounded facade, execution snapshot, action deadline effect, static renderer, CLI foundation | existing `control-core::mission` |
| `behavior-tree-navigation-participation` | navigation leaf adapter, action acknowledgement, status IPC, live CLI view and E2E proof | completed core API |

## Non-negotiable invariants

- `MissionRuntime` remains the sole reducer. The framework facade delegates to
  it and does not introduce a second scheduler.
- Startup validation bounds node count, children, registered leaves, resource
  conflicts, retries, loops, activations and action invocations before a tree
  can run.
- The reducer reads no wall or monotonic clock and does no I/O. An external
  scheduler returns deadlines as correlated events.
- State snapshots and IPC representations have fixed capacities. Publishing a
  status snapshot is best effort and cannot delay, authorize or refresh motion.
- `rs-navigation-e2e` remains motor-disabled. The final limiter, mecanum map
  and fake sink remain on the only executable motion path.

## Compatibility and rollback

Existing `Mission*` APIs remain valid during V1. The new `BehaviorTree*`
facade is additive and uses the same IR/runtime. The navigation status channel
is additive; its absence or stale state makes the CLI report unavailable but
never changes action or motion admission. Reverting the navigation child
returns the E2E harness to its present direct action loop without changing
Nav2 or hardware behavior.
