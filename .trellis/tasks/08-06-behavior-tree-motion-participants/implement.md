# Project behavior-tree framework — implementation plan

## Ordered delivery

1. Complete and review the `behavior-tree-core` child.
   - Expose the public facade over the existing bounded mission IR/runtime.
   - Add bounded activation snapshots and action-level deadline effects.
   - Add deterministic static rendering and the `btctl` CLI foundation.
2. Complete and review the `behavior-tree-navigation-participation` child.
   - Add correlated navigation action acknowledgement.
   - Drive the motor-disabled navigation run through the tree facade.
   - Publish and inspect a bounded participation snapshot.
3. Run cross-child integration checks.
   - Core replay/allocation tests.
   - Navigation fake-server success/cancel/fault tests.
   - Motor-disabled real Nav2 smoke where the host environment is available.
   - Full `tools/ci.sh`.

## Review gates

- Start only the core child first; it has no dependency on Nav2 or the external
  `chassis_nav` repository.
- Do not change `chassis_nav` behavior-tree XML or add Nav2 internal
  instrumentation.
- Before the navigation child changes its shared manifest, compare exact YAML
  bytes with the peer repository and preserve unrelated dirty work there.
- Treat a missing/stale inspection snapshot as an observability failure only;
  it must not alter control authority or stop timing.
