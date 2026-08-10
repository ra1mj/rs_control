# Behavior-tree navigation participation — implementation plan

1. Start only after the core child is checked and committed; read the cross
   layer, robot-contract, logging and realtime specifications.
2. Add generic action acknowledgement semantic/codec support using the existing
   `ActionAck` wire message. Extend navigation action IPC and bridge tests.
3. Add the fixed participation snapshot value, codec, latest channel,
   namespace attachment APIs and manifest/schema declarations. Update the
   byte-identical `chassis_nav` manifest copy and its contract tests in an
   isolated staged change.
4. Rework `rs-navigation-e2e` to build the checked navigation-shaped tree and
   translate every IPC response/deadline into core events. Preserve the sole
   limiter, mecanum projection and fake-sink path.
5. Publish best-effort snapshots after transitions/at low observation cadence.
   Add `btctl inspect` as a read-only subscriber and process-level CLI tests.
6. Extend fake-server and motor-disabled E2E tests for accepted/rejected goals,
   cancellation, bridge loss, proposal expiry and stale/missing status.
7. Run targeted Rust tests, isolated bridge build/tests, navigation deployment
   verification and full host CI. Run the real Nav2 smoke only in its existing
   motor-disabled host environment.

## Rollback points

- The acknowledgement and status channel are additive. Older consumers reject
  the new layout rather than silently reading it.
- If the core integration needs rollback, restore the direct E2E action loop;
  no Nav2 tree, authority rule or final-control path has changed.
