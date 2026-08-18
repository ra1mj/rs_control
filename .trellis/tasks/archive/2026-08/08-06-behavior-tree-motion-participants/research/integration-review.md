# Behavior-tree and motion-participant parent integration review

## Child closure

Both ordered implementation children are archived:

- `08-06-behavior-tree-core`: bounded behavior-tree facade over the existing
  mission IR/runtime.
- `08-06-behavior-tree-navigation-participation`: motor-disabled navigation
  leaf, acknowledgement mapping, read-only participation inspection and E2E
  integration.

The facade remains an additive view of `MissionRuntime`; it does not introduce
a second scheduler, a clock, I/O, a dynamic executor or a control authority
path. Nav2 remains an external action provider. Its internal behavior tree and
XML are neither modified nor inspected.

## Parent review matrix

| Integration concern | Evidence |
|---|---|
| Bounded deterministic outer-tree API | `control-core::behavior_tree::tests::{facade_validates_and_renders_every_supported_composition, facade_snapshot_tracks_dispatch_cancellation_terminal_and_stale_events}` and `mission::runtime` replay/cancellation tests |
| Action deadline, timeout and cancellation correlation | `control-core::behavior_tree::tests::action_deadline_is_armed_once_and_preserves_timeout_through_cancellation`; `mission::runtime::tests::{cancellation_before_dispatch_binding_waits_for_goal_acceptance, timeout_and_scope_cancellation_are_correlated_and_nonterminal}` |
| Exact navigation acknowledgement before feedback | `rs-navigation-e2e` tests `navigation_leaf_waits_for_exact_ack_and_defers_pre_ack_cancellation` and `rejected_navigation_acknowledgement_is_terminal_without_feedback`; `contract-codec::mapping::tests::action_ack_is_exact_admission_evidence_without_terminal_payload` |
| Read-only bounded participation snapshot | `navigation-ipc::tests::{action_ack_and_participation_snapshot_are_typed_fresh_and_fixed, participation_channel_attaches_as_one_latest_diagnostic_snapshot}`; `btctl` CLI test `inspect_reads_a_fresh_snapshot_without_consuming_it` |
| Every motor-disabled downstream stage is visible | `apps/rs-navigation-e2e/src/main.rs::NavigationParticipation` emits typed records for tree, navigation action/bridge, authority, proposal, final limiter, mecanum and fake sink; `btctl` CLI inspection tests assert the rendered stage set |
| Fail-closed motion boundary | `rs-navigation-e2e` test `localized_feedback_loss_terminates_an_acknowledged_navigation_leaf`; `tests/control.rs::{proposal_timeout_and_peer_loss_emit_bounded_zero_decisions, wrong_owner_expiry_stale_duplicate_reorder_and_prior_boot_never_reach_sink}` |
| Existing motor-disabled Nav2 verification | archived navigation-child acceptance evidence and [`evidence/navigation-plugin-host-2026-08-04.md`](../../../../evidence/navigation-plugin-host-2026-08-04.md) retain the prior real-Nav2 host run and its raw SHA-256-anchored observations; current validation reruns only Rust host tests |

## Current validation

The parent integration run executes:

```text
cargo test -p control-core -p navigation-ipc -p contract-codec \\
  -p navigation-plugin-config -p btctl -p rs-navigation-e2e
cargo clippy -p control-core -p navigation-ipc -p contract-codec \\
  -p navigation-plugin-config -p btctl -p rs-navigation-e2e \\
  --all-targets --all-features -- -D warnings
cargo run --locked -p architecture-check -- check
tools/ci.sh
```

## Qualification boundary

All parent evidence is host software evidence and terminates at a bounded fake
sink. The integration creates no physical motor/CAN output and does not qualify
an Orange Pi/RK3588, realtime timing, physical stopping behavior, Nav2
production deployment, or Nav2 internal behavior-tree behavior.
