# Kinetic arm algorithms parent integration review

## Child closure

Both implementation children are archived:

- `07-31-kinetic-arm-stack`: project-owned arm model, algorithms and admission
  boundary.
- `07-31-rl-policy-control`: exact-pinned RTen 0.24.0 ONNX policy deployment
  on a named non-realtime worker.

The source relationship to KINETIC remains provenance only. The production
Cargo graph contains no KINETIC Git/path/submodule/workspace dependency or
public KINETIC namespace.

## Parent review matrix

| Integration concern | Evidence |
|---|---|
| SI identity, profile/group ordering and morphology isolation | `message-model` arm tests; `arm-model::profile::tests::{mixed_fixture_supports_variable_dof_units_and_multiple_groups, semantic_limit_or_mapping_change_changes_profile_identity}`; `rl-policy-control/tests/runtime.rs::mit_gain_profile_and_order_mismatch_fail_at_startup` |
| Native KAA-10 algorithms and bounded performance | archived `07-31-kinetic-arm-stack/research/integration-review.md`; archived `08-03-advanced-arm-algorithms/research/integration-review.md` and its isolated Release host reports |
| RTen dependency, safe loading and immutable artifact contract | archived `07-31-rl-policy-control/research/dependency-admission.md`; `rl-policy-control::{artifact,graph,manifest}` tests and `tests/runtime.rs::artifact_hash_extra_file_and_unknown_manifest_field_fail_closed` |
| Action decoding and fixed MIT gains | `tests/runtime.rs::{checked_fixture_loads_and_runs_fixed_gain_mit, all_non_mit_action_semantics_produce_typed_proposals, mit_gain_profile_and_order_mismatch_fail_at_startup}` plus `contract-codec` and `arm-control` golden tests |
| Reload, recurrent state, queue and timeout failure | `tests/runtime.rs::{late_result_is_withdrawn_and_reload_changes_generation, recurrent_state_advances_only_after_acceptance_and_reset_discards_it, recurrent_queue_discontinuity_withdraws_inflight_result, explicit_worker_shutdown_is_typed_and_non_authorizing}` |
| Authority and collision boundary | `PolicyDecision::into_collision_unchecked` produces only a semantic proposal; `arm-control` admission retains lease/deadline/lifecycle checks; `arm-control/src/execution.rs` compile-fail proof prevents semantic proposals from implementing `PhysicalArmExecutor` |
| Existing non-arm compatibility | normalized configuration and schema tests run in `tools/ci.sh`; checked-in `arm.morphology: none` configurations retain deterministic validation/hashes |

## Qualification boundary

All evidence is Linux host software evidence. No physical arm, target board,
Orange Pi/RK3588, CAN transmission, collision qualification, camera or live
ROS test was run or implied. The parent leaves collision, physical actuation,
closed-chain/parallel, floating-base and coordinated dual-arm qualification
explicitly open.

## Validation

The integration run executes:

```text
cargo test -p message-model -p contract-codec -p motor-damiao
cargo test -p arm-model -p arm-algorithms -p arm-control -p rl-policy-control
cargo clippy -p arm-model -p arm-algorithms -p arm-control -p rl-policy-control --all-targets --all-features -- -D warnings
cargo run --locked -p architecture-check -- check
cargo metadata --locked --no-deps --format-version 1
cargo deny check
tools/ci.sh
```

The local work is committed but deliberately not pushed by this integration
workflow.
