# Advanced arm algorithms integration review

## Scope and child state

All four delivery children are archived under `.trellis/tasks/archive/2026-08/`:

- `08-03-advanced-ik-kernels`
- `08-03-sampling-roadmap-planners`
- `08-03-optimization-constraint-planners`
- `08-03-arm-reactive-task-performance`

The parent review is host software only. It does not claim target-board,
realtime, collision-system, CAN, ROS, camera, GPU or physical-arm evidence.

## Requirement-to-test matrix

| Parent concern | Evidence |
|---|---|
| Distinct IK identity and typed unsupported/non-convergent cases | `tests/kinematics.rs`: `prepared_generic_kernels_are_distinct_and_context_bound`, `solver_admission_and_unreachable_failures_are_typed`, `analytical_kernels_admit_only_matching_axis_conventions` |
| Prepared identity, deterministic seeds, bounded cache/batch and warm IK allocation | `tests/kinematics.rs`: `bio_ik_is_seed_repeatable_and_cache_batch_storage_is_bounded`, `generic_admission_supports_mixed_dof_and_warm_solves_do_not_allocate` |
| Distinct sampling/roadmap behavior, repeatability, capacity and work exhaustion | `tests/planning.rs`: `blocked_direct_path_executes_each_distinct_sampling_family`, `sampling_plan_is_seed_deterministic_and_shortcut_preserves_constraints`, `prepared_prm_reuses_graph_and_rejects_constraint_generation_changes`, `impossible_constraints_exhaust_budget_and_deferred_topologies_are_typed` |
| Distinct optimization/constrained behavior and constraint-only region evidence | `tests/planning_optimization.rs`: `prepared_chomp_and_stomp_improve_distinct_objectives_deterministically`, `iris_certifies_regions_and_prepared_gcs_reuses_exact_context`, `blocked_paths_execute_each_optimization_and_constraint_family`, `every_prepared_artifact_rejects_constraint_generation_changes` |
| Reactive finite/bounded output and unavailable collision policy | `tests/reactive.rs`: `rmp_combines_reach_damping_limits_and_rejects_collision_policy`, `prepared_rmp_reuses_jacobian_storage_and_rejects_stale_generation` |
| Task proposal bounds, cancellation/reload invalidation and absence of authority | `tests/task.rs`: `task_compiler_binds_every_stage_to_prepared_context_without_authority`, `cancelled_or_reloaded_task_result_cannot_advance_the_cursor`, `task_compiler_rejects_cartesian_workspace_capacity_without_growth`; `arm-control/src/execution.rs` compile-fail example prevents `ArmTaskProposal` from satisfying `PhysicalArmExecutor` |

## Release host evidence

The isolated `ik_performance` Release test ran on `Linux 6.8.0-136-generic
x86_64 GNU/Linux`, 32 CPUs, Rust MSRV 1.97.0. All measurements are marked
`qualification=host-only`.

| Kernel | P50 ns | P95 ns | Warm allocations |
|---|---:|---:|---:|
| FABRIK | 16,039 | 16,824 | 0 |
| SQP | 2,771 | 2,787 | 0 |
| Bio-IK | 7,672 | 7,744 | 0 |
| Paden-Kahan | 3,005 | 3,029 | 0 |
| Redundant seven-DOF | 47,526 | 48,880 | 0 |
| OPW | 2,828 | 5,644 | 0 |
| IK cache lookup | 161 | 442 | 0 |
| Damped least squares | 1,964 | 1,993 | 0 |

Analytical/cache P95 values satisfy the 50 microsecond target. Numerical IK
P95 values satisfy the 500 microsecond target. The test asserts zero warm
allocations for every listed IK kernel. Bio-IK uses an allocation-free
in-place sort with a total `(fitness, ordinal)` ordering; stable `sort_by`
would allocate a temporary buffer during a generation.

## Comparison with KINETIC audit

`kinetic-algorithm-audit.md` records upstream DLS 7-DOF at about 10.6
microseconds and FABRIK 3-DOF at about 21.5 microseconds. The local DLS and
FABRIK fixtures are different six-DOF prepared-model inputs, with different
tolerances and work counts, so the numbers are not directly comparable. They
are retained only as separate implementation-reference observations; no
upstream timing, target-board or physical qualification is inherited.
