# KINETIC arm-algorithm provenance

## Reference

- Project: `softmata/kinetic`
- Repository: <https://github.com/softmata/kinetic>
- Reviewed commit: `3dbaa61579c4359ce4c35f246fc83e1d3110b646`
- Upstream license: Apache-2.0
- Reviewed snapshot date: 2026-07-28

The commit above is an audit reference for selective algorithm research. It is
not a Cargo dependency, Git submodule, vendored workspace, compatibility API,
or version lock for this repository.

## Integration policy

Project contracts, model/profile compilation, unit types, authority checks,
collision evidence, configuration, error handling, and hardware admission are
implemented natively. An upstream kernel may be adapted only after its
numerical behavior, allocation, randomness, dimensions, dependencies, and
license obligations have been reviewed.

A source file that is substantially copied or adapted must include a concise
header that points here. The inventory below must be updated in the same
change. Purely project-native files do not claim KINETIC origin.

## Derived-kernel inventory

No source file is substantially copied or adapted from KINETIC yet.

| Upstream path | Local path | Purpose | Material changes |
|---|---|---|---|
| _None_ | _None_ | _None_ | _None_ |

## Reviewed source inventory

The following upstream modules were inventoried at the reviewed commit. This
table is a research map, not a statement that code was copied or that every
surface is already implemented.

| Capability | Reviewed upstream paths | Local integration rule |
|---|---|---|
| Kinematics and generic IK | `crates/kinetic-kinematics/src/{chain,forward,ik,dls,fabrik,sqp,bio_ik}.rs` | Project-owned profile/group/order contracts; adapt a numerical kernel only after boundedness review |
| Analytical and cached IK | `crates/kinetic-kinematics/src/{opw,paden_kahan,subproblem,ikfast,cached_ik}.rs` | Explicit geometry compatibility; no hidden fallback |
| Workspace analysis | `crates/kinetic-kinematics/src/workspace.rs` | Explicit seed and sample budget |
| Trajectory | `crates/kinetic-trajectory/src/{trapezoidal,totp,jerk_limited,spline,blend,monitor,validation}.rs` | Bounded project trajectory and caller-owned sampling |
| Dynamics | `crates/kinetic-dynamics/src/lib.rs` | Strict project model/inertia adapter; no nominal masses |
| Sampling and optimization planning | `crates/kinetic-planning/src/{rrt,rrt_star,bi_rrt_star,bitrrt,est,kpiece,prm,chomp,stomp,gcs,iris}.rs` | Explicit work budgets and `CollisionUnchecked` output |
| Cartesian/constrained pipelines | `crates/kinetic-planning/src/{cartesian,constrained_rrt,shortcut,smooth,pipeline,constraint}.rs` | Project-owned validators and typed evidence |
| Reactive control | `crates/kinetic-reactive/src/{servo,filter,policy}.rs` | Proposal-only output; no authority or hardware command |
| Grasp generation | `crates/kinetic-grasp/src/{antipodal,top_down,side,suction,quality}.rs` | Bounded project geometry and collision-unchecked candidates |
| Task composition | `crates/kinetic-task/src/{stage_engine,planners,applications}.rs` | Existing action/cancellation semantics |
| Deferred coordinated planning | `crates/kinetic-planning/src/dual_arm.rs` | Inventory only; return typed unsupported capability |

## Qualification boundary

This record covers source provenance only. Host tests do not qualify collision
safety, physical motion, torque accuracy, realtime scheduling, Orange Pi
deployment, or live ROS behavior.
