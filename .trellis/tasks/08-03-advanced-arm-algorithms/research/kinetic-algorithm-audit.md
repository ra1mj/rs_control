# KINETIC algorithm and performance audit

## Reviewed source

- Repository: `https://github.com/softmata/kinetic`
- Commit: `3dbaa61579c4359ce4c35f246fc83e1d3110b646`
- Commit date: `2026-07-28T15:54:49Z`
- License: Apache-2.0
- Review date: 2026-08-03

The commit matches the snapshot already recorded by the parent task. It is an
audit identity only. This project will not add KINETIC as a dependency,
submodule, vendored workspace or public namespace.

## Performance evidence in the reviewed snapshot

KINETIC's own `docs/benchmarks.md` reports measurements from 2026-03-02 on
Linux x86_64:

| Surface | Reported measurement |
|---|---:|
| DLS IK, 7-DOF | ~10.6 us |
| FABRIK IK, 3-DOF | ~21.5 us |
| Cartesian linear, 10 cm | ~133 us |
| GCS build + plan | ~172 us |
| constrained-RRT setup only | ~986 ns |
| RRT-Connect, no obstacles | ~258 ms |
| RRT-Connect, table | ~244 ms |
| RRT-Connect, 10 obstacles | ~262 ms |

The documentation contains a materially different table in
`docs-site/src/reference/benchmarks.md`, which claims simple RRT-Connect below
100 us and obstacle cases in the 5-80 ms range. The checked Criterion benchmark
source sets `<100 us p50` as an objective, but the more recent measured table
does not show that objective being met. Therefore this project will treat the
numbers as upstream claims/evidence to reproduce, not inherited guarantees.

## Source inventory and adaptation decision

| KINETIC file | SHA-256 | Project decision |
|---|---|---|
| `kinetic-kinematics/src/dls.rs` | `d400dbbc99bb95019d97380ecb7abfe4663fc4f16cee84ea530935023771743c` | replace/rebase the local DLS kernel; this is the sole DLS implementation source |
| `kinetic-kinematics/src/fabrik.rs` | `a4c18f0ff008589d78caa16a3728d2441948c512854789128b9b7a48fd1f9931` | rewrite against project chain/workspace types |
| `kinetic-kinematics/src/sqp.rs` | `623ab907dec8ece96a273a77f4f10101618106db9971e3ae5255b694690c2828` | selectively adapt bounded SQP kernel |
| `kinetic-kinematics/src/bio_ik.rs` | `6b4087f1a6c5566ba0dc2c457d3e7c1d82284c49ead93a9f4461160d2048ca1f` | redesign deterministic seeded population/workspace; no `thread_rng` |
| `kinetic-kinematics/src/cached_ik.rs` | `a4cb18ec426de1a337bd81afb86a5b6166013a5d7edb21cb6719c4060542cb52` | redesign as profile/group/generation-bound bounded cache |
| `kinetic-kinematics/src/opw.rs` | `13459111f0af3012dd480103890cc78f9751bd27566801a5dd0aaa6ab8629173` | adapt analytical equations after geometry and golden review |
| `kinetic-kinematics/src/paden_kahan.rs` | `e14dcb7b5d27e301b9071c9f4dacb85bd8e22d92e3632edee3cb2e35ace0944f` | adapt bounded fixed-capacity subproblem primitives |
| `kinetic-kinematics/src/subproblem.rs` | `c9a8b04d766a9930251150f6040e240511866d07e2bac4e1b8e8cd72cd0203d4` | redesign 6/7-DOF compatibility and fixed-capacity outputs |
| `kinetic-planning/src/rrt.rs` | `fb3e09df4cfa1d53803cdf10241cb3e0b1bde3310dea772cb8a7ff915e0b56fc` | rewrite against semantic validators and preallocated nodes |
| `kinetic-planning/src/rrt_star.rs` | `1f450ec3a2d8e20ca4a93dbc10e7b7681db94bacaf0f4054cae7784d1d7e1c18` | adapt informed sampling/rewiring concepts, not integration shell |
| `kinetic-planning/src/bi_rrt_star.rs` | `d320ee29fde028cf9dae8fea0033f405d3fa5cf3a2729d967bb34750e359eeae` | independently implement two-tree rewiring |
| `kinetic-planning/src/bitrrt.rs` | `c7423111e66bf85627ebb509fb9f57ff83c3544ea7af539af1bbfc71a2a369f1` | adapt transition-test/temperature mechanics with seeded RNG |
| `kinetic-planning/src/est.rs` | `bd2ed10dc3927856280bb34f367bcf8e04bfd87082b263de96071652725c87ab` | independently implement density-weighted expansion |
| `kinetic-planning/src/kpiece.rs` | `8b7bc7c90e6aee9b78649f21fa6bc8c69b881f635fcf9492e25e1225755b4dfc` | independently implement projected-cell exploration |
| `kinetic-planning/src/prm.rs` | `772650a0a84ea2184d8eef48bdba3590df6aa0a0d47bb2ef6513358b7edc9577` | redesign bounded persistent roadmap and fast query split |
| `kinetic-planning/src/chomp.rs` | `5b9e776a451fbfdae65e120f6fab258a803c349f486381b802d40496614ee07b` | adapt smoothness/covariant update with precomputed matrices |
| `kinetic-planning/src/stomp.rs` | `64a7602a42dc3f170e1936ad06117cf76425f07acce49d635b8debc353ae7b3d` | redesign seeded noise matrices and reusable sample workspace |
| `kinetic-planning/src/gcs.rs` | `5e7ba1b7b66798628b70e1451d8b6b1757a5c8f84e9d97470bfbd9ffc6976aef` | adapt region graph/A* query; region evidence stays caller-owned |
| `kinetic-planning/src/iris.rs` | `3f044d967643333d71d5cbaa71e93a2d8bc1986eead91d410bce2c7b193a6e99` | redesign as deterministic offline region compiler; no collision-free claim |
| `kinetic-planning/src/constrained_rrt.rs` | `7aef1ff7b5b0902b02ec0bd1b961c86504cbd2c5de7c5eee9a345dfa59d4e857` | adapt projection/search concepts against project constraints |
| `kinetic-planning/src/cartesian.rs` | `3b7e6fc0acdf908558b4240afc5cac32797e6b7e1de49f11e893ae196ca94523` | adapt pose stepping/jump detection without collision coupling |

## Required deviations from upstream

- KINETIC planning has hard path dependencies on `kinetic-collision`,
  `kinetic-scene`, execution and its own robot/core types. The project-native
  port must depend only on existing arm model, semantic constraint and
  collision-evidence contracts.
- Upstream uses `rand::thread_rng()` in numerical IK, Bio-IK, sampling planners,
  CHOMP/STOMP and IRIS. This project requires explicit recorded seeds and a
  deterministic RNG owned by each request/workspace.
- Upstream hot paths allocate many `Vec`, `HashMap` and `BinaryHeap` values and
  clone joint vectors during iterations. This project will preallocate bounded
  workspaces and forbid container growth after preparation.
- Upstream uses wall-clock `Instant` inside planner kernels. This project uses
  deterministic work counters in kernels and a caller-enforced monotonic
  deadline at the orchestration boundary.
- Upstream IRIS/GCS documentation calls regions collision-free. Without a
  qualified collision provider, this project may call them constraint-feasible
  only and every result remains `CollisionUnchecked`.
- Upstream caches are not bound to the project's immutable profile/group/order
  identity. Native caches and precomputed roadmaps must reject identity or
  generation mismatch.

## Performance interpretation

The microsecond target is attainable only when preparation and online query are
separated:

1. Model geometry analysis, matrix factorization, region generation, roadmap
   construction and workspace allocation are bounded preparation operations.
2. Warm online queries reuse immutable profile/group-bound prepared state.
3. Analytical/cached IK, GCS/PRM queries, Cartesian local paths and nominal
   CHOMP updates are benchmarked as complete calls including validation and
   result construction.
4. Sampling searches still have independent hard deadlines; they cannot promise
   a solution for arbitrary constraints in microseconds.

This split preserves truthful behavior while targeting KINETIC-class latency
for the surfaces for which the reviewed source provides microsecond evidence.

## IK source exclusivity

The user selected this reviewed KINETIC commit as the sole source of all IK
kernel implementations. The project may alter integration types, validation,
randomness, allocation, caches and performance mechanics, but it may not mix in
an IK kernel from another library or an independently substituted algorithm.
An incomplete or unsafe-to-adapt KINETIC solver remains typed unavailable.
