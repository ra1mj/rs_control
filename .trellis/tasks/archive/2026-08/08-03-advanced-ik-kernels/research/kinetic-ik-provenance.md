# KINETIC IK provenance and host evidence

## Reviewed source

- Repository: `https://github.com/softmata/kinetic`
- Commit: `3dbaa61579c4359ce4c35f246fc83e1d3110b646`
- License: Apache-2.0

| Project module | Sole KINETIC kernel source | Reviewed SHA-256 | Material adaptations |
|---|---|---|---|
| `ik/dls.rs` | `kinetic-kinematics/src/dls.rs` | `d400dbbc99bb95019d97380ecb7abfe4663fc4f16cee84ea530935023771743c` | fixed 6x6 algebra, caller-seeded restart, prepared FK/Jacobian, typed evidence |
| `ik/fabrik.rs` | `kinetic-kinematics/src/fabrik.rs` | `a4c18f0ff008589d78caa16a3728d2441948c512854789128b9b7a48fd1f9931` | reusable point buffers, exact project limits, bounded point-to-joint and orientation passes |
| `ik/sqp.rs` | `kinetic-kinematics/src/sqp.rs` | `623ab907dec8ece96a273a77f4f10101618106db9971e3ae5255b694690c2828` | prepared analytical Jacobian, reusable augmented matrix, projected line search |
| `ik/bio.rs` | `kinetic-kinematics/src/bio_ik.rs` | `6b4087f1a6c5566ba0dc2c457d3e7c1d82284c49ead93a9f4461160d2048ca1f` | fixed population, ChaCha8 caller seed, stable ordering, reusable generations |
| `ik/cache.rs` | `kinetic-kinematics/src/cached_ik.rs` | `a4cb18ec426de1a337bd81afb86a5b6166013a5d7edb21cb6719c4060542cb52` | quantized deterministic key, prepared generation identity, no implicit eviction |
| `ik/opw.rs` | `kinetic-kinematics/src/opw.rs` | `13459111f0af3012dd480103890cc78f9751bd27566801a5dd0aaa6ab8629173` | conservative Y-Z-Y axis admission, canonical base frame, streaming candidates, mandatory project-FK validation |
| `ik/paden_kahan.rs` | `kinetic-kinematics/src/paden_kahan.rs`, `kinetic-kinematics/src/subproblem.rs` | `e14dcb7b5d27e301b9071c9f4dacb85bd8e22d92e3632edee3cb2e35ace0944f`, `c9a8b04d766a9930251150f6040e240511866d07e2bac4e1b8e8cd72cd0203d4` | fixed subproblem candidates, corrected cosine-law convention, Z-Y-Z admission and full FK validation |
| `ik/redundant.rs` | `kinetic-kinematics/src/subproblem.rs` (`SubproblemIK7DOF`) | `c9a8b04d766a9930251150f6040e240511866d07e2bac4e1b8e8cd72cd0203d4` | caller-seed-stable bounded sweep, streaming seven-axis candidates and full FK validation |

`ik/prepared.rs` and `ik/analytical.rs` are project integration layers, not
alternative IK kernels. They own morphology identity, geometry admission,
storage, project FK validation and stable candidate selection.

## Source-policy result

The workspace contains no `kinetic` Cargo dependency, Git/path dependency,
submodule or vendored source. The only production IK dispatch is
`arm-algorithms::kinematics::PreparedKinematics::solve`; the legacy
`KinematicChain::solve_ik` wrapper delegates to that surface. Other damped
pseudoinverse uses under reactive control are velocity-control kernels, not IK
implementations.

## Host evidence

Command:

```text
cargo test --release -p arm-algorithms --test ik_performance -- --ignored --nocapture
```

Observed on the development host on 2026-08-03:

```text
analytical OPW P95 = 5,599 ns
cache hit P95      =   439 ns
numerical DLS P95 = 1,985 ns
```

These are isolated host Release measurements, not target-board or realtime
qualification evidence.
