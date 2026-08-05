# Navigation plugin host evidence — 2026-08-04

## Result and scope

The motor-disabled `rs_control` integration completed one real ROS 2 Humble
Nav2 `NavigateToPose` action from `(1.5, -1.5, 0.0)` to
`(1.5, -3.0, 0.0)` on the immutable `warehouse_sim_map` fixture. The action
terminated `Succeeded` after SmacPlanner2D planning, MPPI control proposals,
Rust authority checks, the sole Rust body limiter, mecanum projection and
deterministic odometry/TF feedback.

This is x86_64 host software evidence only. Motor output and CAN transmission
were compiled out. The fresh empty point cloud is a test-only obstacle-input
readiness source, not production localization or perception. Production SLAM,
physical sensors, AArch64 timing and whole-robot qualification remain pending.

Raw evidence is retained in
[`navigation-plugin-host-2026-08-04-active-final/`](navigation-plugin-host-2026-08-04-active-final/).
From that directory, `sha256sum -c SHA256SUMS` validates all 97 captured raw
files. The SHA-256 of `SHA256SUMS` is
`1b654d7e295c0bd2dfd2e71a90221344a63274df8cc202be4d7a5ca1edbfe3f5`.
The matching localization-loss run is retained in
[`navigation-plugin-host-2026-08-04-localization-fault/`](navigation-plugin-host-2026-08-04-localization-fault/);
its 97-file `SHA256SUMS` digest is
`ac5dfda4e3a93fbc45a571757c62dc78ff6f59ed4e745efb80c80f02e07423c6`.
Both runs bind bridge binary
`31b5e886a36230799c8740803f73f83a2d95d17be51e7be6d497f544bca2a568`
and harness binary
`85e2fd96a38d761c7ba1475735cccb610aa26c00950f6ebc3402a78befe8d10a`.
The pre-optimization run with a persistent Python `ros2 launch` parent is
retained in
[`navigation-plugin-host-2026-08-04-ros2-launch-baseline/`](navigation-plugin-host-2026-08-04-ros2-launch-baseline/);
its `SHA256SUMS` digest is
`72c1c37af62e0c261d58ae2b1d133557036f196dc64d160bad990eda7977fa43`.

## One-second concurrent snapshot

CPU is the process `utime + stime` delta over the one-second sample on a
100 Hz kernel, expressed as percent of one logical CPU. PSS is shown because
adding RSS would double-count shared pages.

| Slice | Processes | PSS (KiB) | Threads | CPU |
|---|---:|---:|---:|---:|
| Nav2 managed nodes | 5 | 97,949 | 70 | 12% |
| direct fail-together shell supervisor | 1 | 682 | 1 | 0% |
| isolated Rust ROS bridge | 1 | 13,870 | 11 | 2% |
| motor-disabled Rust control harness | 1 | 2,548 | 1 | 1% |
| synthetic empty-cloud source | 1 | 35,335 | 42 | 1% |
| Complete measured host graph | 9 | 150,384 | 125 | 16% |

The complete graph's additive PSS observation is about 146.9 MiB. It is a
point-in-time measurement, not a commissioned pass/fail budget or peak-memory
claim. A privileged systemd deployment can additionally collect aggregate
cgroup current/peak memory and CPU counters with the checked collector.

Excluding the test-only empty-cloud process, the checked runtime PSS fell from
139,948 KiB with `ros2 launch` to 115,049 KiB with direct native process
supervision. That is a measured reduction of 24,899 KiB (about 24.3 MiB or
17.8%). The shell supervisor itself used 682 KiB PSS. This comparison is only
the one-second host snapshot; it is not a hard resource budget or a production
SLAM/whole-robot qualification.

The earlier `chassis_nav` Nav2/Gazebo component evidence recorded whole-launch
peak RSS of 788,036 and 788,724 KiB for an 18-process graph that included
Gazebo and its validation runner. That peak RSS is retained as a synthetic
component baseline but is not directly compared or added to this graph's PSS.

## Fault and lifecycle evidence

- Real Nav2 action: `Succeeded`, result code `0`.
- Fake action lifecycle: success and correlated terminal cancellation passed.
- Bridge termination: the Rust harness remained alive and emitted typed
  `PeerUnavailable` zero-output evidence in 6 ms, below the 100 ms proposal
  lifetime.
- Nav2 process termination kept both Rust processes alive; the last accepted
  proposal expired at 101.473 ms, including the 5 ms harness observation
  period. The end-to-log process-fault observation was 167 ms because an
  already published DDS sample may arrive after the producer exits.
- Localization/TF loss: the real action terminated `Failed` with result code
  `3`, then emitted typed `ProposalExpired` zero-output evidence at
  101.357 ms while the Rust harness remained alive.
- Missing/wrong/expired authority, stale/reordered/prior-boot proposals and IPC
  loss are also covered by allocation-free Rust integration tests.

The reproducible real-Nav2 command is documented in
[`nav2-host-resource-smoke.sh`](../platform/ros2-humble/navigation-deployment/tests/nav2-host-resource-smoke.sh).
