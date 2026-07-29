# Orange Pi 5 Plus realtime platform — research baseline

Research date: 2026-07-29 (Asia/Shanghai)

Only primary sources and the inspected source trees are used for technical claims. Performance numbers published by another project are treated as hypotheses until reproduced on this project's RK3588 target.

## 1. Selected immutable sources

| Component | Official source | Selected identity | Integrity / note |
|---|---|---|---|
| Orange Pi RT kernel | `https://github.com/orangepi-xunlong/linux-orangepi.git` | branch `orange-pi-5.10-rk35xx-rt`, commit `9f9e9d18574d0914c0d192a90c3babfe1fd63c95` | Makefile reports `5.10.160`; `localversion-rt` is `-rt89` |
| Orange Pi build system | `https://github.com/orangepi-xunlong/orangepi-build.git` | commit `bdba421984211da19191dc6ac6818a247817335f` | official board config supports `orangepi5plus`, `jammy`, RT legacy branch |
| Vendor U-Boot provenance | `https://github.com/orangepi-xunlong/u-boot-orangepi.git` | branch `v2017.09-rk3588`, commit `59fb85d0f6c435c9e5a8e6e0c54aa93b7a65ddfc` | retained bootloader; first kernel slice does not rebuild it |
| Rockchip binary provenance | `https://github.com/orangepi-xunlong/rk-rootfs-build` | rkbin commit `2d00465636c9f53a210ebe20ed96bc0ee9fb36c3` | provenance only in initial kernel-package path |
| Upstream RT patch provenance | `https://cdn.kernel.org/pub/linux/kernel/projects/rt/5.10/older/patch-5.10.180-rt89.patch.xz` | `5.10.180-rt89` | SHA-256 `750912fc91154bb8dc75a68c4ba96f896c73e5d6fa72e4c0e9465425adbea602` |
| Arm GNU toolchain | Arm GNU 11.2-2022.02 AArch64 none-linux-gnu tarball | `gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu` | SHA-256 `52dbac3eb71dbe0916f60a8c5ab9b7dc9b66b3ce513047baa09fae56234e53f3` |
| Clean build base | official Ubuntu image | `ubuntu:22.04@sha256:0d779ea97881505f5ef0039336ee85edba27519bdba968c284c86ee066a973c8` | AMD64 manifest digest selected for cross-build host |
| `rt-tests` | `https://git.kernel.org/pub/scm/utils/rt-tests/rt-tests.git` | tag `v2.10`, peeled commit `40b14a2aac9449dfb07d4f9b4bd873fc3ea62f28` | cyclictest supports duration, affinity, main affinity, memory lock, ns and JSON |
| Horus research input | `https://github.com/softmata/horus.git` | commit `fac3cfb8704e275bde9daec6d554a17c59dd8532` | Apache-2.0; inspected 2026-07-29, not added as dependency |

Source pages:

- Orange Pi RT commit: https://github.com/orangepi-xunlong/linux-orangepi/commit/9f9e9d18574d0914c0d192a90c3babfe1fd63c95
- Orange Pi build commit: https://github.com/orangepi-xunlong/orangepi-build/commit/bdba421984211da19191dc6ac6818a247817335f
- upstream RT archive index: https://www.kernel.org/pub/linux/kernel/projects/rt/5.10/older/
- Arm GNU downloads: https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
- `rt-tests` v2.10 tree: https://git.kernel.org/pub/scm/utils/rt-tests/rt-tests.git/tag/?h=v2.10
- Horus inspected commit: https://github.com/softmata/horus/commit/fac3cfb8704e275bde9daec6d554a17c59dd8532

## 2. Orange Pi RT provenance finding

The official RT branch is not a simple “Linux 5.10.160 + matching 5.10.160 RT patch” combination.

Observed history:

1. `a0ad355f96b28f13720f436a651d45469ed2b583` — commit message records applying `patch-5.10.180-rt89.patch` on Rockchip base `cae91899b67b`.
2. `b38c1f2c911074c1775a65ae65ca6e02d554fadd` — Rockchip runtime fixes affecting arm64 page-table/FPSIMD and cpufreq code.
3. `117bfdb14d47032061e91d842f553d24fb0355f3` — latency optimization updating `arch/arm64/configs/rockchip_rt.config`.
4. selected head `9f9e9d18574d0914c0d192a90c3babfe1fd63c95` still reports `5.10.160-rt89` through Makefile/localversion.

Decision:

- pin and build the complete official RT branch commit;
- do not reapply the RT patch;
- record upstream patch hash and integration commits;
- surface the reported/source version discrepancy in every build manifest;
- require a full regression for any attempted cleanup or upgrade.

This resolves the reproducibility question but does not resolve long-term BSP/security maintenance. A production hardening phase will need a separately qualified newer Rockchip/Orange Pi kernel.

## 3. Kernel configuration findings

The Orange Pi build repository's `legacy-rt.config` selects the RT branch and already contains:

- PREEMPT_RT, high-resolution timers and HZ=1000;
- CPU isolation, RCU boost, IKCONFIG and cgroup/cpuset support;
- CAN core/raw/device support and Rockchip CAN;
- V4L2/DMABUF and USB support.

The inspected config SHA-256 is `4a275a472c81ecb7012df46ece711d72a6aebd86fe59b4350a729e912e6b1244`.

Important safety finding: the vendor `rockchip_rt.config` explicitly disables `CONFIG_CPU_FREQ_THERMAL` as a latency optimization. That conflicts with this project's requirement that throttling remain visible and that active thermal protection not be traded for benchmark results.

Decision: apply a project fragment last with `CONFIG_CPU_FREQ_THERMAL=y`, active cooling and a qualification rule that any observed throttling invalidates the run. Production config can remain lean with ftrace/perf disabled; a separate debug config may be introduced later instead of carrying heavy tracing in the qualified runtime.

## 4. CAN board findings

The official Orange Pi 5 Plus wiki documents the board's CAN overlays and 1 Mbps SocketCAN commands:

- https://www.orangepi.org/orangepiwiki/index.php/Orange_Pi_5_Plus

Source/tree inspection confirms:

- CAN0 overlay: `rk3588-can0-m0.dtbo`; TX physical pin 5, RX physical pin 3;
- CAN1 overlay: `rk3588-can1-m0.dtbo`; TX physical pin 18, RX physical pin 16;
- the inspected raw CAN0 overlay source hash is `bfb78b328e42189fef7843b159e97a0083303b65ea3c8a86c5f357251d2824af`.

The SoC pins are 3.3 V logic-side CAN controller signals, not CANH/CANL. The project therefore requires an external isolated, 3.3 V-compatible transceiver and 120 ohm termination at exactly the two physical bus ends. The exact transceiver model and carrier revision remain required target evidence rather than an invented selection.

Safety decision: Phase B uses loopback/analyzer traffic only. Motor connection, enable and torque belong to the later Damiao child after a physical E-stop/current-limited fixture gate.

## 5. Linux realtime measurement findings

Primary references:

- PREEMPT_RT theory: https://docs.kernel.org/core-api/real-time/theory.html
- CPU isolation guidance: https://docs.kernel.org/admin-guide/cpu-isolation.html
- cyclictest guide: https://wiki.linuxfoundation.org/realtime/documentation/howto/tools/cyclictest/start
- cyclictest test design: https://wiki.linuxfoundation.org/realtime/documentation/howto/tools/cyclictest/test-design

The kernel isolation documentation emphasizes retaining housekeeping CPUs and measuring the workload; it does not justify copying a CPU mask from another platform. The Orange Pi RT config is periodic-tick and does not enable `NO_HZ_FULL`, so this phase will benchmark affinity/cpuset/isolation candidates without assuming full dynticks.

The pinned cyclictest source confirms:

- `--interval` is microseconds;
- the default timer mode is absolute;
- `--nsecs` makes min/max/avg nanoseconds and JSON field `resolution_in_ns` equal `1`;
- `--affinity`, `--mainaffinity`, `--mlockall`, `--policy=fifo`, `--priority`, `--duration`, `--quiet` and `--json=FILE` are available;
- JSON includes global `return_code`, then `thread.*.{cycles,min,max,avg,cpu,node}`.

Decision: invoke ns output unconditionally and compare the 1 kHz threshold as `100000 ns`, preventing a unit ambiguity. Kernel wakeup latency is reported separately from later application execution/deadline misses.

## 6. Horus source review

Horus documentation and repository describe a shared-memory robotics runtime:

- project/source: https://github.com/softmata/horus
- shared-memory concepts: https://docs.horusrobotics.dev/concepts/shared-memory
- scheduler concepts: https://docs.horusrobotics.dev/concepts/core-concepts-scheduler
- RT setup: https://docs.horusrobotics.dev/advanced/rt-setup

The repository README explicitly says its real-hardware validation is still sought and its published benchmark platform is a desktop Intel i9. Therefore no Horus latency number is imported as an RK3588 acceptance threshold.

### 6.1 CPU affinity

Useful pattern:

- node/runtime configuration makes affinity, FIFO priority and memory lock explicit;
- RT work is separated from async/network/logging work;
- capability detection and readback are first-class concepts.

Source finding:

- `horus_sys::rt::pin_to_cores` iterates a list and returns after `core_affinity::set_for_current` succeeds once. It therefore implements “select the first usable single core”, not “set this entire CPU mask”.
- the RT executor can also continue unpinned/normal priority after setup failure and broadly move IRQs away from the RT list.

Pinned sources:

- https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_sys/src/rt/mod.rs#L128-L174
- https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/scheduling/rt_executor.rs#L429-L555

Project decision:

- keep candidate selection and exact masks as different types/operations;
- set Linux logical CPU IDs exactly and verify via readback;
- use a role-aware CAN IRQ policy;
- fail closed for the qualified profile and reserve graceful fallback for an explicitly unqualified development profile.

### 6.2 Shared-memory layout and ring protocol

Useful patterns found in source:

- `TopicHeader` is 64-byte aligned and places producer `head` and consumer `tail` on separate cache lines;
- fan-out preallocates a publisher x subscriber SPSC matrix, avoiding a contended shared MPMC queue;
- capacity is power-of-two with masked monotonic indices;
- every slot has an odd/even, position-tagged version stamp; the consumer checks before and after copying to reject an overwrite/torn read;
- bounded retries avoid an unbounded spin when a slot is continually changing;
- subscribers that fall behind fast-forward to the oldest still-live position (drop-oldest/latest-window behavior).

Pinned sources:

- header/cache layout: https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/topic/header.rs
- seqlock protocol: https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/topic/seqlock.rs
- SHM SPSC matrix: https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/topic/shm_fanout.rs

Project adaptation:

- latest-value state, lossy sensor streams, discrete commands/actions and recorder queues require different overflow contracts;
- every overwrite/reject must increment an observable counter and preserve sequence-gap evidence;
- Protobuf stays out of the 1 kHz local fast path and is mirrored by a non-RT middle-layer bridge;
- layout magic/version/schema/endianness and endpoint-capacity exhaustion become explicit contracts.

### 6.3 Crash-safe process endpoint registration

Horus uses a per-topic/per-endpoint lock file. An exclusive OS `flock` is held for the endpoint lifetime; a PID and active bit are derived metadata. Clean shutdown clears metadata before releasing the lock, while process death releases the lock in the kernel and permits a later process to reclaim the stale slot. A subscriber resets its tail to current head when reclaiming a slot so it does not consume a dead process's backlog.

This is a sound cold-path pattern worth prototyping. The project will additionally test real `SIGKILL`, PID reuse/namespace behavior, filesystem namespace permissions, simultaneous reclaim and full endpoint exhaustion on Linux/RK3588. A heartbeat alone is not accepted as proof that a live process can be safely reclaimed.

### 6.4 POD safety issue

Horus's automatic fast-path predicate is effectively:

```text
!needs_drop::<T>() && size_of::<T>() > 1
```

Its own comments acknowledge validity-invariant limitations. The heuristic is not sufficient for raw shared-memory reconstruction: a raw pointer, padding-bearing struct, restricted enum or `NonZeroU32` can require no destructor while still being invalid/meaningless when copied from another process.

Pinned source:

- https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/pod.rs

Project decision:

- no automatic POD detection;
- require an explicit, reviewed `bytemuck::Pod`-like contract with fixed representation and initialized padding, or transport opaque bytes/descriptors;
- isolate all future mmap/atomic-pointer `unsafe` in a small middle-layer crate while keeping the workspace-wide default `unsafe_code = forbid` elsewhere;
- use descriptors/owned pools for camera and point-cloud payloads instead of putting process-local pointers in shared memory.

## 7. Remaining target-only facts

The following cannot be truthfully completed on the development host and must appear in the first evidence bundle:

- exact Orange Pi 5 Plus board revision and firmware/bootloader build;
- supply model/rating and active cooler model;
- isolated CAN transceiver model, isolation rating, logic supply and termination state;
- actual DT overlay path/hash installed on the target;
- online CPU IDs, capacity/frequency topology, allowed cpuset and selected affinity candidate;
- actual CAN IRQ number/name and writable affinity behavior;
- kernel package/artifact hashes and boot result;
- full 8-hour CPU/memory/storage/Ethernet/NPU/GPU timing/thermal results;
- serial-console rollback rehearsal.

Until these exist, the platform is reproducibly specified and host-checked, but not hardware-qualified.
