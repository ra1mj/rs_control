# Orange Pi 5 Plus realtime platform — technical design

## 1. Design intent

This child task turns the parent architecture's realtime assumptions into a board-specific, reproducible platform contract. It deliberately separates three kinds of truth:

1. **Pinned build truth** — immutable source identities, config fragments and deterministic commands that can be checked on any host.
2. **Target observation truth** — what the actual Orange Pi reports after boot, including topology, IRQ routing, thermal behavior and CAN state.
3. **Qualification truth** — raw timing/stress evidence plus a machine-readable verdict. A candidate configuration is not qualified merely because it looks reasonable.

The platform directory contains OS integration code only. It does not become a hardware-module device API and it does not relax the dependency rule `application_module -> protocol_module -> hardware_module`.

## 2. Deliverable layout

```text
platform/orangepi5plus/
├── README.md
├── sources.lock
├── container/
│   └── Containerfile
├── kernel/
│   ├── rs-control-rt.fragment
│   ├── required.config
│   └── build-kernel.sh
├── config/
│   ├── can.env
│   ├── hardware-evidence.example.env
│   └── rt-profile.env
├── lib/
│   └── common.sh
├── scripts/
│   ├── can-link.sh
│   ├── collect-evidence.sh
│   ├── evaluate-cyclictest.py
│   ├── monitor-thermal.sh
│   ├── qualify-rt.sh
│   └── rt-affinity.sh
├── systemd/
│   ├── rs-control-can.service
│   ├── rs-control-platform-prepare.service
│   └── rs-control-platform-ready.target
├── examples/
│   └── rs-control-motion-rt.service
├── tests/
│   └── host-test.sh
└── docs/
    ├── bring-up.md
    ├── build-and-install.md
    ├── realtime-qualification.md
    └── rollback.md
```

Names may be consolidated during implementation when one script can safely own multiple checks, but the boundaries remain: source/build, runtime preparation, qualification/evidence and operator documentation.

## 3. Source and build provenance

### 3.1 Lock format

`sources.lock` is a data-only POSIX assignment file with a strict allow-list of keys. Scripts parse only known `KEY=value` records and reject shell metacharacters, duplicate keys, missing keys or unexpected keys; they do not blindly execute arbitrary lock-file content.

The lock records:

- Orange Pi kernel repository and exact RT commit;
- Orange Pi build repository commit used as a reference baseline;
- U-Boot and Rockchip binary provenance even though the first build retains the known bootloader;
- upstream RT patch URL and SHA-256 as the provenance of the vendor integration;
- Arm GNU cross-toolchain URL and SHA-256;
- Ubuntu build-container digest;
- `rt-tests` repository/tag/peeled commit;
- Horus repository/commit/license used for design review.

The build script writes the lock-file hash and effective config hash next to artifacts.

### 3.2 Vendor RT version skew

The selected official Orange Pi branch identifies itself as Linux `5.10.160` plus `-rt89`, but its history says it integrated `patch-5.10.180-rt89.patch` on a Rockchip base and then added Rockchip runtime/latency fixes. The project therefore:

- builds the exact vendor RT commit as one inseparable source tree;
- does not download and reapply the upstream patch during the normal build;
- records the upstream patch hash only as provenance;
- records the three integration commits and the reported kernel release;
- treats any attempt to replace the BSP or apply a different RT patch as a new qualification baseline.

This is suitable as a pinned bring-up baseline, not evidence that the vendor tree has the maintenance or security properties of an exact upstream `5.10.180-rt89` tree.

### 3.3 Kernel configuration

The complete board configuration is owned by the pinned Orange Pi build repository, not by merely concatenating the two smaller configs in the kernel tree. The build composes:

```text
orangepi-build/external/config/kernel/linux-rockchip-rk3588-legacy-rt.config
    + project rs-control-rt.fragment
    -> make olddefconfig
    -> verify required.config against final .config
```

The pinned complete vendor config has SHA-256 `4a275a472c81ecb7012df46ece711d72a6aebd86fe59b4350a729e912e6b1244`. The build verifies that hash before using it, then applies the project fragment last. `required.config` is an independent assertion list, so a later Kconfig rename, unmet dependency or vendor override is detected rather than silently accepted.

Critical requirements include PREEMPT_RT, high-resolution timers, HZ=1000, CPU isolation/cpuset support, IKCONFIG, SocketCAN/error reporting, Rockchip CAN, V4L2/DMABUF, USB and `CONFIG_CPU_FREQ_THERMAL=y`.

### 3.4 Build and installation

`build-kernel.sh` supports:

- `--verify-only`: validate lock/config/script invariants without network;
- `--print-plan`: print immutable source identities and commands without mutation;
- `--fetch`: populate an explicit work directory and verify downloads;
- `--build`: build out-of-tree with a pinned cross-toolchain and produce Debian packages;
- explicit `--work-dir`, `--output-dir` and `--jobs`.

It never installs packages on the host or target. Installation remains a documented operator step after serial recovery and a bootable fallback kernel are confirmed.

The pinned container is an optional clean build environment, not a hidden second build definition. The same entry script runs inside it.

## 4. Runtime CPU and IRQ model

### 4.1 Roles, not hard-coded assumptions

The initial RK3588 candidate is:

```text
CPU 0-5  housekeeping / normal services (candidate)
CPU 6    CAN IRQ + CAN I/O RT thread (candidate)
CPU 7    motion-control RT thread (candidate)
```

This matches the intended separation of hardware I/O from control computation, but is not accepted until the board reports online CPU IDs, CPU capacity/frequency clusters and benchmark results. A profile is rejected if:

- any CPU is offline or outside the process's allowed cpuset;
- control and housekeeping overlap;
- no housekeeping CPU remains;
- the CAN IRQ CPU is absent from the I/O role;
- an exact single-CPU role expands or collapses unexpectedly;
- apply-time readback differs from requested affinity.

The robot topology YAML does not contain these fields. They belong to the platform profile.

### 4.2 Exact affinity semantics

Horus exposes a useful node-level affinity concept, but its reviewed `pin_to_cores` implementation iterates candidates and returns after the first successful single-core pin. This project distinguishes:

- `cpu = N`: exactly one Linux logical CPU;
- `cpu_set = LIST`: exactly the entire requested mask;
- `candidate_cpus = LIST`: choose one only through an explicit measured selection step.

No API silently interprets one form as another. The runtime setup path must read back `/proc/self/task/<tid>/status` or `sched_getaffinity` before reporting success. The future Rust RT runtime will isolate its Linux `unsafe` syscall boundary and expose typed errors.

### 4.3 IRQ policy

The platform does not copy Horus's broad “move every IRQ off all RT CPUs” loop. IRQ policy is role-aware:

- general managed IRQs and normal services stay off the control-only CPU;
- the selected CAN IRQ is explicitly placed on the CAN I/O CPU;
- critical hardware IRQ placement is recorded individually;
- all writes to `smp_affinity_list` are verified by readback;
- inability to find or move the required CAN IRQ prevents a qualified profile;
- no affinity is changed while motion is armed.

Candidate boot isolation/cpuset strategies are compared on target. `nohz_full` is not enabled unless the selected kernel actually supports it and it improves the complete CAN/control workload. RT throttling remains enabled by default to preserve a recovery window for housekeeping.

### 4.4 Privilege and failure mode

One small systemd oneshot unit may configure the scheduler budget, governor and
the selected CAN IRQ before motion services. It has the required privileges;
algorithm, middleware and recorder processes do not. Exact application-thread
affinity is owned and read back by the future Rust runtime.

The qualified profile is fail-closed. Missing PREEMPT_RT, memory-lock permission, FIFO scheduling, required CPU/IRQ readback or thermal instrumentation prevents the qualified service target from starting. A separate development mode may continue only while emitting a persistent `unqualified` status.

## 5. SocketCAN platform boundary

### 5.1 Device tree and wiring

The bring-up guide supports the official overlays:

- CAN0: `rk3588-can0-m0`, TX physical pin 5, RX physical pin 3;
- CAN1: `rk3588-can1-m0`, TX physical pin 18, RX physical pin 16.

The controller pins are logic-level TX/RX and must connect to a 3.3 V-compatible isolated CAN transceiver. They are not CANH/CANL. CANH/CANL receive 120 Ω termination only at the two physical bus endpoints.

The exact transceiver model, board revision, isolation rating, supply and termination state are evidence fields because they are not yet selected.

### 5.2 Link management

The platform manager has explicit operations:

```text
plan       print intended commands and validation
up         configure a known-down/unconfigured link at 1 Mbps
status     read details, counters and restart policy
recover    require an explicit disarmed confirmation, then cycle the link
down       require an explicit disarmed confirmation
```

The effective link contract is classic CAN, bitrate 1,000,000, bus-error reporting enabled and `restart-ms 0`. Automatic bus-off recovery is deliberately disabled so a later motor lifecycle can revoke authority before recovery. The Rust hardware_module driver will own frame parsing and operational bus-off policy in its own child task.

## 6. Qualification and evidence

### 6.1 Profiles

Two explicit rates exist:

| Profile | cyclictest interval | Wake latency threshold | Status |
|---|---:|---:|---|
| `1khz` | 1,000 us | 100,000 ns max | preferred |
| `500hz` | 2,000 us | 100,000 ns max | fallback, separately qualified |

`cyclictest` runs one measurement thread with absolute timers (default), FIFO policy, priority 90, `mlockall`, nanosecond output, exact affinity and a main thread on housekeeping. Raw JSON is authoritative. Parsing requires `return_code == 0`, `resolution_in_ns == 1`, at least one cycle and every thread `max <= threshold`.

The cyclictest result qualifies kernel wakeup behavior, not application deadline behavior. The future `motion-rt` task adds execution time/deadline counters; final qualification requires both.

### 6.2 Test matrix

Qualification has separately labelled runs:

1. baseline idle/safety-services run;
2. CPU and memory stress pinned to housekeeping CPUs;
3. target-storage write/sync pressure;
4. real Ethernet bidirectional load with an identified peer;
5. representative NPU/GPU workload;
6. all stressors concurrently for an eight-hour soak.

Missing a real peer, accelerator runner, temperature source or stress process completion makes the concurrent run `incomplete`. A lower result from a partial matrix is still useful research data but is not a qualification pass.

### 6.3 Thermal rule

Active cooling and stable power are mandatory. Evidence samples thermal zones and cpufreq policy/current/max values throughout the run. A performance-governor RT cluster that drops below its requested maximum, a thermal trip/throttle indicator, sensor loss or an operator-declared thermal event invalidates the run. The raw trace is retained even when the verdict is fail/incomplete.

### 6.4 Evidence bundle

Each run uses a new caller-chosen directory; an existing non-empty directory is rejected. It contains at minimum:

```text
platform/
  sources.lock / source-lock.sha256
  manifest.env / manifest.sha256
  hardware-evidence.env / rt-profile.env / can-overlay.dtbo
  kernel-*.txt / device-tree-*.txt / cpu-topology.txt
  rt-affinity.txt / can-*.txt / network-*.txt / tool-version files
  collection-status.env / SHA256SUMS
cyclictest.json / cyclictest.stdout / cyclictest-verdict.env
stress-ng.txt / iperf3.txt / accelerator.txt / *-status.env
thermal.csv / optional thermal event and marker files
rt-affinity-check.txt / verdict.env / SHA256SUMS
```

The verdict is one of `pass`, `fail` or `incomplete` with reason codes. Tools never overwrite the raw inputs when calculating a verdict.

## 7. Horus-derived protocol-module follow-up

The reviewed Horus commit provides useful patterns for the future `zenoh-protobuf-middleware` child:

- a matrix of per-publisher/per-subscriber SPSC channels avoids a shared MPMC hot spot;
- head/tail cache-line separation reduces false sharing;
- power-of-two capacity and monotonic indices simplify bounded rings;
- per-slot odd/even position stamps plus before/after validation reject torn reads during overwrite;
- OS-held `flock` plus owner metadata gives a recoverable cross-process endpoint lease;
- slow-consumer behavior is topology/semantic specific rather than one global policy.

The project will not copy these aspects unchanged:

- `needs_drop::<T>() == false` is not a POD proof; raw pointers, invalid enum discriminants, `NonZero*`, padding and ABI differences remain unsafe.
- fixed-size endpoint matrices require capacity planning and explicit exhaustion metrics.
- drop-oldest/latest-wins needs a dropped-sequence counter; a silent overwrite is insufficient observability.
- serialized messages that allocate on receive are not allowed in the 1 kHz path.
- shared-memory layout needs project magic/version/schema identity, architecture/endianness policy and clean incompatibility rejection.
- file-lock ownership is a cold-path lifecycle mechanism, never a hot-path queue lock.

The eventual implementation stays in the protocol_module layer. application_module algorithms, including chassis/arm/vision, continue to consume protocol_module interfaces and never open shared-memory files directly.

## 8. Rollout and rollback

1. Run host verification and print the immutable build plan.
2. Build packages in a clean pinned container or documented equivalent host.
3. Confirm serial console and current vendor-kernel boot entry.
4. Install new packages without removing the old kernel; add a separate boot entry.
5. Boot and collect a pre-tuning evidence bundle.
6. enable one CAN overlay and validate with an analyzer/loopback, no motor.
7. compare affinity candidates and run the full qualification matrix.
8. publish only the exact package/profile/evidence hash combination that passed.

Rollback selects the retained vendor kernel and prior device-tree entry, restores the previous affinity/governor/CAN bundle, and preserves all failed evidence. No rollback step deletes user data or reflashes the bootloader.

## 9. Compatibility

- RDK X5 gets its own `platform/rdkx5/` implementation of the same evidence/profile contracts.
- A later BSP/kernel upgrade creates a new source-lock identity and cannot reuse old timing evidence.
- Later application services may consume the profile and evidence format but cannot mutate platform configuration from RT threads.
- Later network security can change service credentials without changing the realtime data path.
