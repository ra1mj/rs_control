# Target Qualification Blockers

## Status

Recorded on 2026-08-18 for the coordinating task's acceptance criterion: an
active task may remain active only when its unmet external criterion and the
required evidence are explicit.

The current execution host reports `x86_64` with Ubuntu generic kernel
`6.8.0-136-generic`; `ip -brief link show` reports no CAN interface. The
working tree contains target procedures and example manifests, but no sealed
Orange Pi/RK3588 qualification bundle. This is an observation of this
environment and checkout, not a claim about hardware that may exist elsewhere.
Host results therefore remain host-only evidence.

## Completed Host Deliveries

The following task archives contain the completed host work that precedes the
remaining target gates:

| Delivery | Archive status |
|---|---|
| Arm reactive/task/performance | Archived 2026-08-18 |
| Advanced arm algorithms parent | Archived 2026-08-18 |
| Multi-morphology arm stack parent | Archived 2026-08-18 |
| KINETIC-informed arm algorithms parent | Archived 2026-08-18 |
| Behavior-tree motion participants | Archived 2026-08-18 |
| Host motion runtime and telemetry recording | Archived 2026-08-18; host verification in `08-18-host-motion-runtime-recording/verification.md` |

These archives do not qualify an ARM board, a real CAN link, a motor, a
physical stop path, or a realtime profile.

## Active External Gates

| Task | Exact unmet criterion | Evidence required to unblock |
|---|---|---|
| `07-29-orangepi-rt-platform` | The Orange Pi 5 Plus realtime platform has not completed its target boot, SocketCAN, IRQ/affinity/governor, thermal, and duration qualification. A 1 kHz claim requires maximum wakeup jitter at most 100 us and no deadline misses during the agreed soak. If it fails, 500 Hz must be qualified by a separate complete matrix. | Immutable board revision, power/cooling, Ubuntu 22.04 + pinned Rockchip 5.10/PREEMPT_RT kernel and device-tree identities; CAN transceiver/termination and `can0`/`can1` proof; read-back governor, scheduler, CPU and IRQ placement; raw baseline and concurrent CPU/memory/storage/Ethernet/accelerator/thermal results; eight-hour evidence bundle with manifest; motor-disabled and safety-fixture physical safe-stop records where applicable. |
| `07-30-rk3588-middleware-qualification` | No target candidate matrix or backend/profile decision exists. The task cannot select a profile from host benchmarks. | Recorded Orange Pi identity plus exact source, build, scheduler, affinity and memory-lock identities; comparable baseline and concurrent results for local, project SHM and pinned Horus candidates; latest/ordered, 64/256/1024-byte, normal/slow/stopped/killed peer cases; stress/thermal observations; raw outputs and a complete immutable SHA-256 manifest. The selected backend/profile must cite independently reviewed passing evidence, otherwise remain `failed` or `incomplete`. |
| `07-30-local-shm-rk3588-qualification` | The host SHM child is complete, but the parent cannot close LSQ-11 through LSQ-14 without its middleware child completing target comparison and choosing an evidence-backed profile. | The middleware child's sealed Orange Pi baseline and concurrent evidence, including the chosen backend/profile digest. Reconcile that digest with the parent's required board, kernel/RT patch, device tree, toolchain, thermal, allocation, loss, crash and maximum-latency evidence. |
| `07-29-rust-robot-framework` | The integration parent remains open after host software delivery because its target platform, physical motor/CAN, ARM networking and system safety gates are unmet. | Evidence closing the Orange Pi platform and middleware tasks, then real ARM SocketCAN/DM-J4310-2EC discovery, enable/disable, feedback, timeout and explicit-reset tests; ARM network failure/recovery evidence; target stress results for motion timing, recorder backpressure and safe stopping; and only then the complete integration acceptance review. |

## Required Order

1. Provision the target Orange Pi environment and collect immutable platform
   identity, cooling, kernel and device-tree information.
2. Run the platform baseline and full concurrent qualification. Preserve a
   failed 1 kHz attempt; run a new, independently complete 500 Hz matrix only
   if that fallback is needed.
3. Run the middleware candidate baseline and concurrent matrices using the
   qualified platform procedure and seal the evidence bundle.
4. Reconcile the local-SHM parent with the chosen backend/profile digest.
5. Execute the framework's physical CAN/motor, network and safe-stop
   integration gates with the required safety fixture.

No profile, motor behavior or task status is promoted by this record.
