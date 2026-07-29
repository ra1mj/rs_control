# Orange Pi 5 Plus / RK3588 platform baseline

> **Candidate, not hardware-qualified.** The checked-in CPU map and realtime
> profile are host-verified inputs only. They must not be described as a
> production 1 kHz result until an Orange Pi 5 Plus completes the target-board
> evidence gate and the profile contains that immutable evidence hash.

This directory owns the first deployment baseline for Ubuntu 22.04 Server on
Orange Pi 5 Plus. It covers pinned kernel inputs, PREEMPT_RT configuration,
classic SocketCAN preparation, exact CPU/IRQ placement, evidence collection and
kernel wake-latency qualification. It never sends a CAN frame, connects a motor,
enables torque or starts a robot algorithm.

## Safe host checks

```bash
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/kernel/build-kernel.sh --print-plan
platform/orangepi5plus/scripts/rt-affinity.sh plan
platform/orangepi5plus/scripts/can-link.sh plan
platform/orangepi5plus/tests/host-test.sh
```

These commands do not fetch source, build a kernel, write `/sys` or `/proc`,
configure a real network link, send CAN traffic or require root.

## Directory map

| Path | Responsibility |
|---|---|
| `sources.lock` | Immutable source, toolchain, container, rt-tests and Horus identities |
| `kernel/` | Verified vendor config overlay and package build entry point |
| `config/` | Candidate runtime/CAN profile and hardware evidence template |
| `scripts/` | Explicit link, affinity, evidence, thermal and qualification tools |
| `systemd/` | Fail-closed privileged preparation separated from applications |
| `tests/host-test.sh` | Fixture-only platform regression gate |
| `docs/` | Build, bring-up, qualification and rollback runbooks |

## Runbooks

- [Pinned kernel build and guarded installation](docs/build-and-install.md)
- [Board, overlay, CAN and runtime bring-up](docs/bring-up.md)
- [Realtime qualification and verdict interpretation](docs/realtime-qualification.md)
- [Rollback and evidence retention](docs/rollback.md)

## Safety boundary

The production systemd preparation service uses `--require-qualified`; it
therefore refuses the checked-in candidate profile. Candidate affinity testing
is a separate, manual, disarmed operation. Only the preparation units receive
the privileges needed to change scheduler parameters, CPU governors, IRQ
affinity or SocketCAN state. Middle, upper, FSM, recorder and bridge processes
must not receive those privileges.

The target kernel reports `5.10.160-rt89` from its source Makefile. The vendor
history also records integration from the upstream `5.10.180-rt89` patch. The
source lock preserves both facts; this project does not relabel the vendor tree
as upstream `5.10.180-rt89`.

## Horus design input

Horus is pinned only as an Apache-2.0 research input, not a runtime dependency.
This platform adopts exact logical-CPU semantics and mandatory affinity
readback. The future middle-layer IPC task carries forward the useful
publisher-by-subscriber SPSC layout, cache-line separation, per-slot sequence
validation and crash-reclaim ideas. It must use an explicit fixed-layout shared
memory contract; Rust `needs_drop::<T>() == false` is not accepted as proof that
a type is safe for cross-process memory.
