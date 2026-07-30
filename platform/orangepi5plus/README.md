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
platform/orangepi5plus/tests/middleware-host-test.sh
# After a target run, with an absolute bundle path:
platform/orangepi5plus/scripts/verify-middleware-evidence.py \
  --evidence /absolute/path/to/sealed-middleware-evidence \
  --expected-app-commit <reviewed-full-git-sha>
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
| `tests/middleware-host-test.sh` | Middleware target-plan and evidence-evaluator fixtures |
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
affinity or SocketCAN state. protocol_module, application_module, FSM, recorder and bridge processes
must not receive those privileges.

The target kernel reports `5.10.160-rt89` from its source Makefile. The vendor
history also records integration from the upstream `5.10.180-rt89` patch. The
source lock preserves both facts; this project does not relabel the vendor tree
as upstream `5.10.180-rt89`.

## Horus design input

Horus is pinned only as an Apache-2.0 research input, not a runtime dependency.
This platform adopts exact logical-CPU semantics and mandatory affinity
readback. The future protocol-module IPC task carries forward the useful
publisher-by-subscriber SPSC layout, cache-line separation, per-slot sequence
validation and crash-reclaim ideas. It must use an explicit fixed-layout shared
memory contract; Rust `needs_drop::<T>() == false` is not accepted as proof that
a type is safe for cross-process memory.

The target-only `scripts/qualify-middleware.sh` composes the existing platform
stress/thermal run with the project benchmark and an external pinned-Horus
adapter. `--dry-run` is host-safe. Actual execution refuses non-aarch64 or a
device tree other than Orange Pi 5 Plus/RK3588, requires lock-backed mappings,
clean exact source revisions and usable FIFO/`perf`/memory-lock policy, never
fetches source, writes only to an absent evidence directory, and never promotes
the candidate profile itself. A reviewed adapter `preflight` must prove the
explicit payload layout, `CLOCK_MONOTONIC`, `try_send`/`try_recv`, process-wide
`mlockall` and fault-suite readiness before that directory is created. See the
middleware section of the realtime qualification runbook for adapter and
evidence contracts. The offline verifier checks every nested and top-level
checksum manifest, re-evaluates raw middleware results, revalidates the Horus
preflight and derives the wrapper verdict again. It reports the SHA-256 of the
top-level manifest for external review; it never turns failed or incomplete
evidence into a pass. Long target commands run in isolated sessions. An
HUP/INT/TERM stops and reaps their complete process groups with bounded
TERM-to-KILL escalation; any partial directory is diagnostic-only and a retry
must use a new absent output path.
