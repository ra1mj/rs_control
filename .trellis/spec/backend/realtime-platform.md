# Realtime Platform Contracts

## Scenario: Change a qualified Linux board profile or its evidence path

### 1. Scope / Trigger

Use this contract for code under `platform/`, service units that prepare a board,
realtime thread/IRQ placement, SocketCAN link preparation, kernel inputs or
timing qualification. Host fixtures validate behavior but never establish a
hardware or realtime performance claim.

### 2. Signatures

The Orange Pi baseline exposes these command contracts:

```text
build-kernel.sh --verify-only|--print-plan|--fetch|--build
rt-affinity.sh plan|check|apply|snapshot [--require-qualified]
can-link.sh plan|check|up|status|recover|down
collect-evidence.sh --output <absolute-new-directory> [evidence inputs]
qualify-rt.sh --profile 1khz|500hz --mode baseline|concurrent \
  --duration <time> --output <absolute-new-directory> [stress/evidence inputs]
evaluate-cyclictest.py <json> --threshold-ns <n> \
  [--expected-cpu <id>] [--minimum-cycles <n>]
```

`plan`, `--verify-only`, `--print-plan`, host tests and JSON evaluation are
side-effect-free. `apply`, CAN reconfiguration and target installation are
explicit operator operations and never run from ordinary CI.

Host rollback tests may use `--fixture-nonroot` only when both fixture roots are
absolute and differ from `/sys` and `/proc`; the option cannot grant permission
to modify real kernel state. The rollback regression may additionally inject a
post-governor failure, and that injection also refuses the real roots.

### 3. Contracts

Runtime profile keys are data-only assignments:

```text
PROFILE_FORMAT_VERSION, PROFILE_NAME, PROFILE_QUALIFIED,
QUALIFICATION_EVIDENCE_SHA256, HOUSEKEEPING_CPUS, CAN_IO_CPUS,
CONTROL_CPUS, CAN_IRQ_CPUS, RT_GOVERNOR, RT_RUNTIME_US, RT_PERIOD_US,
REQUIRE_CONTROL_MAX_CAPACITY, REQUIRE_IO_MAX_CAPACITY
```

- CPU values are exact Linux logical CPU sets. A candidate list is not an exact
  mask and is never implemented as “the first core that succeeds”.
- Housekeeping, CAN I/O and control roles are disjoint; the selected CAN IRQ set
  is a subset of the CAN I/O set. Online CPUs, allowed cpuset, capacity,
  PREEMPT_RT, governor, scheduler budget and IRQ affinity are read back.
- A production service uses `--require-qualified`. A profile with
  `PROFILE_QUALIFIED=0` and `QUALIFICATION_EVIDENCE_SHA256=unqualified` is only a
  candidate, even after all host tests pass.
- Classic CAN is exactly 1,000,000 bit/s with bus-error reporting and
  `restart-ms 0`. No platform command transmits a frame or operates a motor.
- Qualification evidence uses a caller-chosen absent/empty directory, preserves
  raw JSON and observations, emits `pass|fail|incomplete`, and hashes its files.
  Kernel wake latency and application deadline evidence remain separate.
- A complete concurrent run is at least eight hours and includes CPU, memory,
  target storage, real bidirectional Ethernet with identified interface, driver,
  carrier and peer route, and a representative accelerator workload plus
  thermal/frequency monitoring.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Source commit/hash/config mismatch | Abort before fetch/build |
| Output directory already contains data | Refuse overwrite |
| CPU role overlap/offline/disallowed/low-capacity RT role | Refuse check/apply |
| CAN IRQ missing or readback differs | Refuse qualified preparation |
| Apply fails after a write | Attempt transactional rollback; keep motion disarmed on rollback error |
| Active mismatched CAN link without disarmed confirmation | Refuse reconfiguration |
| Missing stress tool/input/evidence or too few timing cycles | `incomplete` |
| Maximum cyclic latency above threshold | `fail` |
| Frequency drop, passive/hot/critical trip or operator thermal event | `fail` |
| 1 kHz failure | Preserve evidence; never switch rate automatically |

### 5. Good/Base/Bad Cases

- Good: an immutable Orange Pi build/profile/DT identity completes an eight-hour
  1 kHz concurrent run at `<= 100000 ns`, with full stress and no thermal event.
- Base: a one-minute fixture or target research run executes every available
  stressor and reports `incomplete` because it is below eight hours.
- Bad: label host fixture output as RK3588 qualification, broaden CPU `7` to
  “one of 6,7”, move every IRQ away from every RT role, auto-recover bus-off or
  silently fall back to 500 Hz.

### 6. Tests Required

- `platform/orangepi5plus/tests/host-test.sh` asserts safe plans, strict data
  parsing, CPU role conflicts, exact IRQ/governor readback, CAN recovery gates,
  systemd privilege separation, thermal markers, evidence non-overwrite and
  concurrent partial-run behavior.
- Cyclictest fixtures assert `100000 ns` pass, `100001 ns` fail, integer/unit
  strictness, expected CPU and minimum cycle coverage.
- `tools/ci.sh` includes the host test but performs no fetch, kernel build, root
  write, CAN transmit or target mutation.
- Real kernel boot, analyzer CAN traffic, topology/IRQ behavior and the complete
  eight-hour matrix remain target-board gates and keep the Trellis task active.

### 7. Wrong vs Correct

#### Wrong

```text
for cpu in candidate_cpus:
    if pin(cpu) succeeds: return success
PROFILE_QUALIFIED=1  # set after a host fixture pass
```

#### Correct

```text
set_affinity(exact_cpu_set)
read_back_affinity() == exact_cpu_set or refuse startup
PROFILE_QUALIFIED=1 only with reviewed immutable target evidence SHA-256
```
