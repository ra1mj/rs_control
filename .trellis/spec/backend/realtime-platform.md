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
middleware-qualification --backend local|shm --kind latest|ordered \
  --peer thread|process --payload-bytes 64|256|1024 \
  --capacity <n> --memory-policy prefault|lock-required \
  --warmup <n> --samples <n> [--output <absent-file>]
qualify-middleware.sh --profile 1khz|500hz --mode baseline|concurrent \
  --duration <time> --output <absolute-new-directory> \
  --runner <file> --workspace <dir> --app-commit <sha> \
  --app-build-id <token> --hardware-manifest <file> --overlay-file <file> \
  --horus-source <dir> --horus-runner <file> [stress/evidence inputs]
reviewed-horus-adapter preflight|benchmark|fault-suite \
  --source-commit <sha> --capacity <power-of-two-16..1024> \
  --payload-bytes <64|256|1024|64,256,1024> \
  --memory-policy lock-required --payload-contract repr_c_bytemuck_pod \
  --output <absent-file> [row-specific inputs]
verify-middleware-evidence.py --evidence <absolute-sealed-directory> \
  --expected-app-commit <reviewed-sha> [--expected-sha256 <external-digest>]
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
- The privileged platform unit alone prepares link state/bitrate and performs
  any approved bus-off recovery. The unprivileged adapter opens an explicit
  already-configured interface, uses close-on-exec/nonblocking socket-local
  options, and never invokes link configuration.
- Host SocketCAN/Damiao codec and fake-I/O lifecycle tests do not qualify the
  installed firmware, physical timeout/disable behavior, analyzer-visible wire
  timing, or RK3588 scheduling. Those claims require a separately approved,
  restrained target procedure and immutable evidence.
- Qualification evidence uses a caller-chosen absent/empty directory, preserves
  raw JSON and observations, emits `pass|fail|incomplete`, and hashes its files.
  Kernel wake latency and application deadline evidence remain separate.
- A complete concurrent run is at least eight hours and includes CPU, memory,
  target storage, real bidirectional Ethernet with identified interface, driver,
  carrier and peer route, and a representative accelerator workload plus
  thermal/frequency monitoring.
- `middleware-qualification` is a bounded evidence producer, not a profile
  mutator. Schema v2 deterministic JSON remains labeled
  `qualification=host_research`; it separately records `CLOCK_MONOTONIC`
  one-way receive and `try_publish` distributions/maxima, min/max jitter span,
  runner/peer process CPU time, runner/peer `VmHWM`, hot allocation totals and
  delivery counters. Thread/process consumers are separately executed and both
  producer and consumer hot allocations are included. Requested memory policy is distinct from effective policy:
  local-thread is `not-applicable`, while target SHM must be `lock-required`.
- `qualify-middleware.sh` is the only target promotion envelope. It refuses a
  non-aarch64/non-Orange-Pi-5-Plus/RK3588 host, dirty or wrong source revisions,
  an existing output, invalid platform identity/affinity, an unavailable FIFO/
  memory-lock runner preflight, or unavailable `perf` hardware cycles. It runs
  the pinned external Horus adapter without fetching source, first validates
  its schema-v1 preflight before evidence-directory creation, collects identical
  `perf stat` events for every candidate, composes `qualify-rt.sh`, and hashes
  the retained bundle. Horus comparison capacity is an exact power of two in
  `16..=1024`; every adapter process reports successful process-wide
  `mlockall`, explicit `repr(C)` POD layout, monotonic clock and
  `try_send`/`try_recv` hot path. The generic Horus `needs_drop` POD heuristic
  remains a recorded semantic difference, never layout proof. The envelope
  never edits a runtime profile.
- Every evidence generator hashes all regular files below its output except
  only its own `./SHA256SUMS`; parent manifests therefore anchor nested
  manifests. `verify-middleware-evidence.py` rejects missing/extra/reordered or
  changed files, symlinks and special files, verifies every nested manifest,
  requires the bundle source lock to equal the reviewed verifier checkout,
  revalidates exact input/hardware/Horus contracts, regenerates
  `comparison.json` from raw results and re-derives the wrapper verdict. A
  valid sealed `fail`/`incomplete` bundle verifies successfully but remains
  ineligible; only its printed status describes qualification. The printed
  aggregate digest is the SHA-256 of the top-level manifest. Review supplies
  the expected app commit and may supply a previously stored aggregate digest;
  neither trust anchor is taken from the bundle under review.
- A 1 kHz failure may only lead to a separate explicit `--profile 500hz` run in
  a new output directory. No script performs rate fallback or candidate
  promotion automatically.

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
| Zero/excessive middleware samples/capacity or unsupported payload size | Refuse before creating channels |
| Local backend requested with process peer | Refuse; only SHM is cross-process |
| Existing middleware evidence output | Refuse overwrite |
| Missing target `perf` cycles, lock/FIFO gate or valid Horus preflight | Refuse before evidence creation |
| Missing candidate JSON/`perf` pair or pinned Horus fault evidence | `incomplete` |
| Torn value, unaccounted loss, producer fault, hot allocation or invalid counted evidence | `fail` |
| Missing/changed/reordered evidence file, unanchored nested manifest or verdict mismatch | Offline verification failure |

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
- `platform/orangepi5plus/tests/middleware-host-test.sh` asserts target dry-run
  identity/lock/matrix policy, host refusal, complete/missing/bad evidence,
  deterministic comparison bytes, semantic mismatch, torn-fault rejection,
  nested-manifest anchoring, raw-result re-evaluation and tamper rejection.
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

# In a nested evidence directory, exclude only the manifest being written.
find . -type f ! -path './SHA256SUMS' ... > SHA256SUMS
verify every child SHA256SUMS before accepting the top-level manifest digest
```
