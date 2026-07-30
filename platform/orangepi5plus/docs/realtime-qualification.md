# Realtime qualification

## Result scope

This phase measures kernel wake latency. It does not yet prove that the future
motion-control loop completes every application deadline. The later Rust motion
runtime must add execution-time, overrun and missed-deadline counters. Final
robot qualification requires both evidence sets.

Two explicit timing profiles exist:

| Profile | Interval | Maximum accepted wake latency |
|---|---:|---:|
| `1khz` | 1,000 us | 100,000 ns |
| `500hz` | 2,000 us | 100,000 ns |

The 500 Hz profile is a separately tested fallback. No script selects it
automatically after a 1 kHz failure.

## Required target inputs

Copy `config/hardware-evidence.example.env`, fill every `unrecorded` value and
set `SERIAL_CONSOLE_VERIFIED=1`. Every passing run requires that file, the exact
active CAN overlay and a non-placeholder build identity. A complete concurrent
run also requires:

- an identified real Ethernet peer;
- the physical Ethernet interface that must route to that peer;
- a target-storage directory;
- an executable representative NPU/GPU workload runner;
- active cooling and valid thermal/frequency sensors.

The accelerator runner receives `RS_QUALIFICATION_DURATION_SECONDS` and must
sustain the representative workload for that duration. It also receives
`RS_QUALIFICATION_THERMAL_EVENT_FILE`; the runner or operator may create that
file when an external thermal/throttle indicator is observed. Every stressor is
pinned to housekeeping CPUs; CAN I/O and the timing thread retain their exact
roles.

## Baseline research run

```bash
sudo platform/orangepi5plus/scripts/qualify-rt.sh \
  --profile 1khz \
  --mode baseline \
  --duration 10m \
  --output /var/lib/rs-control/qualification/<unique-baseline-id> \
  --hardware-manifest /absolute/path/hardware-evidence.env \
  --overlay-file /absolute/path/rk3588-can0-m0.dtbo \
  --app-build-id <immutable-build-id>
```

A baseline result is useful for tuning but does not replace the complete stress
matrix.

## Complete eight-hour run

```bash
sudo platform/orangepi5plus/scripts/qualify-rt.sh \
  --profile 1khz \
  --mode concurrent \
  --duration 8h \
  --output /var/lib/rs-control/qualification/<unique-concurrent-id> \
  --hardware-manifest /absolute/path/hardware-evidence.env \
  --overlay-file /absolute/path/rk3588-can0-m0.dtbo \
  --network-peer <ethernet-test-peer> \
  --network-interface <ethernet-interface> \
  --accelerator-runner /absolute/path/validated-accelerator-runner \
  --storage-path /absolute/path/on-target-storage \
  --app-build-id <immutable-build-id>
```

The wrapper uses cyclictest FIFO priority 90, memory lock, nanosecond JSON, one
measurement CPU and a separate housekeeping main thread. It requires at least
95 percent of the expected timing cycles, verifies the CPU reported in JSON and
rejects a maximum above `100000 ns`. Pressure includes CPU, memory, target
storage, bidirectional Ethernet and the accelerator workload. A run shorter than
8 hours still executes available stressors but is always `incomplete`.

## Verdicts

`verdict.env` has one of three states:

- `pass`: kernel wake latency, run duration, required evidence, stressors and
thermal/frequency checks all passed for the selected profile and mode;
- `fail`: maximum latency exceeded the threshold, a passive/hot/critical thermal
  trip was reached, a frequency drop was observed or an operator thermal event
  was recorded;
- `incomplete`: evidence, cycles, exact CPU placement, a tool, pressure source,
  duration or monitor result was missing or invalid.

Raw JSON, command output, load status, thermal samples, platform observations and
`SHA256SUMS` remain in the caller-chosen directory. The evaluator does not edit
raw evidence. Reusing a non-empty output directory is rejected.

To evaluate retained cyclictest JSON without running a test:

```bash
platform/orangepi5plus/scripts/evaluate-cyclictest.py \
  /absolute/path/cyclictest.json \
  --threshold-ns 100000 \
  --expected-cpu 7
```

Do not mark `PROFILE_QUALIFIED=1` until the complete target evidence bundle has
been reviewed and its immutable aggregate hash is placed in
`QUALIFICATION_EVIDENCE_SHA256`.

## Middleware candidate qualification

Build the project `middleware-qualification` executable from a clean exact
commit and place the pinned Horus checkout outside the project checkout. The
reviewed adapter source and its lock file live under `horus-adapter/`, outside
the project Rust workspace; the built executable is an external research tool
and does not add Horus to the robot workspace or runtime graph. Build it only
through the checked-in offline entry point:

```bash
platform/orangepi5plus/scripts/build-horus-adapter.sh --verify-only
platform/orangepi5plus/scripts/build-horus-adapter.sh \
  --horus-source /absolute/path/clean/pinned-horus \
  --output /absolute/path/absent/reviewed-horus-adapter
```

The builder verifies the exact source lock, clean checkout and pinned commit,
renders the external path only in an isolated temporary build tree, uses
`cargo build --offline --locked`, embeds the actual commit in the binary and
creates the requested output without overwriting. On an x86 development host,
the built binary can exercise its real separate-process payload/fault paths
without writing qualification evidence:

```bash
/absolute/path/reviewed-horus-adapter self-test \
  --source-commit fac3cfb8704e275bde9daec6d554a17c59dd8532
```

This prints `target_qualification=false`, skips the target memory-lock claim,
and is forbidden on aarch64. It cannot replace target preflight or evidence.
The adapter's `preflight` subcommand emits
schema-v1 JSON proving the exact source/target, power-of-two capacity, complete
payload/kind matrix, explicit `repr(C)` bytemuck-POD layout, Linux monotonic
clock, `try_send`/`try_recv` hot path, successful
`mlockall(MCL_CURRENT|MCL_FUTURE)` and fault-suite readiness. Its `benchmark`
subcommand emits
the same schema-v2 timing/counter/CPU/memory fields as the project runner plus
`candidate=horus`, the exact `source_commit`, target identity and effective
lock-required policy. Each benchmark process must acquire and report its own
process-wide memory lock. Its `fault-suite` subcommand emits schema v1 with the
pinned commit, target architecture, and explicit `producer_bounded`,
`no_torn_values`, `crash_reclaim`, and `losses_accounted` booleans.
It must also report explicit slow/stopped consumer, full-queue, clean-exit and
killed-peer outcomes; omission is not treated as a passing comparison.

The adapter must not wrap the repository's bundled `all_paths_latency`
benchmark: that program fixes different payloads/iterations/CPU IDs and uses a
non-monotonic fallback outside x86. Comparison capacity is restricted to a
power of two in `16..=1024`; the pinned custom-capacity path rounds allocation
up but retains the request in its shared header, so other values are not exact
comparisons. The retained semantic report also states that Horus's generic
`needs_drop` POD heuristic is not the adapter payload's layout proof.

First inspect the side-effect-free plan:

```bash
platform/orangepi5plus/scripts/qualify-middleware.sh \
  --profile 1khz --mode concurrent --duration 8h \
  --capacity 64 --warmup 10000 --samples 1000000 --dry-run
```

Then run on the identified Orange Pi 5 Plus as a user with FIFO, memory-lock
and perf-counter authority:

```bash
sudo platform/orangepi5plus/scripts/qualify-middleware.sh \
  --profile 1khz --mode concurrent --duration 8h \
  --output /var/lib/rs-control/qualification/<unique-middleware-id> \
  --runner /absolute/path/middleware-qualification \
  --workspace /absolute/path/clean/rs_control \
  --app-commit <full-clean-git-sha> --app-build-id <immutable-build-id> \
  --hardware-manifest /absolute/path/hardware-evidence.env \
  --overlay-file /absolute/path/rk3588-can0-m0.dtbo \
  --horus-source /absolute/path/clean/pinned-horus \
  --horus-runner /absolute/path/reviewed-horus-adapter \
  --network-peer <ethernet-test-peer> \
  --network-interface <ethernet-interface> \
  --accelerator-runner /absolute/path/validated-accelerator-runner \
  --storage-path /absolute/path/on-target-storage
```

The wrapper first refuses the wrong architecture/device tree, dirty or wrong
source revisions, invalid hardware/profile identity, an existing output, a
failed FIFO + lock-required + hardware-cycle preflight, or invalid Horus
preflight evidence. The Horus preflight runs in a temporary directory and only
successful validated evidence is moved into the new bundle. It then runs
latest and ordered semantics at 64/256/1024 bytes for local-thread, project
SHM-thread, project SHM-process, and pinned Horus under the same
CPU/priority/config while
the platform thermal/stress envelope is live. Each result has a paired raw
`perf stat` file with cycles, instructions, task-clock, context switches, CPU
migrations and page faults. Process CPU time and peak resident memory are also
reported separately for the project SHM peer.

The wrapper owns each long command and the platform envelope in a separate
`setsid` session. HUP, INT, or TERM stops the whole owned process group,
escalates from TERM to KILL after a bounded grace period, reaps the direct
child, and exits with 129, 130, or 143. An interruption after output creation
leaves an intentionally unsealed diagnostic directory. Do not resume it or add
a manifest manually; preserve it and retry with a new absent output path.

`comparison.json` retains distributions, maxima, jitter span, counters,
allocation totals, CPU/memory and perf observations. Safety failures dominate;
missing evidence is `incomplete`, not a favorable ranking. `verdict.env` also
includes the platform verdict, and `SHA256SUMS` covers the whole bundle. A 1 kHz
failure is preserved. Testing 500 Hz requires a separate explicit run with
`--profile 500hz` and a different absent output directory; no fallback or
profile promotion is automatic.

From a clean checkout of the reviewed application commit, review a sealed
bundle independently before using its digest:

```bash
platform/orangepi5plus/scripts/verify-middleware-evidence.py \
  --evidence /var/lib/rs-control/qualification/<unique-middleware-id> \
  --expected-app-commit <reviewed-full-clean-git-sha>
```

The path must be absolute. The verifier checks the exact file set and hashes at
every nested `SHA256SUMS`, requires each parent manifest to anchor child
manifests, requires the bundle source lock to match the reviewed checkout,
validates profile/hardware identities and the Horus preflight,
re-runs `evaluate-middleware.py` against the raw results, and derives the
wrapper status/reason again. It exits zero when a sealed bundle is internally
valid even if its reported qualification is `fail` or `incomplete`; selection
still requires the printed `qualification_status=pass`. The printed
`evidence_sha256` is the SHA-256 of the top-level `SHA256SUMS` and must be
recorded outside the bundle before any profile promotion. Subsequent reviews
add `--expected-sha256 <externally-recorded-digest>` so a fully re-sealed but
different bundle is also rejected.

The project `thread` rows use a real separate consumer thread with an atomic
per-sample acknowledgement outside the timed producer call; they are not
single-thread loopback measurements. The process row and Horus adapter declare
`peer_execution=separate_process_paired` and preserve peer CPU/memory evidence.
