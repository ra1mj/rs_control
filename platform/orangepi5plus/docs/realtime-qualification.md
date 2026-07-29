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
