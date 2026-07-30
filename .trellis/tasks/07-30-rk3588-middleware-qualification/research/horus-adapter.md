# Pinned Horus adapter research

Scope: source commit `fac3cfb8704e275bde9daec6d554a17c59dd8532`.
This is a source-level adapter contract, not target performance evidence.

## Delivered software adapter

The reviewed implementation now lives at
`platform/orangepi5plus/horus-adapter/` with its own manifest template and lock;
it is not a member of the application workspace. The only supported build entry
is `scripts/build-horus-adapter.sh`, which validates the clean pinned checkout,
renders its absolute path in a temporary build tree, builds offline/locked and
embeds the verified source commit. The binary implements the three evidence
subcommands below plus a non-aarch64 `self-test` that emits no evidence and
prints `target_qualification=false`.

Against the local clean pinned checkout, the standalone crate passes five unit
tests, all-target Clippy with warnings denied, release build and a real
separate-process self-test covering every payload size, slow/stopped consumers,
explicit full accounting, clean exit and SIGKILL replacement. These are
software gates only; `mlockall`, perf and RK3588 identity remain target gates.

## Why the bundled benchmark is not reusable

- `benchmarks/src/bin/all_paths_latency.rs` fixes its own iteration count,
  warm-up, message type and CPU IDs. CPU 8 is outside an eight-core RK3588,
  and its payload/configuration does not match the project matrix.
- `benchmarks/src/timing.rs` uses `RDTSC` on x86 and falls back to wall-clock
  `SystemTime` elsewhere. The comparison requires Linux `CLOCK_MONOTONIC` for
  both processes.
- The target comparison therefore needs a reviewed external adapter. It must
  use separate producer/consumer processes and the same 64/256/1024-byte,
  latest/ordered, capacity, warm-up and sample settings as the project rows.

## Capacity and hot-path contract

- `RingTopic::with_capacity` allocates with `capacity.next_power_of_two()` but
  initializes the shared header with the requested value. A non-power-of-two
  request therefore cannot be treated as an exact comparable capacity.
- The adapter accepts only powers of two in `16..=1024`, matching the pinned
  implementation's documented automatic-capacity bounds. It uses
  `try_send`/`try_recv`; the bounded-retry lossy `send` and sleeping
  `send_blocking` paths are excluded.

## Payload and memory contract

- `communication/pod.rs` treats a generic type as POD when it has no destructor
  and is larger than one byte. That heuristic is broader than a cross-process
  layout proof and is retained as a semantic difference.
- The adapter payload must be `repr(C)` and independently checked as bytemuck
  POD, with sequence and monotonic send time in the first two `u64` fields.
- The pinned Linux SHM implementation maps files under `/dev/shm` but does not
  lock the process mappings. Before creating topics, the adapter must call and
  verify `mlockall(MCL_CURRENT | MCL_FUTURE)` and report
  `memory_lock_mechanism=mlockall_current_future`.

## Required adapter interface

`preflight` creates schema-v1 JSON at an absent `--output`. It proves the exact
source commit, aarch64/Linux identity, capacity/payload/kind matrix,
`CLOCK_MONOTONIC`, explicit payload contract, `try_send`/`try_recv`, successful
memory locking and fault-suite readiness. The platform wrapper validates this
file before creating its evidence directory.

`benchmark` creates one schema-v2 JSON result for the requested row. In
addition to the common comparison schema it reports requested/effective
lock-required policy, process-wide lock mechanism, explicit payload contract,
Horus POD heuristic acknowledgement and hot-path API.

`fault-suite` creates schema-v1 JSON and exercises bounded producer behavior,
no torn values, explicit loss/full accounting, slow/stopped consumers, clean
exit and killed-peer reclaim. It reports the same payload and memory-lock
contract. Horus's generic corruption helpers or clean-drop comments alone are
not evidence for these process tests.

Source anchors:

- <https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/benchmarks/src/bin/all_paths_latency.rs>
- <https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/benchmarks/src/timing.rs>
- <https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/pod.rs>
- <https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_core/src/communication/topic/mod.rs>
- <https://github.com/softmata/horus/blob/fac3cfb8704e275bde9daec6d554a17c59dd8532/horus_sys/src/shm/linux.rs>
