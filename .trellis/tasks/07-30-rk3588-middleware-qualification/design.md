# RK3588 middleware qualification — Technical design

## Boundary and current environment

The checked-in runner produces candidate measurements, but only a platform
envelope may turn them into target evidence. The development host is x86_64
and has no `/proc/device-tree/compatible`; it can validate fixtures and plans
but cannot close any RK3588 acceptance item.

```text
middleware-qualification (safe bounded measurement binary)
        |
        +-- local/thread
        +-- project SHM/thread
        +-- project SHM/process

qualify-middleware.sh (Orange Pi target envelope)
        |
        +-- exact board/kernel/profile/source/build checks
        +-- taskset/chrt/memory-lock run policy
        +-- pinned external Horus runner verification
        +-- baseline or platform concurrent-stress envelope
        +-- process crash/fault suite
        +-- raw results + evaluator + SHA256SUMS
```

The Rust runner always labels its standalone output `host_research`. Only the
target envelope, after verifying physical identity and the complete platform
run, may emit an aggregate target verdict. It never edits a runtime profile.

## Measurement clock and payloads

Each fixed 64/256/1024-byte payload stores sequence and Linux
`CLOCK_MONOTONIC` send time in its first two `u64` fields. The receiver records
`receive_clock - sent_clock`, including the process peer because Linux
monotonic clocks share the same kernel time domain. Producer-call duration is
measured independently around only `try_publish`. The runner records both
log2-nanosecond histograms/maxima, raw counters, process CPU time, process
high-water resident memory and combined producer/consumer hot allocation
totals. A backwards or overflowing
clock observation aborts the run rather than being interpreted as fast
delivery.

`rustix` supplies the safe clock interface; it does not add a new workspace
unsafe boundary.

The target envelope also wraps each candidate with identical `perf stat`
events: cycles, instructions, task-clock, context switches, CPU migrations and
page faults. A missing/unsupported cycle counter fails the preflight before an
evidence directory is created.

## Target entry gates

An actual run requires:

- aarch64 plus device-tree identities `xunlong,orangepi-5-plus` and
  `rockchip,rk3588`;
- the filled hardware manifest, exact mode/profile, PREEMPT_RT readback,
  governor/affinity/scheduler checks and memory-lock policy;
- a build ID and exact rs-control git commit;
- the checked-in source-lock digest;
- an executable project runner hash;
- an external Horus checkout whose `HEAD` equals
  `fac3cfb8704e275bde9daec6d554a17c59dd8532`, plus a recorded runner hash;
- an absent output directory.

No host override can produce a passing target verdict. Fixture roots may be
used only by the host regression tests and their verdict is always
`incomplete`.

## Matrix and concurrency

The default matrix is latest/ordered × 64/256/1024 bytes × local-thread,
SHM-thread and SHM-process, with identical capacity/warm-up/sample settings.
Both `*-thread` candidates use a real separate consumer thread and a
preallocated atomic sample acknowledgement; producer waiting is outside the
separately measured `try_publish` call. `SHM-process` uses a separate worker
process and reports its CPU/memory independently.
The external Horus adapter receives the same settings and must emit the
versioned comparison schema while retaining semantic differences.

For concurrent mode the envelope launches the existing `qualify-rt.sh`
baseline/concurrent procedure, waits for its thermal/stress evidence to become
live, runs the middleware matrix under the declared CPU/scheduler policy, then
waits for the complete platform duration. A run below eight hours remains
incomplete even when every middleware sample passes.

## Safety and fault evidence

The target executes the full locked project SHM suite offline to cover
live endpoint refusal, clean exit, real SIGKILL, corrupt PID/epoch, odd-stamp
writer death and stopped consumers. Horus remains external and its adapter must
provide equivalent raw fault results or is marked incomplete. Any torn value,
producer timeout, unaccounted generation, nonzero hot allocation, live reclaim,
thermal/frequency failure or identity mismatch is disqualifying.

## Evidence and rollback

Every raw input/result is written once under a new output directory. The
evaluator writes a deterministic comparison report and the envelope hashes all
files into `SHA256SUMS`. Failed/incomplete bundles are preserved. Rollback is
to discard a candidate output directory and rerun into a different absent
directory; profiles remain candidate/unqualified until a separately reviewed
passing digest is installed.
