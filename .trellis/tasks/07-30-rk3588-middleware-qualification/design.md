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
- a schema-v1 adapter preflight proving the exact payload/kind matrix,
  `CLOCK_MONOTONIC`, `try_send`/`try_recv`, explicit `repr(C)` bytemuck-POD
  payloads, process-wide `mlockall` and fault-suite readiness;
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
versioned comparison schema while retaining semantic differences. Capacity is
restricted to an exact power of two in `16..=1024`: the pinned Horus custom
constructor allocates at `next_power_of_two` but initializes its shared header
from the request. Its bundled latency benchmark is excluded because it fixes
different messages, iterations and CPU IDs and uses wall-clock timing on
non-x86. The adapter also records that Horus's generic `needs_drop` POD
heuristic is broader than its independently verified payload layout.

For concurrent mode the envelope launches the existing `qualify-rt.sh`
baseline/concurrent procedure, waits for its thermal/stress evidence to become
live, runs the middleware matrix under the declared CPU/scheduler policy, then
waits for the complete platform duration. A run below eight hours remains
incomplete even when every middleware sample passes.

## Safety and fault evidence

The target executes the full locked project SHM suite offline to cover
live endpoint refusal, clean exit, real SIGKILL, corrupt PID/epoch, odd-stamp
writer death and stopped consumers. Horus remains external and its adapter must
provide equivalent raw fault results or is marked incomplete. Every adapter
process must acquire and verify `mlockall(MCL_CURRENT | MCL_FUTURE)` itself;
the pinned SHM mapping layer does not do this. Any torn value,
producer timeout, unaccounted generation, nonzero hot allocation, live reclaim,
thermal/frequency failure or identity mismatch is disqualifying.

## Evidence and rollback

Every raw input/result is written once under a new output directory. The
evaluator writes a deterministic comparison report and the envelope hashes all
files into `SHA256SUMS`; each parent manifest includes child manifests so the
top digest anchors the complete tree. The offline verifier checks every
manifest and exact file set, rejects symlinks/special files, revalidates input,
hardware and Horus contracts, regenerates the comparison from raw results and
re-derives the wrapper verdict. Its printed aggregate digest is the SHA-256 of
the top manifest and is recorded outside the bundle. A valid sealed
failed/incomplete bundle verifies as internally consistent but is never
selection evidence. Failed/incomplete bundles are preserved. Rollback is
to discard a candidate output directory and rerun into a different absent
directory; profiles remain candidate/unqualified until a separately reviewed
passing digest is installed.

Every long target command and the concurrent platform envelope run in their
own `setsid` session. HUP/INT/TERM exits the wrapper with the corresponding
129/130/143 status after bounded process-group TERM/KILL cleanup and direct
child reaping. If interruption happens after output creation, the partial
directory is retained for diagnosis but has no qualification standing and is
never resumed or re-sealed; the retry uses a new absent directory.
