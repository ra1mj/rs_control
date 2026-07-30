# Bug Analysis: middleware interruption did not terminate child work

## 0. Evidence and confidence

Initial hypotheses were: shell signal traps resume execution after cleanup
(45%), killing one PID leaves descendants alive (35%), or the concern is only
theoretical because the shell forwards signals (20%). A process-level probe
sent `TERM` directly to a POSIX shell waiting on `sleep`; after five seconds
both processes were still alive. Source inspection then showed that the
middleware trap only killed the platform wrapper PID and returned. This raises
the combined shell-delay/process-tree cause to greater than 99% confidence.

A follow-up regression exposed a separate shell-dialect issue: dash's `kill`
builtin rejected `kill -0 -- -PGID`, while zsh accepted it. The portable form
used by these Ubuntu `/bin/sh` scripts is `kill -SIGNAL -PGID`.

## 1. Root Cause Category

- **Category**: D/E — test coverage gap plus implicit process-model assumption.
- **Specific cause**: cleanup functions were registered directly for
  `INT`, `TERM`, and `HUP`, so a handled signal could return to the interrupted
  qualification flow instead of exiting. The platform cleanup also signaled
  only a background wrapper shell PID. A non-interactive shell can defer its
  trap while it waits for a foreground child, leaving the actual platform,
  benchmark, or stress processes running.

## 2. Why Earlier Checks Did Not Catch It

1. Shell syntax tests and target dry-run checks never delivered a signal to a
   live process tree.
2. The existing `qualify-rt.sh` trap looked like a reusable cleanup pattern,
   but its correctness normally relies on terminal-wide or service-cgroup
   signal delivery, not a signal sent only to its PID.
3. The first group-kill regression used zsh-compatible `-- -PGID` syntax. The
   production interpreter is `/bin/sh` (dash on Ubuntu), so the test had to be
   executed through the declared interpreter to expose the incompatibility.

## 3. Prevention Mechanisms

| Priority | Mechanism | Specific action | Status |
|---|---|---|---|
| P0 | Architecture | Launch every long middleware command and the platform envelope in a distinct `setsid` session | DONE |
| P0 | Runtime | On wrapper exit, send bounded group `TERM`, escalate to group `KILL`, and reap the direct child | DONE |
| P0 | Runtime fallback | If session establishment fails but its direct child survives, terminate that PID with the same bound | DONE |
| P0 | Signal contract | Map HUP/INT/TERM to 129/130/143 and exit; never let a signal cleanup handler return into qualification | DONE |
| P0 | Integration tests | Exercise cooperative group shutdown, ignored-TERM escalation, unsafe PGID refusal, and exact TERM trap status under `/bin/sh` | DONE |
| P1 | Documentation | Record interrupted-bundle and process-supervision semantics in the platform spec and runbook | DONE |

## 4. Systematic Expansion

- **Similar issues**: Horus preflight, each perf-wrapped matrix row, project and
  Horus fault suites, evaluator execution, and the concurrent platform envelope
  all had the same foreground-wait or descendant-lifetime risk. They now share
  the supervised-session path.
- **Design improvement**: process-tree ownership is explicit. Cleanup targets a
  process group created by this wrapper, validates that the PGID is safe, has a
  finite grace period, and waits for the owned leader. Detection that the
  concurrent platform envelope exited also stops the remaining candidate
  matrix instead of collecting rows outside the declared stress condition.
- **Process improvement**: shell lifecycle tests must run with the script's
  declared `/bin/sh`, and must test the process tree rather than only the trap
  source text.

## 5. Knowledge Capture

- [x] Realtime platform executable contract updated.
- [x] Middleware target runbook and platform README updated.
- [x] Task design and implementation plan updated.
- [x] Process-level host regressions added.
- [x] No template sync applies: this repository has no
      `src/templates/markdown/spec/` directory.
