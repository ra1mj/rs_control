# Lifecycle and restart safety design

## Ownership

`control-core` owns application lifecycle semantics. The reducer consumes
already-validated, serialized semantic events and returns state plus an
ordered fixed-capacity effect batch. It never reaches into protocol or
hardware implementations.

## State model

The public leaf states remain:

```text
Boot -> Standby -> Armed -> Running -> Degraded
                       \-> Fault
all non-Shutdown states -> EStop
all states -> Shutdown
```

`EStop` also carries reducer-owned latch context: asserted, or release
observed. That context is exposed in the projection but does not create a
second externally visible leaf state.

## Restart correlation

`LifecycleMachine::new(boot_id)` receives the nonzero identity generated for
this process start. It never accepts a prior lifecycle checkpoint.

Health reconstruction, Arm, Start, fault-clear and E-stop-reset events carry a
process boot identity. Mismatches are rejected before any authority change.
Arm sequences must strictly increase within a boot. Start must reference the
currently accepted Arm sequence. Each fault occurrence also advances a
reducer-owned generation that an explicit clear must match. Therefore delayed
prior-boot events, stale fault clears and same-boot duplicates cannot
re-authorize motion.

## Reconstruction

The live-health collector remains outside the hot reducer. Each incorporated
observation advances a current-boot, strictly increasing revision while the
machine remains inhibited in `Boot`. Completion has one typed result:

- `Ready`;
- `Missing`;
- `Conflicting`;
- `TimedOut`;
- `Faulted`.

Evidence contains the current FSM boot identity and a strictly increasing
revision. Duplicate or reordered observations are rejected. Only `Ready` can
leave `Boot` for `Standby`. All four terminal unsafe results enter `Fault`.
Incomplete collection is represented by progress without a completion event;
the machine remains inhibited in `Boot`.

## Emergency stop

Assertion is globally handled for every state except completed `Shutdown`.
Each assertion advances a reducer-owned generation and clears any previously
observed release. A release observation changes only latch context; it must
match both the current process boot and assertion generation, so a delayed
release cannot clear a newer assertion. The public state stays `EStop` and
motion stays inhibited. Software reset is accepted only after release and only
when correlated to the same current process boot and assertion generation.
Reset clears arm authority and still projects motion inhibit. It returns to
`Boot` if E-stop interrupted reconstruction, to `Fault` if it preempted or
observed a fault, and otherwise to `Standby`; reset never bypasses an unmet
reconstruction or fault-clear prerequisite.

## Effect ordering

Every changed transition out of `Running` begins with:

```text
CancelMission
ReleaseResources
```

State-specific effects follow. Any projection whose resulting state is
`Boot`, `Standby`, `Degraded`, `Fault`, `EStop` or `Shutdown` ends with
`MotionInhibited`. The largest batch is statically bounded.

## Diagnostics

Stable arrays enumerate every state and event kind. Diagnostic constructors
reach each state through the production machine. The report and DOT generator
then dispatch canonical events through the same reducer. They may allocate
because they are host/CI diagnostics; construction and dispatch may not.

## Verification

- exhaustive state/event report row count;
- startup, unsafe reconstruction and restart inhibition;
- current-boot and strictly increasing sequence guards;
- global/latching E-stop behavior;
- exact Running exit order;
- duplicate, loss, reorder, timeout, fault and restart traces;
- deterministic replay and diagnostic bytes;
- zero allocation over every transition class;
- production dependency graph remains free of `statig`.
