# Lifecycle and restart safety

## Goal

Implement the production robot lifecycle reducer in `control-core` so startup,
restart, mission exit, faults and emergency-stop handling are deterministic,
allocation-free and fail closed without depending on hardware.

## Requirements

- Use one direct exhaustive enum reducer; `statig` remains confined to the G1
  development harness.
- Construct every process instance with a nonzero process `boot_id`, start in
  `Boot`, and synchronously project `MotionInhibited`.
- Accept only live-health reconstruction evidence correlated to the current
  process boot. Successful reconstruction reaches `Standby`; missing,
  conflicting, timed-out or faulted reconstruction reaches inhibited `Fault`.
- Require a fresh current-boot Arm sequence after every restart. Start must
  reference the currently accepted Arm sequence; stale, duplicated and
  reordered authority events are explicit typed rejections.
- Make E-stop assertion preempt every state except completed `Shutdown`.
  Keep it latched until release is observed and a current-boot software reset
  is explicitly accepted.
- Leaving `Running` synchronously emits `CancelMission` before
  `ReleaseResources`, followed by any inhibit effect.
- Use fixed-capacity effect storage. Dispatch must not allocate, block, read a
  clock, perform I/O, serialize or log.
- Generate deterministic complete transition reports and DOT graphs by
  exercising the production reducer, not a second edge table.
- Keep this task host-only. Physical E-stop actuation and target timing remain
  hardware qualification gates.

## Acceptance Criteria

- [x] All lifecycle states and event kinds have an explicit reducer outcome.
- [x] Startup and all Boot paths provide motion-inhibit evidence.
- [x] Ready reconstruction reaches `Standby`; every unsafe reconstruction
  completion reaches inhibited `Fault`.
- [x] Prior-boot Arm, Start, clear and E-stop-reset events cannot authorize a
  restarted machine.
- [x] Duplicate and reordered reconstruction/Arm sequences are rejected
  deterministically.
- [x] E-stop is global, latched, and needs release observation followed by
  explicit current-boot reset.
- [x] Every transition leaving `Running` preserves exact cancel/release order.
- [x] Replay, complete report and DOT output are byte deterministic.
- [x] Warm construction and dispatch allocate zero heap objects.
- [x] Warnings-denied Clippy, architecture policy and `tools/ci.sh` pass.

## Notes

Host tests do not qualify physical E-stop wiring, motor power removal, target
deadlines, process supervision or Orange Pi behavior.
