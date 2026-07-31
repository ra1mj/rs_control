# Lease enforcement cache

## Goal

Implement the software-only authority state machine that lets `motion-rt`
enforce one finite lease per exclusive resource without last-writer-wins
behavior, heap allocation, or dependence on a hardware driver.

## Background

`message-model::ResourceLease` and `CommandEnvelope` already validate their
individual structure, local expiry and exact authority fields. No
`application_module` owner currently reduces grant/renew/release/expire/
preempt events, records the current authority-service epoch, or invalidates all
authority after FSM loss/restart.

The parent design requires two-phase takeover: revoke the old lease, confirm
the resource is safe, and only then activate its replacement. A new authority
service session must never revive cached leases or commands from an older
epoch.

## Dependencies

- Completed core message/command contracts and architecture policy.
- Completed local/Zenoh ingress authority validation.
- No dependency on Orange Pi, SocketCAN, a motor fixture, or the Phase E
  runtime composition root.

## Requirements

- **LEC-01:** `control-core` owns a fixed-capacity
  `LeaseEnforcementCache<const CAPACITY: usize>` and deterministic `LeaseEvent`
  reducer. It consumes validated protocol-module values and never imports a
  concrete middleware or hardware crate.
- **LEC-02:** Session epoch zero is rejected. Starting a session requires an
  epoch strictly greater than every previously seen epoch and removes all
  active or pending authority.
- **LEC-03:** Grant accepts a nonexpired lease only for the current session and
  an unoccupied resource. A competing grant never replaces the incumbent.
- **LEC-04:** Renew requires exact resource/owner/lease/epoch identity, an
  active lease, and a strictly later nonexpired expiration.
- **LEC-05:** Release requires exact resource/lease/epoch identity and removes
  either active or pending authority. Tick expiry removes every lease whose
  exclusive expiration satisfies `expires <= now`.
- **LEC-06:** Preempt requires the exact active lease and a distinct, valid
  replacement for the same resource/current epoch. It immediately removes old
  command authority and enters `AwaitingSafe`; only an exact safe-confirm event
  can activate the replacement.
- **LEC-07:** Session loss requires the exact current epoch, clears every
  active/pending resource, and leaves the cache unable to authorize until a
  strictly newer session starts.
- **LEC-08:** Command authorization requires a current active lease, unexpired
  lease and command, and exact resource/owner/lease/session fields. Pending,
  released, expired, foreign-owner, competing-source and prior-epoch commands
  fail closed.
- **LEC-09:** Every removal returns fixed-capacity typed evidence including the
  lease, whether it had active command authority, and release/expiry/preempt/
  session-loss/session-replacement reason.
- **LEC-10:** Event application and command authorization perform no heap
  allocation, blocking, logging, serialization, clock reads, or I/O.

## Acceptance Criteria

- [x] Unit tests cover every event, invalid transition, capacity boundary and
      exclusive expiry boundary with exact effects.
- [x] Two-phase preemption proves no command is authorized between old
      revocation and safe confirmation.
- [x] Interleaved commands from competing owners prove there is no
      last-writer-wins path.
- [x] FSM loss and higher-epoch restart reject old leases, old commands and
      same/lower session restart.
- [x] Allocation instrumentation observes zero allocations for the event and
      authorization hot paths.
- [x] `cargo run --locked -p architecture-check -- check` and `tools/ci.sh`
      pass.
- [x] Parent G3 software checkboxes are marked complete only for behavior
      proven by this host task; Phase E and all target/hardware gates remain
      open.

## Out of Scope

- FSM policy for deciding priorities or which owner receives a lease.
- Physical safe-stop confirmation, motor enable/control, systemd, or RT thread
  setup.
- Protobuf changes, network distribution of lease events, persistence, or
  automatic restoration after restart.
- Locks or a multi-thread shared mutable cache; the reducer is single-owner
  state intended for deterministic motion-thread consumption.

## Traceability

- Parent: `.trellis/tasks/07-29-rust-robot-framework/implement.md` G3.
- Design: parent `design.md` section 8.
- Contracts: `.trellis/spec/backend/robot-contracts.md`,
  `.trellis/spec/backend/error-handling.md`, and
  `.trellis/spec/guides/cross-layer-thinking-guide.md`.
