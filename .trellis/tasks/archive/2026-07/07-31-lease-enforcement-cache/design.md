# Lease enforcement cache design

## Ownership and flow

```text
FSM/authority policy emits LeaseEvent
  -> motion-rt single owner applies event to control-core cache
  -> cache returns fixed-capacity LeaseEffect/removal evidence
  -> command envelope is authorized against current active entry
  -> composition root performs the declared safe action/driver write
```

The cache owns enforcement, not grant priority policy or hardware actions.
`message-model` remains the owner of validated lease/command fields.

## State

```rust
LeaseEnforcementCache<CAPACITY> {
    highest_session_epoch: u64,
    current_session_epoch: Option<u64>,
    entries: ArrayVec<LeaseEntry, CAPACITY>,
}

LeaseEntryState::{Active(ResourceLease), AwaitingSafe(ResourceLease)}
```

An entry exists at most once per resource. Pending replacement authority is
stored but cannot authorize commands. Fixed capacity makes all mutations
bounded and allocation-free.

## Events

```rust
LeaseEvent::{
    StartSession { session_epoch },
    Grant(ResourceLease),
    Renew(ResourceLease),
    Release(LeaseKey),
    Preempt { current: LeaseKey, replacement: ResourceLease },
    ConfirmSafe(LeaseKey),
    SessionLost { session_epoch },
    Tick,
}

cache.apply(event, now_monotonic_ns)
    -> Result<LeaseEffect<CAPACITY>, LeaseCacheError>
cache.authorize(command, now_monotonic_ns)
    -> Result<ResourceLease, LeaseCacheError>
```

`LeaseKey` contains resource, lease ID and session epoch. `LeaseEffect`
contains a stable effect kind, an optional activated/renewed lease, and bounded
`LeaseRemoval` evidence. Each removal records its reason and whether the entry
was active or merely pending safe confirmation.

## Transition rules

| Event | Required state | Result |
|---|---|---|
| Start session | nonzero epoch `> highest` | clear entries, make epoch current |
| Grant | current matching epoch, valid lease, resource absent | active |
| Renew | exact active identity, later valid expiry | replace active lease |
| Release | exact active/pending key | remove |
| Preempt | exact active key, distinct valid replacement | old removed; replacement pending |
| Confirm safe | exact pending replacement, still valid | active |
| Tick | any | remove every active/pending expired entry |
| Session lost | exact current epoch | clear entries; current becomes absent |

Start-session, session-loss, release, preempt and expiry effects report all
removed entries. Starting a newer session is allowed even if the old session
did not send a loss event, but it revokes all old authority.

## Command enforcement

Authorization first checks the command's exclusive deadline, then requires a
current session and an active resource entry. It validates the lease at the
same supplied local monotonic time and compares resource, owner, lease ID and
session epoch exactly. It returns the matching lease only as evidence; it does
not execute a command or refresh state.

## Rollback

The change adds application-only APIs and dependencies. Rollback removes the
lease module and restores the parent G3 checkboxes. It does not change
Protobuf, message-model, middleware, hardware, or deployment behavior.
