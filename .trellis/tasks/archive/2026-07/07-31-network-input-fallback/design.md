# Network input fallback design

## Boundary

The flow remains:

```text
middleware-zenoh validates payload
  -> local cache stores value plus exclusive monotonic expiry
  -> middleware-api fallback evaluator
  -> backend-neutral decision consumed by composition/application wiring
```

`middleware-api` owns the decision because the policy must not depend on
Zenoh. The evaluator uses a borrowed cached value and does not own storage.
`ReceivedSample` remains opaque and retains its validated header so the
`SourceGuard` adapter can exclude a cache whose source or boot is no longer
current.

## API

```rust
CachedInput::new(value: &T, expires_monotonic_ns: u64) -> CachedInput<'_, T>

evaluate_network_input(
    fallback: FallbackBehavior,
    route_available: bool,
    cached: Option<CachedInput<'_, T>>,
    now_monotonic_ns: u64,
) -> NetworkInputDecision<'_, T>

NetworkInputDecision::{UseCurrent(&T), UseHeld(&T), Unavailable, SafeStopSignal}
```

The evaluator is total and infallible. The Zenoh ingress boundary has already
validated the sample and computed its checked local expiry. A cache with
`now < expiry` is fresh; equality is expired.

## Decision table

| Route | Cache | Policy | Decision |
|---|---|---|---|
| available | fresh | any | `UseCurrent` |
| disconnected | fresh | `HoldUntilExpired` | `UseHeld` |
| disconnected | fresh | `Unavailable` | `Unavailable` |
| disconnected | fresh | `SafeStopSignal` | `SafeStopSignal` |
| either | missing/expired | `HoldUntilExpired` | `Unavailable` |
| either | missing/expired | `Unavailable` | `Unavailable` |
| either | missing/expired | `SafeStopSignal` | `SafeStopSignal` |

The decision borrows the value and cannot change its expiry. Reconnect changes
only route availability; it never turns an expired value back into a current
one. Authorizing a new publisher boot immediately makes a cached sample from
the retired boot ineligible, even if its original local expiry is in the
future.

## Fault script

The existing Zenoh integration fixture remains transport-free and
deterministic. It will add accepted samples with nonuniform local receive times
and translated ages, assert their exact exclusive expiries, then retain the
existing loss/duplicate/reorder/expiry/quality/source/restart/disconnect
sequence and counter checks.

## Rollback

The new API is additive. Rollback removes the evaluator/tests and restores the
two parent D3 checkboxes without changing wire formats, runtime configuration,
or any hardware path.
