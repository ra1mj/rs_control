# Network input fallback policy

## Goal

Complete the parent framework's software-only D3 network-fault contract with a
backend-neutral, allocation-free decision that tells a consumer whether to use
a current sample, temporarily hold a disconnected sample, expose unavailable,
or request a safe stop.

## Background

`middleware-api::FallbackBehavior` already declares `HoldUntilExpired`,
`Unavailable`, and `SafeStopSignal`. `middleware-zenoh::SourceGuard` already
validates source, boot, sequence, freshness, quality and disconnect/reconnect,
but no executable contract combines route availability, the last accepted
sample's local expiry, and the declared fallback.

The existing fault-matrix test covers loss, duplicate, reorder, delay,
disconnect, reconnect, and publisher restart. It does not explicitly prove
that irregular arrival timing preserves exact local expiries or that each
fallback policy yields a deterministic result at its boundary.

## Requirements

- **NIF-01:** `middleware-api` owns a pure backend-neutral fallback evaluator;
  application code must not import `middleware-zenoh` or inspect its
  `ReceivedSample`.
- **NIF-02:** The evaluator accepts only a declared `FallbackBehavior`, route
  availability, an optional cached value with exclusive local monotonic
  expiry, and the current local monotonic time.
- **NIF-03:** An available route with an unexpired cached value returns
  `UseCurrent`. A disconnected route with `HoldUntilExpired` and an unexpired
  value returns `UseHeld`.
- **NIF-04:** Missing or expired cache under `HoldUntilExpired` returns
  `Unavailable`. `Unavailable` and `SafeStopSignal` react immediately to a
  disconnected route even while its cache remains fresh.
- **NIF-05:** `SourceGuard` retains source/boot identity in each opaque
  accepted sample and treats a cached prior-boot sample as absent immediately
  after a new boot is authorized.
- **NIF-06:** Expiry is exclusive: `now < expires` is usable and
  `now == expires` is expired. No fallback extends or refreshes expiry.
- **NIF-07:** The decision path is allocation-free, nonblocking, deterministic,
  and performs no logging, serialization, clock read, or network operation.
- **NIF-08:** The Zenoh fault script explicitly proves accepted varying delay
  and jitter, exact expiry arithmetic, sequence loss accounting, duplicate,
  reorder, expiry, degraded quality, unexpected source, disconnect, reconnect,
  publisher restart, and retired-boot rejection.

## Acceptance Criteria

- [x] Unit tests cover every decision across connected/disconnected,
      present/missing, fresh/last-valid/expired cache conditions.
- [x] Boundary tests prove exclusive expiry and that neither reconnect nor
      fallback refreshes an accepted sample.
- [x] Authorizing a replacement publisher boot immediately invalidates a
      still-fresh cached sample from the retired boot.
- [x] Allocation instrumentation observes zero allocations for all decisions.
- [x] The fault-matrix integration test explicitly exercises all D3 network
      fault categories and retains exact saturating counters.
- [x] `cargo run --locked -p architecture-check -- check` and `tools/ci.sh`
      pass.
- [x] Only the two remaining parent D3 software checkboxes are marked
      complete; Orange Pi and target-board gates remain open.

## Out of Scope

- Network emulation, latency qualification, or target-board evidence.
- Application estimator/controller fallback source selection.
- New middleware transports, async work, retries, automatic reauthorization,
  or deadline/lease changes.
- Physical motor motion, Orange Pi configuration, or realtime qualification.

## Traceability

- Parent: `.trellis/tasks/07-29-rust-robot-framework/implement.md` Phase D3.
- Design: parent `design.md` sections 6.6 and 13.
- Contracts: `.trellis/spec/backend/robot-contracts.md` and
  `.trellis/spec/guides/cross-layer-thinking-guide.md`.
