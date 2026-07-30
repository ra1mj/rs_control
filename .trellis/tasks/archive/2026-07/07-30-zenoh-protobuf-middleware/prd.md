# Zenoh Protobuf middleware

## Goal

Deliver the first Rust `protocol_module` middleware slice: backend-neutral,
bounded local delivery contracts plus a non-realtime Zenoh/Protobuf adapter
that preserves canonical `robot.v1` semantics and cannot stall the motion
control producer.

## Background and scope decision

The parent robot-framework task requires an interchangeable local fast path,
Zenoh/Protobuf delivery, canonical key/QoS semantics and deterministic network
fault behavior. This child owns the host-testable safe-Rust slice:

- backend-neutral ports and endpoint contracts;
- a preallocated in-process latest-value and ordered-event backend;
- the actual non-RT Zenoh adapter and deterministic host fault tests;
- additive network-command intent contracts needed to enforce local authority.

The mmap shared-memory backend, raw atomic/`unsafe` boundary, crashed-process
lease reclamation, `SIGKILL` recovery and RK3588 performance qualification are
deferred to a separately reviewed child task. The workspace currently forbids
`unsafe`, while the parent requires mmap payload/layout safety to receive an
explicit architecture review. A future IPC backend must preserve the ports and
observable delivery semantics defined here.

## Requirements

- **ZM-01:** `application_module` consumers depend only on typed middleware
  ports. They do not depend on Zenoh, generated Protobuf DTOs, a local channel
  crate, or a shared-memory implementation.
- **ZM-02:** The local API distinguishes latest-value samples/setpoints from
  ordered discrete events. Both classes have bounded storage, explicit
  sequence/drop/overflow observations, and nonblocking producer operations.
- **ZM-03:** RT-visible storage is allocated before use. A 1 kHz producer path
  performs no Zenoh call, Protobuf encoding, logging, blocking wait, async
  executor work, or hidden unbounded allocation.
- **ZM-04:** A stalled or failed consumer cannot indefinitely stop or back up a
  producer. Latest-value delivery retains the newest publishable value;
  ordered delivery returns a typed full/disconnected result according to its
  declared policy.
- **ZM-05:** One checked key builder/parser owns the canonical concrete key
  shape `robot/<robot_id>/v1/<domain>/<resource>/<channel>`. Wildcards are
  accepted only by APIs whose contract explicitly permits selectors.
- **ZM-06:** Network payloads remain canonical `robot.v1` Protobuf. The adapter
  uses `contract-codec` as the only DTO/semantic conversion boundary and
  rejects malformed, oversized, missing-field and unknown-enum inputs.
- **ZM-07:** The adapter supports streams, correlated action/ack events,
  bounded query/queryable operations and liveliness. Every endpoint declares
  a semantic QoS class and nonzero payload, queue and freshness/timeout bounds.
- **ZM-08:** Network-originated data is exposed to local consumers only after
  source, boot, sequence, local reception time, validity, quality and, for
  commands, session epoch and lease checks. Expired, duplicate, reordered,
  unapproved-boot and prior-lease inputs produce typed rejection outcomes and
  saturating counters.
- **ZM-09:** Cross-host command input is a command intent, never a sender-owned
  absolute monotonic deadline. The adapter must match the asserted authority
  tuple against an existing local lease and construct the executable
  `CommandEnvelope` from local reception time and a route-capped TTL. A remote
  timestamp cannot mint, renew or extend authority.
- **ZM-10:** A dedicated non-RT bridge owns the Zenoh session/runtime,
  Protobuf allocation, retries and reconnect behavior. It mirrors selected
  local samples through bounded ports and writes validated network snapshots
  without extending their validity.
- **ZM-11:** Deterministic host tests cover slow consumers, overflow,
  duplicate/reorder/stale filtering, publisher restart/boot authorization,
  disconnect/reconnect, query timeout and action correlation. API separation
  and instrumentation must show that local producer progress is independent
  of Zenoh worker progress.
- **ZM-12:** Third-party dependencies are exact-version pinned, use the
  minimum required features, pass license/advisory/source policy, and remain
  private to their owning `protocol_module` crate. No unstable or Zenoh
  shared-memory feature is enabled in this child.
- **ZM-13:** Host correctness and allocation tests are necessary but are not
  reported as RK3588 realtime qualification. Target-board latency, maximum
  jitter, cycle/load, prefault and contention evidence remains additive and
  records the parent task's platform/kernel/load manifest.

## Out of scope

- Adding middleware fields to `config/robots/*.yaml`.
- Changing the canonical package away from `robot.v1` or bypassing
  `contract-codec`.
- Running Zenoh, serialization, reconnect, diagnostics formatting or disk I/O
  on a realtime thread.
- Treating Zenoh priority, reliable transport, average host latency or ordinary
  Ethernet as a 1 ms hard-realtime guarantee.
- Importing Horus as a runtime dependency or copying its code.
- Implementing ROS 2/DDS transport; the existing `ros2-mapping` contract and a
  later bridge remain separate integration concerns.
- Implementing or qualifying the deferred cross-process shared-memory backend.

## Acceptance criteria

- [x] Classified `protocol_module` crates provide backend-neutral middleware
      ports, a safe local backend and a Zenoh adapter without forbidden graph
      edges.
- [x] Latest-value and ordered-event tests demonstrate bounded,
      allocation-free-after-construction and nonblocking producer operations,
      including exact publish/read/drop/overflow/disconnect observations.
- [x] The checked key contract round-trips every supported concrete domain and
      rejects invalid, wildcarded-concrete and cross-version inputs without
      panic.
- [x] Stream, action/ack, query/queryable and liveliness endpoints use bounded
      descriptors with explicit semantic QoS and payload limits.
- [x] At least one state stream traverses semantic value -> Protobuf -> real
      loopback Zenoh adapter -> decoded semantic value.
- [x] At least one network command intent is decoded, matched against a local
      active lease and materialized with a local deadline capped by route TTL
      and lease expiry; stale/prior-lease inputs are rejected.
- [x] Deterministic fault tests assert delay, loss, duplicate, reorder, stale
      data, publisher boot change, disconnect/reconnect and stopped bridge
      behavior with typed counters and fallback/unavailable state.
- [x] Public local producer APIs expose neither a Zenoh session nor an executor
      wait, and continue returning in bounded time when the bridge stops.
- [x] Dependency policy, architecture fixtures, Protobuf compatibility gates
      and the full `./tools/ci.sh` gate pass.
- [x] The task records the deferred mmap/RK3588 follow-up and does not mark
      parent shared-memory or target qualification criteria complete.

## Traceability notes

- Parent source: `.trellis/tasks/07-29-rust-robot-framework/`, requirements
  R22, R26-R32, R64, R71-R80, R104-R105 and implementation phase D1-D3.
- Horus research input is pinned at commit
  `fac3cfb8704e275bde9daec6d554a17c59dd8532`; it remains a comparison source,
  not a dependency.
- This is a complex task and requires reviewed `design.md` and `implement.md`
  before `task.py start` changes its status from planning to implementation.
