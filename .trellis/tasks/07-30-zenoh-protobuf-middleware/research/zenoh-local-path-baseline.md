# Zenoh and local-path research baseline

Research date: 2026-07-30

## Repository evidence

- The parent design assigns `middleware-api`, `middleware-local` and
  `middleware-zenoh` to `crates/protocol_module/`.
- The parent D1-D3 checklist requires latest-value and ordered-event channels,
  a non-RT Zenoh bridge, canonical key expressions, stream/action/query/
  liveliness classes and deterministic network fault tests.
- `.trellis/spec/backend/quality-guidelines.md` currently forbids `unsafe` for
  the whole workspace. `.trellis/spec/backend/robot-contracts.md` requires any
  mmap/raw-atomic shared-memory implementation to remain private to a reviewed
  `protocol_module` boundary with an explicit fixed layout contract.
- The Orange Pi research already pins the Horus comparison source to commit
  `fac3cfb8704e275bde9daec6d554a17c59dd8532`. Its queue/seqlock/flock patterns
  are comparison inputs only; published desktop measurements are not accepted
  as RK3588 evidence.
- Existing `message-model` values carry fixed-capacity identifiers and checked
  source/boot/sequence/time/quality metadata. `contract-codec` is already the
  sole fallible conversion owner for `robot.v1` Protobuf.

## Current Zenoh Rust API

The current public Rust crate inspected for this plan is `zenoh` 1.9.0.
`cargo info zenoh@1.9.0` reports Rust 1.75 minimum and a broad default feature
set including authentication, compression, TCP, TLS, UDP, QUIC, Unix sockets
and WebSocket. The first trusted-LAN adapter should therefore disable default
features and enable only the transports demonstrated by its deployment/tests;
the exact dependency stays pinned and committed in `Cargo.lock`.

The reviewed first profile uses explicit router endpoints and therefore pins
only the stable `transport_tcp` feature. UDP data transport/scouting, TLS,
QUIC, authentication, compression, WebSocket, unstable APIs and Zenoh shared
memory stay disabled until a deployment profile requires and tests them.

Official API evidence:

- QoS exposes priority and congestion control on publisher/query builders.
  Transport priority queues only take effect when transport QoS is enabled.
  `CongestionControl::Block` can wait for transport capacity, so it is never
  used from an RT producer and remains owned by the non-RT adapter.
  <https://docs.rs/zenoh/1.9.0/zenoh/qos/index.html>
- Zenoh key expressions are a validated routing language; checked
  `KeyExpr::new`/`OwnedKeyExpr` and `join` APIs exist. The project must still
  own its narrower semantic grammar so wildcard selectors cannot enter a
  concrete command publisher by accident.
  <https://docs.rs/zenoh/1.9.0/zenoh/key_expr/index.html>
- Queryables receive a `Query` and explicitly send data/error replies; a get
  can yield zero or more replies. The middleware contract therefore needs
  bounded timeout, correlation and reply-count behavior instead of presenting
  query/reply as an infallible single response.
  <https://docs.rs/zenoh/1.9.0/zenoh/query/index.html>
- Liveliness tokens are tied to a session and can be queried or subscribed to;
  disappearance means loss of session/connectivity observation, not automatic
  permission to disable local safety checks or extend a lease.
  <https://docs.rs/zenoh/1.9.0/zenoh/liveliness/index.html>
- The upstream repository documents pub/sub, query/reply and optional shared
  memory as separate features. This project does not enable Zenoh shared
  memory in the first safe local backend because its layout/safety contract is
  independently owned and reviewed.
  <https://github.com/eclipse-zenoh/zenoh>

## Planning consequences

1. Keep semantic endpoint descriptors and key parsing independent of Zenoh
   types; the adapter performs the final checked conversion to a Zenoh key.
2. Keep the Zenoh session and all blocking/async work on a non-RT worker.
3. Use safe, preallocated in-process channel dependencies behind private
   wrappers; benchmark candidates before naming one a target-board winner.
4. Test the bridge's filters and fault state machine deterministically without
   requiring multicast or timing-sensitive network behavior in the default CI
   gate. Add actual two-session Zenoh smoke tests where they can use isolated,
   explicit endpoints without making host CI flaky.
5. Split mmap shared memory and RK3588 qualification from this host slice. The
   future backend must preserve the same port results and counters so callers
   do not learn whether delivery is in-process or IPC.

## Safe local-channel candidate review

The workspace license allowlist excludes MPL-2.0, so the otherwise suitable
`triple_buffer` 9.0.0 crate is not selected and the task does not broaden the
license policy merely to obtain one implementation.

`waitfree-sync` 0.3.3 is MIT/Apache-2.0, requires Rust 1.81 and supplies a
triple-buffer latest-value primitive. Source inspection shows preallocated
slots, acquire/release atomics and internal `UnsafeCell`/`unsafe` hidden behind
safe nonclonable endpoints. The wrapper still:

- require reviewed `Copy` values for the RT latest path because reads invoke
  `Clone` internally;
- add its own endpoint liveness, generation and saturating counters;
- prove steady-state allocation behavior in project tests;
- avoid claiming the dependency's host behavior is an RK3588 result.

The `waitfree-sync` SPSC full result does not return the rejected value, which
violates the port's ownership-preserving backpressure contract. `rtrb` 0.3.4
is therefore the selected MIT/Apache-2.0 ordered queue: `PushError::Full(T)`
returns ownership, capacity is explicit and endpoint abandonment is observable.
It remains private to the same safe wrapper and is not claimed as the target
board performance winner.

## Dependency-policy finding during implementation

The exact Zenoh 1.9.0 TCP-only graph adds ISC-licensed utilities and one
MPL-2.0 `option-ext` utility through `zenoh-util -> shellexpand -> dirs`.
`deny.toml` allows ISC and scopes MPL-2.0 only to `option-ext@0.2.0`; MPL is not
globally enabled and remains unavailable for a direct local backend choice.

Zenoh 1.9.0 also declares `lz4_flex 0.10.0` unconditionally even when its
`transport_compression` feature is disabled. RUSTSEC-2026-0041 applies to the
block decompression APIs, while all Zenoh call sites are compiled only under
that disabled feature. The advisory exception is therefore exact, reasoned
and coupled to the invariant that this crate enables only `transport_tcp`.
It must be removed when upstream updates the dependency, and compression must
not be enabled while the exception exists.

## Cross-host monotonic-time finding

`CommandEnvelope.deadline_monotonic_ns` and `ResourceLease` expiration are
documented as values in the local enforcement clock domain. They cannot be
interpreted across different hosts. The safe additive resolution is a
`robot.v1` command-intent contract containing the asserted lease identity and
a requested TTL. On ingress, the adapter compares that assertion with a local
active lease and uses local reception time to create the executable envelope.
The transmitted source/wall timestamp remains freshness evidence only and can
never extend local authority.
