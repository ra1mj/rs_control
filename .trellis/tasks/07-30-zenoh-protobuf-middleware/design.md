# Zenoh Protobuf middleware — Technical design

## 1. Outcome and safety boundary

This task adds three `protocol_module` crates and the smallest additive
canonical protocol change required for safe cross-host control:

```text
application algorithm / motion loop
           |
           | typed, nonblocking ports only
           v
    middleware-api
           |
           +--> middleware-local ---- safe in-process wait-free storage
           |
           +--> middleware-zenoh ---- non-RT worker only
                         |
                         v
                  contract-codec
                         |
                         v
                 robot.v1 Protobuf
                         |
                         v
                       Zenoh
```

The RT-facing side never owns a Zenoh `Session`, future, callback, payload
buffer or generated DTO. The adapter receives only the consumer side of local
ports. Zenoh callbacks, encode/decode, allocation, retry, timeout and reconnect
state are confined to the non-RT bridge.

This child contains no workspace-owned `unsafe`. It wraps reviewed safe APIs
from pinned dependencies. A future mmap backend is a replacement below
`middleware-api`, not an extension of application-facing types.

## 2. Repository and dependency shape

```text
crates/protocol_module/
  middleware-api/       # keys, endpoint/QoS contracts, port traits/results
  middleware-local/     # safe in-process latest/SPSC implementation
  middleware-zenoh/     # session worker, route pumps, ingress guards
  message-model/        # additive network command-intent semantic values
  contract-codec/       # sole intent/sample Protobuf conversion owner
  proto-generated/      # generated robot.v1 DTOs

proto/robot/v1/
  control.proto         # additive CommandIntent/typed intent messages
```

Workspace and `architecture.toml` classify all three crates as
`protocol_module`. Expected dependency direction is:

```text
middleware-local -> middleware-api
middleware-zenoh -> middleware-api + message-model + contract-codec + zenoh
contract-codec -> message-model + proto-generated
```

`middleware-zenoh` does not depend on the local implementation; it consumes
port traits. Composition roots select and connect the implementation later.

Initial dependency pins:

| Dependency | Pin/features | Reason |
|---|---|---|
| `waitfree-sync` | `=0.3.3` | MIT/Apache-2.0 safe API provides the latest-value triple buffer |
| `rtrb` | `=0.3.4` | MIT/Apache-2.0 bounded SPSC queue returns ownership from `PushError::Full(T)` |
| `zenoh` | `=1.9.0`, default features off, `transport_tcp` only | Current stable pub/sub, query and liveliness API without `unstable` or `shared-memory` |
| `allocation-counter` | `=0.8.1`, dev-only | Safe test API for per-thread allocation assertions |

The dependency source review records internal atomics/`unsafe`, licenses,
minimum Rust versions and known API constraints. Pinning a safe dependency
does not certify target realtime behavior; the target benchmark remains the
selection gate. Source inspection rejected `waitfree-sync`'s ordered queue for
this contract because a full push does not return the rejected `T`; `rtrb`
preserves that ownership without exposing either dependency in public APIs.

## 3. Backend-neutral contracts

### 3.1 Concrete keys and selectors

`middleware-api` owns `ConcreteKey`, `KeyDomain`, `KeyChannel` and the narrow
selector constructors. A concrete key has exactly six nonempty segments:

```text
robot/<robot_id>/v1/<domain>/<resource>/<channel>
```

`robot_id`, `resource` and `channel` use checked fixed-capacity ASCII
identifiers. `domain` is an exhaustive enum for `device`, `control`, `health`,
`fsm` and `perception`. Concrete construction rejects `/`, `*`, `$`, empty
segments, unsupported major versions, noncanonical spelling and trailing
segments. Parsing and formatting round-trip byte-for-byte.

Wildcard APIs use a separate `KeySelector` type constructed only by named
operations such as robot-wide observation or one-domain observation. A
`ConcreteKey` cannot be constructed from or converted unchecked from a
selector. The adapter performs a final checked conversion to Zenoh's key type;
Zenoh's broader grammar never becomes the project grammar.

### 3.2 Endpoint descriptors

An `EndpointDescriptor` contains:

- stable endpoint ID and concrete key;
- exact fully-qualified Protobuf type;
- direction and `InteractionClass` (`LatestStream`, `OrderedEvent`,
  `ActionCommand`, `ActionAck`, `Query`, `Queryable`, `Liveliness`);
- `SemanticQos` class;
- nonzero maximum payload bytes, local/adapter queue capacity and maximum
  age or request timeout;
- exact allowed source for single-writer routes;
- fallback behavior (`HoldUntilExpired`, `Unavailable`, `SafeStopSignal`),
  which the adapter reports but does not execute as application policy.

Validation rejects incompatible combinations, including wildcard publishers,
queries without a timeout, command routes without local authority, latest
streams with ordered retry semantics and zero bounds. Route collections reject
duplicate IDs, keys and ownership.

Semantic QoS maps conservatively:

| Class | Zenoh priority | Congestion | Semantic completion |
|---|---|---|---|
| control latest | `RealTime` | drop | newest valid sample; local lease still enforced |
| state latest | `DataHigh` | drop | sequence gaps observable |
| action/event | `InteractiveHigh` | drop + bounded application retry | matching ack/terminal event |
| query | `InteractiveHigh` | bounded request timeout | zero/one checked reply for project API |
| presence | `InteractiveHigh` | Zenoh liveliness | availability observation only |
| bulk | `DataLow` | drop | explicitly lossy |

No `CongestionControl::Block` is reachable from an RT-facing API. Application
acks and local deadlines, not an unstable Zenoh reliability option, define
command completion and safety.

### 3.3 Port surfaces

The API separates producer and consumer capabilities:

```rust
trait LatestPublisher<T> {
    fn try_publish(&mut self, value: T) -> LatestPublish;
    fn counters(&self) -> ChannelCounters;
}

trait LatestSubscriber<T> {
    fn try_read_latest(&mut self) -> Option<LatestSample<T>>;
}

trait EventPublisher<T> {
    fn try_publish(&mut self, value: T) -> Result<EventPublish, EventPublishError<T>>;
}

trait EventSubscriber<T> {
    fn try_receive(&mut self) -> Option<OrderedSample<T>>;
}
```

`LatestSample` and `OrderedSample` carry a channel-local generation. Outcomes
distinguish accepted, overwritten/full and disconnected. Counters are
saturating and expose published, consumed, overwritten/full and disconnected
totals. One writer and one reader are structural ownership constraints: the
endpoints are not clonable.

RT-qualified local values are `Copy + Send + 'static`; owned heap payloads do
not enter the latest RT port. Ordered event values are moved into preallocated
slots and retain ownership on a full result. Non-RT action payloads may own
bounded allocations but use a separately declared non-RT endpoint.

## 4. Safe local backend

`middleware-local` wraps the pinned `waitfree-sync` triple buffer for latest
values and `rtrb` for ordered events. Ordered construction rejects zero
capacity, while a full `rtrb` push returns the original value through the
backend-neutral typed error.

The wrapper adds:

- nonclonable typed endpoints;
- shared cache-separated atomic liveness/generation/counter metadata;
- sequence-gap/overwrite calculation;
- `Drop`-based endpoint-disconnected observation;
- saturating single-writer counters;
- no dependency types in public signatures.

All buffers and `Arc` control blocks are created at setup. Steady-state publish
and read operations on representative fixed-layout values are checked with an
allocation counter. A stopped reader can cause ordered events to return
`Full`, but cannot make the producer wait; latest writes continue and report
overwrites. Dropping a reader changes future results to `Disconnected` without
destroying the producer endpoint.

Host contention tests prove functional progress and accounting, not an upper
latency bound. Prefault/mlock and RK3588 measurements remain in the deferred
target task.

## 5. Canonical network command intent

### 5.1 Why a new intent is required

The existing `CommandEnvelope.deadline_monotonic_ns` is explicitly in the
local enforcement clock domain. Two hosts do not share that domain, even if
their wall clocks are synchronized. Therefore the Zenoh ingress must never
decode a remote absolute deadline as an executable local command.

`control.proto` gains additive `CommandIntent`, `ChassisSetpointIntent` and
`ActionGoalIntent` messages. `CommandIntent` carries:

- validated `Header` source/boot/sequence/validity metadata;
- asserted resource, owner, lease ID and session epoch;
- command ID for idempotency/correlation;
- a nonzero requested TTL, not an absolute deadline.

Corresponding `message-model` values are structurally validated. The codec
owns all DTO conversion. Materialization takes an existing local
`ResourceLease`, local receive time and route maximum TTL, requires the entire
asserted authority tuple to match, then calls
`CommandEnvelope::from_bridge_ingress`. The resulting deadline is no later
than either local receipt plus capped TTL or local lease expiration.

Existing `ChassisSetpoint`/`ActionGoal` messages remain the executable local
and same-monotonic-domain contracts. They may be mirrored as observations but
are not accepted by a cross-host command-ingress route.

### 5.2 Network sample wrapper and freshness

Accepted network data is stored as `ReceivedSample<T>` with local reception
time, computed local expiry, source/boot/sequence identity and freshness
evidence. A local consumer calls `usable_at(now)` and never waits for a new
network sample.

The route selects one explicit policy:

- `SynchronizedAgeRequired`: caller supplies measured source-to-local age and
  uncertainty; missing or excessive uncertainty rejects the sample.
- `ReceptionBounded`: expiry starts at local receipt and is capped by both
  header validity and route maximum age; evidence is marked reception-only and
  cannot be presented as end-to-end age.

Unix/ROS timestamps remain observation/correlation metadata and never create a
local monotonic deadline.

### 5.3 Source, boot and sequence guard

Each input route has a bounded, configured source set. The guard rejects an
unexpected source, duplicate sequence, lower sequence and an unapproved boot
ID. A boot change is authorized only by a boot-identified presence/handshake
transition; seeing a different payload boot ID is insufficient. Previously
retired boot IDs are retained in bounded storage; exhaustion fails closed and
requires adapter restart/reconfiguration.

A transport disconnect marks the route unavailable but retains current
source/sequence state. Reconnect can resume the same boot only with a higher
sequence. An explicitly authorized new boot resets its sequence and retires
the old boot, so delayed prior-boot frames remain rejected.

## 6. Zenoh adapter runtime

`ZenohBridge::spawn` validates every descriptor and opens all declarations on
a dedicated named non-RT thread before returning readiness. The caller
receives only a lifecycle handle and local ports; it never receives the
session. Startup errors return before motion can be armed.

Egress flow:

```text
RT/local publisher -> latest/event local port -> bridge poll
  -> contract-codec encode -> size check -> Zenoh put/query reply
```

Ingress flow:

```text
Zenoh callback -> bounded nonblocking adapter inbox -> bridge worker
  -> size check -> contract-codec decode -> ingress guard
  -> local ReceivedSample / executable command snapshot
```

If a callback inbox is full, the callback drops according to the descriptor
and increments a typed counter; it does not block a Zenoh runtime thread. The
bridge owns bounded retry/action correlation and query state. Query APIs expose
one project reply or typed `NoReply`, `MultipleReplies`, `RemoteError`,
`MalformedReply` or `Timeout`; arbitrary reply streams do not escape.

Liveliness maps token put/delete to availability observations. It never grants
authority. Boot identity is verified through the configured boot-specific
presence/handshake contract before a new publisher epoch is accepted.

Shutdown is cooperative and outside RT: signal, close declarations/session,
join, then return final counters. A worker panic or session error marks every
owned route unavailable. The latest local values may remain inspectable, but
`usable_at` expires them and no lease is extended.

## 7. Test and fault model

Pure bridge guards receive the local monotonic reception time explicitly. A
scripted sequence of ingress frames deterministically covers:

- delay and jitter;
- loss, duplicate and reorder;
- queue saturation and stopped consumer;
- disconnect/reconnect;
- same-boot resume and authorized publisher restart;
- zero/multiple/error/late query replies;
- action retry, ack correlation and terminal result.

These tests assert exact rejection reasons and saturating counters without
multicast or wall-clock sleeps. A separate serialized loopback test uses an
explicit TCP router/client pair and unique keys to exercise real Zenoh 1.9.0
pub/sub, bounded query timeout, queryable and liveliness declarations. Its
only sleeps are inside a deliberately slow non-RT queryable used to prove the
timeout classification.

Allocation tests warm up the harness, then measure only steady-state local
publish/read operations. Contention tests pause the bridge/reader and assert
producer calls still return immediately with overwrite/full/disconnected
outcomes. No host test is labelled a target-board realtime result.

## 8. Compatibility, operations and rollback

- Protobuf changes are additive within `robot.v1`; Buf format/lint/breaking and
  cross-language generation run before the golden descriptor is updated.
- Key major version remains `v1`; unsupported versions fail parsing rather
  than being coerced.
- No route or Zenoh setting enters robot topology YAML. Runtime Zenoh endpoint
  configuration belongs under `config/zenoh/` in a later deployment task.
- No `unstable`, shared-memory, TLS or QUIC feature is enabled merely because
  Zenoh defaults provide it. Trusted-LAN v1 transport does not preclude later
  authentication/ACL profiles.
- If dependency audit or host tests reject `waitfree-sync` or `rtrb`, retain
  `middleware-api` and replace only `middleware-local`; do not relax RT
  constraints. Target selection still depends on RK3588 evidence.
- If the Zenoh adapter fails, local ports and safety remain usable. Removing
  `middleware-zenoh` leaves local behavior intact.
- Before release, additive intent messages can be reverted with the adapter.
  After publication, removed field numbers/types are reserved and replaced
  additively rather than reinterpreted.

## 9. Deferred follow-up

Create a separate `local-shm-rk3588-qualification` child after this task. It
owns explicit fixed-layout payload traits, layout magic/version/schema ID,
mmap/raw atomics, cache-line layout, torn-read detection, process endpoint
leases, PID reuse assumptions, `SIGKILL` reclamation, Horus comparison and
Orange Pi latency/jitter/cycle/allocation/stress evidence. It must not change
the semantics or counters established by the host local ports.
