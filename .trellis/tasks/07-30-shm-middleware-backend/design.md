# Shared-memory middleware backend — Technical design

## 1. Architecture and ownership

```text
application/control code
        |
        v
middleware-api traits and outcomes
        |
        +----------------------+-----------------------+
        |                      |                       |
middleware-local       middleware-shm          middleware-zenoh
safe in-process        Linux cross-process     non-RT network bridge
waitfree/rtrb           fixed-layout mmap       Protobuf/TCP
```

`middleware-shm` is a protocol-module implementation. Callers receive typed
publisher/subscriber capabilities; they never receive a path, `File`, mapping,
lock, raw pointer or layout header. Construction and cleanup are startup/admin
operations and are deliberately separate from hot methods.

The first SHM layout is Linux-only, little-endian and SPSC. That matches the
already-public nonclonable capabilities and avoids introducing an MPMC
arbitration contract merely because the Horus comparison implements fanout.

## 2. Public surface

Illustrative signatures (final names may tighten during implementation):

```rust
pub trait ShmPayload: bytemuck::Pod + Send + Sync + 'static {
    const SCHEMA_ID: [u8; 16];
    const SCHEMA_VERSION: u32;
}

pub trait ShmCodec<T>: Send + 'static {
    type Repr: ShmPayload;
    type Error;

    fn encode(&mut self, value: &T) -> Self::Repr;
    fn decode(&mut self, value: Self::Repr) -> Result<T, Self::Error>;
}

pub struct ShmNamespace { /* private root */ }
pub struct ShmChannelConfig {
    pub capacity: NonZeroU32,
    pub memory_policy: MemoryPolicy,
}

impl ShmNamespace {
    pub fn create_latest<R: ShmPayload>(...) -> Result<(), ShmError>;
    pub fn attach_latest_publisher<T, C: ShmCodec<T>>(..., codec: C) -> Result<ShmLatestPublisher<T, C>, ShmError>;
    pub fn attach_latest_subscriber<T, C: ShmCodec<T>>(..., codec: C) -> Result<ShmLatestSubscriber<T, C>, ShmError>;
    pub fn create_ordered<R: ShmPayload>(...) -> Result<(), ShmError>;
    pub fn attach_ordered_publisher<T, C: ShmCodec<T>>(..., codec: C) -> Result<ShmEventPublisher<T, C>, ShmError>;
    pub fn attach_ordered_subscriber<T, C: ShmCodec<T>>(..., codec: C) -> Result<ShmEventSubscriber<T, C>, ShmError>;
    pub fn remove_if_unused(...) -> Result<(), ShmError>;
}
```

Creation owns only the channel format under an exclusive region lock; it does
not claim an endpoint. Publisher and subscriber attach independently after
full validation, which permits either startup order and exact crash recovery.

`ShmPayload` is safe to implement because `bytemuck::Pod` carries the unsafe
bit-validity/layout promise. Schema constants are validated for nonzero values
and are compatibility identity, not memory-safety proof. `ShmCodec` keeps the
semantic public `T` distinct from its fixed representation: encoding valid
semantic state is infallible/bounded, while decode validation may reject
corrupt bytes and increment diagnostics without exposing an invalid `T`.
An identity codec is provided only for `T: ShmPayload`.

## 3. Namespace and file lifetime

One channel uses:

```text
<private-root>/<channel>.region
<private-root>/<channel>.region.lock
<private-root>/<channel>.publisher.lock
<private-root>/<channel>.subscriber.lock
```

- The root is created mode 0700; regular files are mode 0600.
- Channel names are 1..=96 ASCII bytes from `[a-z0-9._-]`, cannot start with
  `.`, cannot contain `..`, and are never interpolated from a raw route.
- Region creation uses `create_new`; it never truncates an existing object.
- Every mapping holds a shared region lock. Initialization and explicit
  removal require the exclusive region lock and nonblocking checks of both
  endpoint locks.
- Endpoint locks are exclusive and nonblocking. The held file handle is stored
  in the endpoint and has no hot-path use.
- Destructors clear derived metadata and release locks but never unlink files.
- The trust boundary is a private runtime directory owned by the service UID.
  Advisory locks protect cooperating project processes; arbitrary same-UID
  file mutation is outside this trusted local deployment boundary and is
  stated in the unsafe proof.

## 4. Layout v1

The map base and every dynamic section are at least 64-byte aligned. All offset
calculation uses checked `usize` operations before mapping access.

```text
0x000  cache line 0: immutable identity + atomic ready/magic
0x040  cache line 1: producer generation/head + published/common counters
0x080  cache line 2: consumer generation/tail + consumed/common counters
0x0c0  cache line 3: endpoint epochs/diagnostic PID fields/extra counters
0x100  slot area: capacity * aligned(payload bytes)
...    version area: capacity * AtomicU64, aligned to 64
```

Identity contains layout magic/version, endianness, required atomic width,
channel kind, capacity, payload schema ID/version, payload size/alignment, slot
stride, version offset and total length. Attach checks all fields against the
type/config and the exact file metadata length.

Creation initializes atomics and slots, prefaults/touches every page, applies
the requested map lock, then publishes the ready magic with `Release`. Attach
loads ready with `Acquire` before reading immutable fields. The library never
resizes or remaps a live file.

## 5. Contained unsafe boundary

Only `src/mapping.rs` opts out of the workspace `unsafe_code` deny. The crate
root and all other modules keep unsafe denied. `mapping.rs` also denies unsafe
operations inside unsafe functions so every raw action has its own block.

The boundary owns:

1. `memmap2::MmapOptions::map_mut` after fixed-length/private-file validation.
2. Placement initialization of the aligned header/atomic arrays before ready.
3. Converting validated offsets to atomic references.
4. Copying `T: ShmPayload` bytes into/out of aligned slots.

It does not expose raw references or pointers publicly. Atomics are never mixed
with non-atomic access after ready. Payload bytes are accessed only by the
single producer and a bounded seqlock reader. The map object outlives every
private view because endpoints own it directly.

## 6. Latest-value protocol

Use at least three slots. Logical position `p` maps to `p % capacity` and has a
version stamp:

```text
writing p = (p << 1) | 1   // odd
complete p = (p << 1)      // even
```

Publication:

1. Load producer-owned position.
2. Store odd stamp, then a release fence.
3. Copy the Pod bytes into the slot.
4. Store even stamp `Release`.
5. Publish head/generation `Release` and update common counters.

Read:

1. Load head `Acquire`; if unchanged/zero, return `None`.
2. Select newest logical position.
3. Require the exact expected even stamp, copy to aligned `MaybeUninit<T>`,
   issue the acquire fence/load, and require the same stamp again.
4. On mismatch, discard bytes without constructing an invalid `T` and retry up
   to a fixed constant. After exhaustion, return `None` and increment the
   backend read-race counter.
5. On success, advance consumer generation and common counters.

Because every bit pattern of the codec representation is valid, a temporary
byte copy cannot create an invalid Rust value. It is not passed to the codec
unless both stamp checks agree, and a decode error is counted and suppressed.

## 7. Ordered-event protocol

The ordered ring uses producer-owned head and consumer-owned tail on separate
cache lines. Capacity is fixed and nonzero. A slot version identifies the
exact logical generation occupying it; odd/even publication prevents torn
reads using the same bounded helper as latest.

- `head - tail >= capacity` yields `EventPublishError::Full(value)` immediately.
- Missing subscriber lease yields `Disconnected(value)` immediately.
- A successful publish advances head only after the slot is complete.
- Receive reads exactly tail, validates the stamp, then advances tail.
- A new/reclaimed subscriber sets tail to current head under its exclusive
  lease. Pending events owned by the dead subscriber are counted as skipped;
  they are never silently replayed to a replacement process.

## 8. Endpoint ownership and crash recovery

The endpoint lock is authority; active bits, PID and endpoint epoch are
diagnostics. Claim order is:

1. Open the deterministic endpoint lock file.
2. `try_lock` exclusively.
3. On `WouldBlock` or any error, return typed occupied/error; never reclaim.
4. After holding the lock, repair stale derived metadata, increment endpoint
   epoch, and record the current PID diagnostically.
5. For a subscriber, reset tail to head and account abandoned backlog.

On clean drop, clear active/PID metadata before the lock handle closes. On
`SIGKILL`, the kernel closes the descriptor. The next process proves
abandonment by acquiring the lock and repairs metadata. PID reuse, `/proc`
visibility, wall clocks and heartbeat expiry never grant ownership.

## 9. Error model and diagnostics

Startup errors distinguish invalid name, exists/not found, occupied endpoint,
unsupported target/endian/atomic width, wrong object type, layout mismatch,
schema mismatch, corrupt length/offset, arithmetic overflow, prefault/mlock
failure, file/lock/map IO, and cleanup while live. Backend diagnostics also
distinguish semantic codec rejection from seqlock races.

Hot methods retain the middleware port outcomes. `ShmDiagnostics` adds
saturating counters for attach rejection, bounded seqlock races, endpoint
reclaims and abandoned events skipped. It contains only integers and is read
without formatting or allocation.

## 10. Testing and fault injection

- Pure layout tests validate every checked arithmetic and compatibility field.
- Compile-time examples accept a padding-free `repr(C)` payload and reject
  bool/pointer/padding-bearing payloads through `bytemuck::Pod` bounds.
- A private test failpoint pauses after the odd stamp; the parent kills the
  writer and proves readers expose no torn value.
- Linux integration tests spawn the current test executable as a peer with
  explicit env/arguments and bounded readiness files/pipes. `Child::kill` and
  `wait` provide the real abrupt-exit path.
- Allocation tests warm all paths before `allocation_counter::measure`.
- A small concurrency model exercises publish/read ordering; full mmap process
  behavior remains covered by real Linux tests.

No test opens a production namespace, requires root, mutates kernel settings,
or runs unboundedly.

## 11. Benchmark handoff

`middleware-qualification` is an integration-layer binary because it composes
multiple protocol backends. It accepts bounded payload/capacity/warm-up/sample
settings, uses fixed-layout benchmark payloads (64/256/1024 bytes), and emits a
versioned JSON result only after the measurement. Host invocations are labeled
`host`; target identity and qualification verdict remain the next child's job.

The binary preallocates its histogram/sample state, measures zero hot
allocations, uses a monotonic clock shared across Linux processes, and records
common/backend counters. Horus is run externally by the physical child from
the locked source commit; it is not a Cargo dependency.

## 12. Rollback

- If the unsafe review fails, remove `middleware-shm` and the benchmark's SHM
  mode; `middleware-api`, `middleware-local`, and Zenoh remain valid.
- A rejected layout is never migrated in place. Stop endpoints, preserve the
  old object for diagnosis, explicitly remove it, then create layout v1.
- No drop path deletes data, kills a process, changes scheduler state or marks
  a target profile qualified.
