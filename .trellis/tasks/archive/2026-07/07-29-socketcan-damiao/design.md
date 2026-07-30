# Rust SocketCAN and Damiao Driver Design

## 1. Architecture

This task adds two Linux/hardware-specific crates after the repository naming
migration:

```text
crates/hardware_module/
  hardware-module-core/   # backend-neutral frame, time, lifecycle contracts
  transport-socketcan/    # Linux raw classic-CAN adapter and bus accounting
  motor-damiao/           # DM-J4310-2EC MIT codec/discovery/state machine
```

`motor-damiao` depends on `hardware-module-core`, not
`transport-socketcan`. A later hardware-module bus owner composes both and owns
frame routing. Linux types and file descriptors do not cross into
`protocol_module`; the eventual protocol adapter receives validated samples and
health summaries only.

`07-29-module-layer-naming` is a hard implementation prerequisite. No new crate
is first added under a retired path and moved later.

## 2. Core frame and lifecycle contracts

### 2.1 CAN values

`hardware-module-core` owns small, copyable values:

- `StandardCanId`: validated `0..=0x7ff` identifier;
- `ClassicCanFrame`: standard data frame with fixed `[u8; 8]` storage and
  validated DLC `0..=8`;
- `ReceivedCanFrame`: frame plus the local monotonic observation time and
  optional fixed-size diagnostic timestamp metadata;
- `CanErrorEvent`: raw Linux-compatible error-class bitset, raw eight bytes and
  observation time without depending on a socketcan type;
- `CanBusEvent`: `Data(ReceivedCanFrame)` or `Error(CanErrorEvent)`.

Extended and RTR traffic is rejected by the SocketCAN adapter rather than
silently coerced to a standard data frame. Error frames remain events and are
never passed into the motor feedback decoder as data.

Only `ReceivedCanFrame.observed_monotonic` is valid for local freshness.
Diagnostic timestamp fields carry an explicit origin/clock-domain tag and may
be absent; no wall/PHC timestamp is compared directly with `MonoTime`.

### 2.2 Driver contract evolution

The existing contract cannot consume a frame and cannot emit enable/disable or
multi-step configuration frames. It evolves before the Damiao implementation:

```rust
trait DeviceDriver<F, const MAX_OUTPUT_FRAMES: usize>
where
    F: Copy,
{
    type Command;
    type Sample;
    type Fault;

    fn lifecycle(&self) -> LifecycleState;
    fn start_configure(&mut self, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<(), DeviceError<Self::Fault>>;
    fn start_enable(&mut self, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<(), DeviceError<Self::Fault>>;
    fn start_disable(&mut self, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<(), DeviceError<Self::Fault>>;
    fn handle_input(&mut self, frame: F, received_at: MonoTime, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<Option<Self::Sample>, DeviceError<Self::Fault>>;
    fn poll(&mut self, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<(), DeviceError<Self::Fault>>;
    fn encode_command(&mut self, command: &Self::Command, now: MonoTime, out: &mut FrameBatch<F, MAX_OUTPUT_FRAMES>) -> Result<(), DeviceError<Self::Fault>>;
    fn report_transport_fault(&mut self, now: MonoTime) -> Result<(), DeviceError<Self::Fault>>;
}
```

The signatures are the intended ownership contract; implementation may use a
small output/event result type if that produces clearer typed outcomes. All
output appends to caller-owned storage. A coordinator reports partial/fatal TX
before accepting further commands. `Configuring` is added to the lifecycle and
cannot transition directly to `Enabled`.

## 3. SocketCAN adapter

### 3.1 Dependency and unsafe boundary

Use exact `socketcan` 3.6.2 with default features disabled. Its reviewed Linux
FFI stays external; every project crate continues inheriting
`unsafe_code = "forbid"`. Direct libc/nix frame I/O and an in-repository sys
crate are rejected for this slice.

The adapter wraps the socket behind a private backend trait so deterministic
unit tests can drive the receive/transmit logic without a kernel interface. The
production backend is the only type that opens a named interface.

### 3.2 Two startup phases

1. The privileged platform unit verifies `can0` is up at 1 Mbit/s with
   `berr-reporting on` and `restart-ms 0`.
2. The unprivileged adapter opens the explicit name, applies CLOEXEC and
   non-blocking mode, subscribes to bounded errors/timestamps and installs a
   broad standard-data discovery policy.
3. Startup serializes register discovery for each expected ESC ID while motors
   remain disabled.
4. Cross-device ID validation completes.
5. The adapter replaces discovery filters with exact MST-ID standard-data
   filters, drains startup residue and is frozen for the realtime owner.

Filter replacement allocates inside the dependency, so it is forbidden after
handoff. Receive-own-messages/loopback settings are explicit and test-covered;
the control path does not depend on seeing its own transmitted frames.

### 3.3 Bounded I/O

RX calls `read_frame_with_timestamps()` repeatedly only up to the supplied
output capacity/configured drain ceiling. Each successful read is classified
as data, error or unsupported before publication. `WouldBlock` terminates the
drain successfully. Reaching the ceiling increments `rx_drain_limit_hits`; it
does not claim a drop count.

The chosen API ignores the `SO_RXQ_OVFL` cmsg. This task therefore models kernel
overflow visibility as `Unavailable` and never exposes zero as proof of no
drops. Adding exact kernel drop telemetry later requires either an upstream
safe API or a separately reviewed FFI decision and is a hardware-qualification
follow-up, not silent local unsafe.

TX iterates over the caller slice with one `write_frame()` call per frame. A
typed error/report preserves the accepted prefix count. `WouldBlock`/`ENOBUFS`
increments backpressure; other errors increment TX errors. No retry occurs
inside the adapter.

### 3.4 Error state

The control socket error mask is:

```text
TX_TIMEOUT | CRTL | TRX | ACK | BUSOFF | RESTARTED
```

The event classifier always tests raw bits. `BUSOFF | ACK`, for example, is
still BUS-OFF. BUS-OFF sets `LinkState::Faulted`, blocks normal TX and notifies
all device state machines. A later privileged recovery may recreate the socket
only while disarmed; `RESTARTED` is evidence, not authorization to resume.

`SocketCanHealth` contains detailed monotonic counters and maps conservatively
to the generic `TransportHealth` summary. Generic counter values gain an
explicit measured/unavailable representation where needed, so unsupported
kernel drop telemetry cannot masquerade as zero. Counter overflow saturates
rather than wrapping to a plausible low value.

### 3.5 Bus budget

For a standard classic-CAN data frame with DLC `d`, unstuffed bits including
intermission are `47 + 8d`. The stuffable region is `34 + 8d`; its conservative
maximum inserted bits are `floor((stuffable - 1) / 4)`. The planned worst-case
cost is therefore:

```text
47 + 8d + floor((33 + 8d) / 4)
```

At DLC 8 this is 135 bits. The calculator sums every declared command,
feedback, maintenance and explicit reserve flow with checked integer/rational
arithmetic. A plan passes only when
`planned_bits_per_second * 100 <= bitrate * threshold_percent`; the default
threshold is 70. Error retransmissions are not scheduled capacity and are
covered by the unused margin/fault policy.

For query-style DM MIT traffic, one command normally budgets one feedback
frame. At 1 Mbit/s and DLC 8, one motor at 1 kHz consumes a worst-case 27%; three
motors would exceed the 70% gate. The framework therefore never equates a 1 kHz
control loop with a 1 kHz command/feedback rate for every motor.

## 4. Damiao protocol

### 4.1 Source identity and supported mode

The byte contract is independently implemented from the pinned manufacturer
manual at product commit `9db7863cba1c386ad9b8dedad3286465011ca6b2` and
corroborated with SDK commit `ae360bde6a5af373e5a68440f80708bea321689d`.
The C++ SDK has no clear license and is not copied. Only current MIT mode is
supported; unknown firmware/mode remains disarmed.

### 4.2 IDs and discovery

The YAML motor ID is the expected `ESC_ID`. A Damiao-specific newtype accepts
only `1..=15`, preserving the generic config model's vendor-neutral range.

Register requests use standard ID `0x7ff`. Configuration serially reads:

| RID | Value | Verification |
| ---: | --- | --- |
| `0x07` | `MST_ID` | nonzero standard ID; bus-wide unique/collision-free |
| `0x08` | `ESC_ID` | equals expected ID and Damiao safe range |
| `0x09` | `TIMEOUT` | nonzero and within typed safety policy |
| `0x0a` | `CTRL_MODE` | exactly `1` (MIT) |
| `0x15` | `PMAX` | finite, positive, policy accepted |
| `0x16` | `VMAX` | finite, positive, policy accepted |
| `0x17` | `TMAX` | finite, positive, policy accepted |

Every reply is checked against expected reply ID when known, embedded ESC ID,
opcode, RID and deadline. During the first MST read, the discovery router uses
the embedded ESC ID to associate a candidate reply; after that, the discovered
MST ID is mandatory.

`DamiaoBusSet::validate` runs after every driver reaches a verified disabled
state. All ESC IDs and all MST IDs are unique; an MST ID cannot equal any ESC ID
or `0x7ff`. Final receive filters are derived only from this validated set.

### 4.3 Runtime scaling and command codec

The MIT eight-byte layout is exactly the manufacturer layout. `kp` uses
`[0, 500]` and `kd` uses `[0, 5]`; position, velocity and feed-forward torque use
the verified runtime `PMAX`, `VMAX` and `TMAX`. Conversion uses truncation to
match the pinned SDK vectors. Constructors reject invalid limits, NaN,
infinity and any value outside the selected range.

A typed `DamiaoSafetyPolicy` is supplied outside robot YAML. It bounds accepted
discovered position/velocity/torque limits, maximum motor timeout, feedback age
and transition deadlines. No universal 24 V or 48 V preset is guessed.

### 4.4 Feedback and state

Feedback accepts only an eight-byte standard data frame on the discovered
`MST_ID`, with `data[0] & 0x0f` equal to expected ESC ID and an outstanding fresh
query/command. It yields status, position, velocity, torque and raw temperature
bytes with monotonic receive time and a monotonically increasing sample
sequence.

Known status codes map to typed states/faults. Every unspecified nibble remains
`Unknown(raw)` and faults closed. Missing feedback increments missed/stale
counters and never creates a zero-valued sample.

The current manual describes query-style feedback, so the driver does not rely
on unsolicited periodic frames. If target capture reveals unsolicited frames,
they remain diagnostic until firmware-specific behavior is explicitly
qualified.

### 4.5 Lifecycle and maintenance separation

```text
Unconfigured -> Configuring -> Disabled -> Enabling -> Enabled
Enabled/Enabling -> SafeStop -> Disabling -> Disabled
any configuration/active state -> Faulted
Faulted --explicit recovery authorization--> Configuring
```

The exact transition table in code remains stricter than this overview:

- configuration only issues bounded read requests;
- enable/disable frames begin transitions but feedback status completes them;
- enabled commands require fresh feedback and verified scaling;
- deadline/fault/transport events stop command acceptance immediately;
- clear error is explicit while disarmed and completes only with valid safe
  feedback; it never enables;
- BUS-OFF faults without attempting to transmit a frame on the dead bus;
- recovery returns through configuration and explicit enable authorization.

Failure-state ownership is exact:

| Event | Immediate state/action |
| --- | --- |
| stale or missing feedback while enabling/enabled and transport still usable | `SafeStop`; reject commands and issue one bounded disable handshake |
| safe-stop disable confirmed by matching fresh feedback | `Disabled`; invalidate the prior configuration generation before any later enable |
| safe-stop disable submission/confirmation fails or expires | `Faulted` |
| unknown/fault motor status, malformed or mis-correlated expected reply | `Faulted` |
| configuration, enable or disable transition deadline expires | `Faulted` |
| partial/fatal TX, reported transport fault or BUS-OFF | `Faulted`; do not attempt traffic on a dead link |

`Faulted -> Disabled` is forbidden. An explicitly authorized recovery first
restores a usable transport when necessary and then transitions to
`Configuring`; only a complete read/verify handshake may reach `Disabled`.
Unrelated frames excluded by routing/filter policy are accounted for and
ignored, but a frame presented as the outstanding device reply and failing its
identity/correlation checks is a protocol fault.

Enable, disable, zero and clear-error bytes plus register read/write/store bytes
are pure codec facts and receive host tests. Only read and guarded
enable/disable/clear paths are available to the ordinary driver. Zero, runtime
writes and flash store live behind a separate disarmed maintenance surface that
this task does not wire into any executable.

Process death cannot run a guaranteed cleanup handler. The motor's verified
nonzero communication timeout is therefore part of the arming precondition;
physical timeout behavior remains a target test.

## 5. Test design

### 5.1 Pure host tests

- standard ID/DLC construction and every invalid boundary;
- lifecycle transition table and caller-owned output-full behavior;
- SocketCAN conversion, filters, raw error masks and detailed counters;
- combined BUS-OFF error bits;
- bounded RX and accepted-prefix TX with a fake backend;
- worst-case bus bits for DLC 0 through 8, overflow and threshold edges;
- exact MIT and feedback golden vectors;
- exhaustive decode over all 16-bit and 12-bit integer fields;
- all 16 status nibbles and arbitrary `[u8; 8]` feedback payloads;
- register bytes, correlation mismatches and deadlines;
- configuration/lifecycle/freshness/fault behavior using deterministic time;
- bus-wide ESC/MST collision matrix.

No default test opens a CAN device. Optional `vcan` tests require an explicit
nonphysical interface name, are ignored by default and perform no link setup.

### 5.2 Hardware gate kept pending

With a restrained motor, analyzer, emergency-stop procedure and separate
operator approval, a later task records hardware/firmware versions, register
values, one query/reply capture, feedback behavior, timeout stop, lifecycle
transitions, error recovery, load and RK3588 timing. It never writes zero, IDs,
bitrate, mode or flash during initial qualification.

## 6. Operational rollback

There is no hardware mutation in this implementation task. A code rollback
removes the two crates and restores the prior core trait. During later hardware
qualification, rollback means stop commands, verify physical disable/timeout,
preserve evidence and restore only parameters changed by an explicitly approved
maintenance procedure; this implementation never bulk-writes motor flash.
