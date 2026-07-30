# Rust SocketCAN and Damiao Driver

## Goal

Implement a safe, bounded Rust hardware path for Linux classic SocketCAN and a
DM-J4310-2EC MIT-mode motor without transmitting to physical hardware in the
default development or CI workflow.

## Background

The Orange Pi 5 Plus platform task establishes an RK3588 candidate with classic
CAN at 1 Mbit/s, CAN error reporting enabled and automatic bus restart disabled.
It does not qualify real hardware or transmit motor frames.

Manufacturer research pins the DM-J4310-2EC V1.2 manual and SDK sources and
shows that `PMAX`, `VMAX`, `TMAX`, motor IDs, mode and communication timeout are
runtime registers. The driver must discover and verify them while disarmed;
hard-coding one SDK preset as universal product truth would be unsafe.

This task depends on `07-29-module-layer-naming`. All implementation paths and
architecture classes below use the canonical names `hardware_module`,
`protocol_module` and `application_module`; the dependency is explicit rather
than implied by both tasks sharing a parent.

## Requirements

### 1. Architecture and shared contracts

- **HD-1:** Complete `07-29-module-layer-naming` before creating new crates.
- **HD-2:** Add `transport-socketcan` and `motor-damiao` under
  `crates/hardware_module/`; both may depend only on `hardware_module` crates and
  approved external libraries.
- **HD-3:** Keep Linux/socketcan types private to `transport-socketcan`. Put the
  backend-independent 11-bit classic-CAN frame, received-frame metadata and bus
  event contracts in `hardware-module-core` so the Damiao codec does not depend
  on Linux.
- **HD-4:** Evolve the existing device-driver contract to accept an explicit
  input frame, emit lifecycle/configuration frames into caller-owned fixed
  storage, poll bounded deadlines and enter a safe state on a reported transport
  fault. Add a `Configuring` lifecycle state for the multi-frame read/verify
  handshake.
- **HD-5:** Realtime-facing receive, transmit, codec, lifecycle and deadline APIs
  are synchronous, bounded, caller-buffered and contain no heap allocation or
  blocking retry loop.
- **HD-6:** Robot YAML remains unchanged. It supplies only expected motor IDs;
  protocol settings and safety timing policy remain typed deployment/profile
  inputs outside that document.
- **HD-7:** Revise generic transport health where necessary so an unavailable
  counter is distinguishable from a measured zero. Monitoring must not infer
  “no loss” from an unsupported kernel metric.

### 2. SocketCAN adapter

- **SC-1:** Pin `socketcan` exactly to 3.6.2 with default features disabled.
  Preserve the workspace `unsafe_code = "forbid"`; do not add local libc/FFI.
- **SC-2:** Open only an explicitly supplied interface that the privileged
  platform service already configured. Set close-on-exec and non-blocking mode,
  and never bring the interface up/down, change bitrate or restart it.
- **SC-3:** Separate disarmed initialization from the realtime path. Filter
  construction/replacement, socket option changes and startup draining occur
  only before arming.
- **SC-4:** Support a discovery-stage filter policy followed by exact standard
  data-frame feedback filters. Exact filters include `CAN_SFF_MASK`,
  `CAN_EFF_FLAG` and `CAN_RTR_FLAG`, rejecting extended/RTR aliases.
- **SC-5:** Subscribe the control socket only to bounded safety-relevant CAN
  error classes. Preserve the raw error bitmask and raw bytes; detect BUS-OFF by
  bit test even when it is combined with other error bits.
- **SC-6:** Drain at most a caller/configuration-defined number of frames per
  call. Treat `WouldBlock` as queue-empty. Make one bounded transmit attempt per
  frame and report partial acceptance/backpressure; never call
  `write_frame_insist`.
- **SC-7:** Record a local monotonic observation timestamp for freshness and
  preserve available kernel software/socket/hardware timestamps only as
  diagnostic clock-domain metadata.
- **SC-8:** Expose monotonic counters for RX/TX, malformed/unsupported frames,
  drain-limit hits, backpressure, CAN error classes, BUS-OFF and restart events.
  If the selected safe API cannot observe the kernel `SO_RXQ_OVFL` counter,
  expose that capability as unavailable rather than reporting a false zero.
- **SC-9:** BUS-OFF faults the link and stops normal transmission. Recovery is
  an explicit, privileged, disarmed platform operation; recovery never
  auto-enables any motor.
- **SC-10:** Implement an integer-overflow-checked classic-CAN budget calculator
  using the standard-frame worst-case bit-stuffing bound and reject plans above
  a configurable threshold whose default is 70% of configured bitrate.

### 3. DM-J4310-2EC MIT codec and discovery

- **DM-1:** Implement only current MIT mode on standard 11-bit classic-CAN data
  frames. Do not silently fall back to legacy firmware or other control modes.
- **DM-2:** Encode the exact eight-byte MIT command layout. Reject non-finite and
  out-of-range physical values; do not silently clamp.
- **DM-3:** Decode exact eight-byte feedback into raw status, motor identity,
  position, velocity, torque, MOS temperature and rotor/winding temperature.
  Preserve unknown status nibbles and treat them as faults.
- **DM-4:** Apply the Damiao-specific safe `ESC_ID` policy `1..=15` at the vendor
  boundary while leaving the generic YAML standard-CAN range unchanged.
- **DM-5:** During serialized, disarmed configuration, read and correlate
  `MST_ID`, `ESC_ID`, `CTRL_MODE`, `TIMEOUT`, `PMAX`, `VMAX` and `TMAX`. Require
  the observed ESC ID to equal the expected YAML ID, mode to equal MIT, IDs to
  satisfy the bus collision policy, and scaling limits to be finite, positive
  and accepted by a typed compiled/deployment safety policy.
- **DM-6:** Treat manufacturer SDK presets only as pinned host-test vectors. The
  production codec uses the verified values read from registers and refuses to
  arm before scaling identity is known.
- **DM-7:** Build exact frames for enable, disable, save-position-as-zero, clear
  error and register read/write/store operations. Ordinary startup and command
  APIs may read only; zero, runtime writes and flash store remain disarmed
  maintenance-only operations with no automatic call path.
- **DM-8:** Correlate register replies by arbitration ID, embedded target ID,
  opcode, register ID and deadline. Correlate state feedback by final `MST_ID`,
  embedded low-nibble identity and an outstanding query/command window.
- **DM-9:** Validate the complete discovered bus set before arming: expected ESC
  IDs are unique, discovered MST IDs are unique/nonzero, no MST ID collides with
  a command ID or `0x7ff`, and every device has a verified configuration.

### 4. Safe lifecycle

- **LC-1:** The driver starts `Unconfigured`, performs a bounded
  `Configuring` handshake, and reaches `Disabled` only after every required
  register value is verified.
- **LC-2:** Enable and disable are explicit handshakes. Successful frame
  submission alone does not complete either transition; matching fresh status
  feedback must confirm enabled or disabled state before its deadline.
- **LC-3:** An enabled command is accepted only with a verified configuration,
  an explicit enable request, fresh matching feedback and a satisfied typed
  timeout/safety policy.
- **LC-4:** Stale or missing feedback while enabling/enabled first enters
  `SafeStop`, stops command acceptance and starts one bounded disable handshake
  when the transport remains usable. Unknown/fault status, malformed or
  mis-correlated reply, transition timeout, partial/fatal TX, transport fault or
  BUS-OFF enters `Faulted` directly; failure to confirm the safe-stop disable by
  its deadline escalates from `SafeStop` to `Faulted`. Missing data is never
  replaced with numeric zero.
- **LC-5:** Clear-error is an explicit disarmed action, never implies enable and
  is never retried automatically. Position-zero and flash-store operations have
  no production startup or automatic-recovery route.
- **LC-6:** Process death cannot rely on Rust destructors. Arming requires a
  nonzero motor communication timeout that satisfies the supplied safety policy;
  target qualification must separately prove that timeout stops output.
- **LC-7:** `Faulted` cannot transition directly to `Disabled`. Recovery after
  any fault requires explicit authorization, a new bounded configuration
  handshake and a new explicit arming sequence. No link-restart,
  process-restart or feedback event causes automatic re-enable.

### 5. Evidence and safety boundary

- **EV-1:** Default tests use pure codecs and fake backends only. They do not
  open `can0`, configure networking or send any lifecycle/control frame.
- **EV-2:** An optional ignored test may use an already-created, explicitly
  named nonphysical `vcan` interface; it does not create or delete interfaces.
- **EV-3:** Host evidence is labelled host-only. Real firmware compatibility,
  physical disable, timeout behavior, bus fault recovery and RK3588 timing remain
  a separately approved hardware qualification gate.
- **EV-4:** Preserve source identity, commit/blob hashes and protocol caveats in
  task research; independently implement wire facts without copying unlicensed
  manufacturer C++ code or redistributing the manual.

## Acceptance Criteria

- [x] The naming-migration task is complete and its full CI gate passed before
      any SocketCAN/Damiao implementation commit.
- [x] `hardware-module-core` exposes copyable, validated classic-CAN and
      received-event types, explicit metric availability and the revised
      bounded device lifecycle contract.
- [x] `transport-socketcan` compiles with `unsafe_code = "forbid"`, exact
      `socketcan = 3.6.2`, default features off and no async runtime.
- [x] Pure tests cover frame conversion, exact filters, all subscribed error
      bits, combined `BUSOFF | other` classification, timestamps/capabilities,
      partial/backpressured TX, bounded RX and health counters.
- [x] The budget calculator proves the worst-case bit count at every DLC
      `0..=8`, handles arithmetic overflow and rejects a plan one unit above the
      70% threshold.
- [x] MIT tests match every pinned manufacturer/SDK golden vector, exhaust raw
      16-bit and 12-bit decode domains, cover encode boundaries/non-finite input
      and never panic for arbitrary eight-byte payloads.
- [x] Feedback tests cover every documented status and every unknown status
      nibble, wrong ID, wrong embedded identity, RTR/extended/error input and
      every invalid DLC.
- [x] Register tests cover exact little-endian read/write/store bytes and reject
      wrong reply ID, target, opcode, register, length and deadline.
- [x] Fake-I/O lifecycle tests prove configuration-before-enable, verified
      scaling, explicit enable/disable confirmation, stale/fault/bus-off
      fail-safe behavior, safe-stop escalation, timeout enforcement, forbidden
      `Faulted -> Disabled` and no automatic re-enable.
- [x] Bus-set validation rejects every ESC/MST/reserved-ID collision.
- [x] A source review finds no heap collection, sleep, blocking retry, async
      call, filesystem/network configuration or persistent motor write in any
      declared realtime path.
- [x] `cargo test -p hardware-module-core -p transport-socketcan -p motor-damiao`
      and `tools/ci.sh` pass on the host without accessing physical CAN.
- [x] Documentation explicitly states that hardware support remains
      unqualified until the target-board/analyzer gate passes.

## Out of Scope

- Physical motor enable, disable, zero, parameter write or flash store.
- Creating/deleting `vcan`, changing `can0`, or privileged bus-off recovery.
- Non-MIT Damiao modes, CAN FD and vendor USB-adapter framing.
- Choosing a deployment-specific motor voltage/limit/timeout policy without
  target identity and qualification evidence.
- The `protocol_module` Zenoh/Protobuf bridge, Horus-inspired shared memory,
  application controllers, FSM and full multi-motor realtime process.
- Claiming RK3588 deadline, physical watchdog or motor firmware qualification.
