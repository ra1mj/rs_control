# Rust SocketCAN and Damiao Driver Implementation Plan

## 1. Prerequisite naming migration

- [ ] Complete and independently commit `07-29-module-layer-naming`.
- [ ] Run its metadata, architecture and full CI gates.
- [ ] Reload the renamed backend specifications before writing hardware code.

## 2. Shared hardware contracts

- [ ] Add validated `StandardCanId`, fixed-storage `ClassicCanFrame`, received
      timestamp metadata, raw `CanErrorEvent` and `CanBusEvent` to
      `hardware-module-core`.
- [ ] Add `Configuring` and its fail-closed lifecycle transitions.
- [ ] Revise `DeviceDriver` to consume an explicit frame, emit configuration and
      lifecycle frames through `FrameBatch`, poll deadlines and accept an
      explicit transport-fault notification.
- [ ] Give generic health counters an explicit unavailable state where a backend
      cannot produce a measurement; retain saturating monotonic measured values.
- [ ] Preserve typed errors, caller-owned buffers and non-blocking guarantees.
- [ ] Add unit tests for every constructor, transition and full-output path.

Validation:

```bash
cargo test -p hardware-module-core
```

## 3. SocketCAN adapter

- [ ] Add `transport-socketcan` under `hardware_module`, workspace membership,
      architecture classification and exact `socketcan = 3.6.2` dependency with
      default features disabled.
- [ ] Implement pure conversions between project standard frames and
      `socketcan` classic data frames; reject extended/RTR/invalid forms.
- [ ] Implement exact standard-ID filter construction, bounded error mask and
      two-phase disarmed filter policy.
- [ ] Implement the production socket constructor with explicit interface,
      CLOEXEC, non-blocking mode, timestamps and startup drain.
- [ ] Put the socket behind a private backend contract and implement bounded
      receive plus accepted-prefix, one-shot transmit logic.
- [ ] Preserve diagnostic timestamps while stamping local `MonoTime` for
      freshness.
- [ ] Implement raw error-bit classification, BUS-OFF TX inhibition and detailed
      saturating health counters; represent kernel-overflow visibility as
      unavailable when not observable.
- [ ] Implement the checked classic-CAN worst-case budget calculator and 70%
      default gate.
- [ ] Add pure fake-backend tests and an ignored opt-in existing-`vcan` test that
      never creates/deletes/configures an interface.

Validation:

```bash
cargo test -p transport-socketcan
```

## 4. Damiao codec and configuration

- [ ] Add `motor-damiao` under `hardware_module` with no dependency on
      `transport-socketcan` or `config-model`.
- [ ] Implement Damiao ESC ID, runtime limits and typed safety-policy values.
- [ ] Implement checked integer/physical conversion and exact MIT packing.
- [ ] Implement feedback decoding, all known/unknown statuses, raw temperatures,
      source identity and sample sequence.
- [ ] Implement pure lifecycle/system frames and read/write/store register
      codecs without exposing automatic persistent-write paths.
- [ ] Implement serialized register transaction correlation/deadlines for
      `MST_ID`, `ESC_ID`, `TIMEOUT`, `CTRL_MODE`, `PMAX`, `VMAX` and `TMAX`.
- [ ] Implement verified configuration and bus-wide ESC/MST/reserved-ID
      collision validation.
- [ ] Add exact manufacturer/SDK-derived golden vectors and exhaustive raw-field
      decoder tests without copying external source code.

Validation:

```bash
cargo test -p motor-damiao codec
cargo test -p motor-damiao register
```

## 5. Safe driver lifecycle

- [ ] Implement `Unconfigured -> Configuring -> Disabled` as a bounded read-only
      handshake.
- [ ] Implement explicit enable/disable handshakes completed only by matching
      fresh feedback.
- [ ] Require verified runtime scaling, bus IDs, motor timeout and safety policy
      before enabled command encoding.
- [ ] Enforce query/feedback correlation, feedback freshness and transition
      deadlines.
- [ ] Send stale/missing enabled feedback through the bounded
      `SafeStop -> Disabling` path; escalate a failed/expired disable to
      `Faulted`.
- [ ] Fault directly on unknown/fault status, malformed or mis-correlated
      expected reply, transition timeout, partial/fatal TX, transport fault and
      BUS-OFF.
- [ ] Keep clear-error disarmed and explicit; prove it never auto-enables.
- [ ] Keep zero/runtime-write/store unreachable from startup/ordinary command
      paths.
- [ ] Forbid `Faulted -> Disabled`; require explicit recovery through a fresh
      `Configuring` handshake.
- [ ] Test the state machine using deterministic fake time/I/O only.

Validation:

```bash
cargo test -p motor-damiao lifecycle
```

## 6. Cross-cutting specifications and evidence

- [x] Update the parent requirements/design/plan: replace the obsolete
      universal static-limit assumption with disarmed register discovery plus
      typed policy bounds.
- [ ] Update robot contracts/error handling with lifecycle-frame and BUS-OFF
      semantics learned during implementation.
- [ ] Keep source pins, license boundary and firmware contradictions in task
      research.
- [ ] Document host-only status and the exact pending target/analyzer gate.

## 7. Quality gate

- [ ] Run `cargo fmt --all -- --check`.
- [ ] Run Clippy with workspace warnings denied.
- [ ] Run:

```bash
cargo test -p hardware-module-core -p transport-socketcan -p motor-damiao
```

- [ ] Run `cargo run -p architecture-check -- check`.
- [ ] Run `tools/ci.sh`.
- [ ] Run `git diff --check` and review every unsafe, blocking, allocation,
      hardware-access and persistent-write call site; expected local unsafe and
      physical hardware calls are both zero.
- [ ] Commit and push host-only implementation separately from any later target
      qualification evidence.

## 8. Pending target qualification

Do not mark hardware support qualified or this target evidence complete until a
separately approved restrained-motor/analyzer procedure verifies exact firmware,
register identity, query/reply behavior, physical timeout stop, enable/disable,
fault handling, bus load and RK3588 timing. Zeroing and persistent parameter
writes remain separate destructive maintenance procedures.
