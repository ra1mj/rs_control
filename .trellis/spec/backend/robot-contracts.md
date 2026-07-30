# Robot Cross-Layer Contracts

## Scenario: Add or change a robot contract

### 1. Scope / Trigger

Use this contract whenever work changes a `hardware_module` trait, robot YAML field, semantic `protocol_module` message, Protobuf field, command authority field, codec or composition-root startup mapping. These are safety/cross-layer changes even when the diff is small.

### 2. Signatures

Current executable commands:

```text
robotctl config validate <path>
robotctl config print-normalized <path>
architecture-check check [--workspace-root <path>] [--policy <path>]
```

Core Rust boundaries:

```rust
RobotConfig::parse(yaml: &str) -> Result<RobotConfig, ConfigError>
Header::new(fields: HeaderFields) -> Result<Header, SemanticError>
ResourceLease::new(fields: ResourceLeaseFields) -> Result<ResourceLease, SemanticError>
CommandEnvelope::new(fields: CommandEnvelopeFields) -> Result<CommandEnvelope, SemanticError>
FrameTransport::{receive, transmit, health}
DeviceDriver::{lifecycle, start_configure, start_enable, start_disable,
              handle_input, poll, encode_command, report_transport_fault}
```

### 3. Contracts

Robot YAML allows only:

```yaml
schema_version: 1
chassis: { morphology: <compiled chassis>, motor_ids: <role -> id> }
arm: { morphology: <profile name>, motor_ids: <role -> id> }
imu: { transport: uart|usb, baud_rate: <1200..4000000> }
```

It never contains frame layouts, checksums, controller gains, device paths, Zenoh settings or executable expressions.

Every external `Header` contains source ID, 16-byte non-zero boot ID, boot-local sequence, source monotonic timestamp, optional Unix correlation timestamp, non-zero validity, optional frame ID, clock state/uncertainty and quality. Commands additionally carry resource ID, owner ID, 16-byte lease/command IDs, non-zero session epoch and deadline.

Protobuf package and key-space major version are `robot.v1`. Enums reserve zero as `<ENUM>_UNSPECIFIED`; removed field numbers are reserved and never reused. SI units appear in field names/comments. Coordinates are right-handed: x forward, y left, z up, radians for angles.

Local cross-process fast paths remain protocol-module implementations behind the
same semantic ports. Shared memory uses an explicit fixed-layout payload
contract, layout magic/version/schema identity, bounded capacity, cache-line
separated SPSC ownership, torn-read detection, crash-reclaim leases and
sequence/drop counters. `application_module` code never opens the shared-memory object. Rust
`needs_drop::<T>() == false` is not evidence that a type is safe for shared
memory; any mmap/raw-atomic `unsafe` stays private to a reviewed `protocol_module` boundary.

Classic-CAN device paths use `StandardCanId`, fixed-storage
`ClassicCanFrame`, local-monotonic `ReceivedCanFrame` and raw `CanErrorEvent`
contracts from `hardware-module-core`. Diagnostic socket/kernel/hardware
timestamps retain explicit clock-domain tags and are never compared with
`MonoTime`. Device lifecycle output is appended to caller-owned `FrameBatch`
storage; configuration and enable/disable are bounded multi-frame handshakes,
not successful-write aliases.

Damiao robot YAML IDs are expected `ESC_ID` values. The vendor boundary narrows
them to `1..=15`, discovers `MST_ID`, `TIMEOUT`, `CTRL_MODE`, `PMAX`, `VMAX` and
`TMAX` read-only while disarmed, validates the complete bus ID set, and requires
an explicit enable after verification. Runtime writes, zero-position and flash
store are maintenance-only codecs with no ordinary driver call path.

### 4. Validation & Error Matrix

| Condition | Owner | Result |
|---|---|---|
| Unknown YAML field/type | `config-model` parser | `ConfigError::Syntax` |
| Unsupported schema version | `config-model` | `UnsupportedSchemaVersion` |
| Chassis roles differ from morphology | `config-model` | `RoleMismatch` |
| Duplicate/out-of-range group motor ID | `config-model` | typed duplicate/range error |
| Non-`none` arm role mismatch | later `application_module` profile + composition root | refuse startup/arming |
| Missing wire submessage | `contract-codec` | `CodecError::MissingField` |
| Unknown/zero wire enum | `contract-codec` | `CodecError::UnknownEnum` |
| ID not 16 bytes/all zero | `message-model` | `Id128Error` |
| Zero validity/session/deadline | `message-model` constructor | `SemanticError` |
| Expired lease/command | enforcement/codec at supplied monotonic time | `SemanticError::Expired` |
| Forbidden Cargo edge | `architecture-check` | non-zero exit with all violations |
| Incompatible SHM layout/schema/endianness | `protocol_module` local IPC boundary | reject attach before exposing a port |
| Full/slow/crashed SHM endpoint | `protocol_module` queue policy | explicit latest/drop/reject result plus counters; producer never blocks indefinitely |
| Extended/RTR/malformed classic CAN input | `transport-socketcan` | discard with a saturating typed counter; never coerce into a device frame |
| BUS-OFF or partial/fatal CAN transmit | transport + bus owner | fault link, report every affected driver, inhibit normal TX, require explicit disarmed recovery |
| Missing/stale Damiao feedback | `motor-damiao` | bounded safe-stop disable path or typed fault; never synthesize numeric zero |
| Unknown/fault motor status or mismatched reply | `motor-damiao` | `Faulted`; recovery must repeat read-only configuration and explicit enable |

### 5. Good/Base/Bad Cases

- Good: `mecanum_4` with exactly four compiled roles, unique IDs and a valid UART IMU; normalization is stable and produces a recorded SHA-256.
- Base: arm morphology `none` with `{}` motor IDs.
- Bad: placing CAN bitrate in robot YAML, mapping two chassis roles to one ID,
  accepting `UNSPECIFIED` quality, sending an expired command, letting
  `application_module` code
  mmap IPC directly or treating a destructor-free Rust type as shared-memory POD.

### 6. Tests Required

- Config unit tests assert every chassis profile, deterministic parse-normalize-parse/hash, unknown fields/version, role mismatch, arm identifier/`none`, duplicate/range IDs, transport/baud and size bound.
- CLI integration tests assert exit status and byte-stable output.
- Codec tests assert header/lease/command round trips, malformed bytes, missing submessages, identity length, unknown enums and expiry.
- Architecture fixtures assert application_module-to-hardware_module, hardware_module-to-protocol_module, protocol_module-to-application_module and production-to-test-support rejection; the real graph must also pass.
- `tools/bufw lint` and `breaking` plus `proto-compat-check` are mandatory for schema changes.
- `protocol_module` local IPC tests assert incompatible-layout rejection, torn reads,
  endpoint exhaustion, slow consumers, sequence/drop accounting and `SIGKILL`
  lease reclamation; target benchmarks compare it with the pinned Horus commit.
- SocketCAN tests use a private fake backend for exact SFF filters, bounded RX,
  one-shot accepted-prefix TX, raw combined error bits, unavailable kernel-drop
  telemetry, BUS-OFF inhibition and checked classic-CAN budgeting. The optional
  ignored `vcan` test opens only a caller-provided nonphysical interface.
- Damiao tests assert manufacturer-derived golden vectors, every raw status,
  exhaustive 16/12-bit decode domains, exact register correlation, read-only
  configuration, bus collisions, feedback freshness and fail-closed lifecycle
  transitions without opening a CAN interface.

### 7. Wrong vs Correct

#### Wrong

```rust
// Generated DTO leaks into application_module code and may contain quality = 0/999.
fn control(input: proto_generated::robot::v1::Header) { /* ... */ }
```

#### Correct

```rust
// The protocol_module boundary owns fallible conversion; application_module code receives a valid semantic type.
let header = contract_codec::decode_header(bytes)?;
controller.step(TimedInput { header, value })?;
```
