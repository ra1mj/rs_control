# Map motor feedback contract

## Goal

Complete the parent framework's software-only D2 feedback path by mapping one
validated hardware-module motor sample into a backend-neutral semantic
`MotorState` and its existing `robot.v1.MotorState` Protobuf contract exactly
once at the protocol-module boundary.

## Background

`motor-damiao::DamiaoFeedback` already exposes validated position, velocity,
torque, status, local observation time, sequence, and two raw temperature
bytes. `proto/robot/v1/device.proto` already declares `MotorState`, but
`message-model` has no matching semantic type and `contract-codec` has no
motor-state or Damiao mapping.

The Damiao temperature bytes are explicitly unqualified: their signedness,
sentinels, and Celsius conversion have not been established by physical
hardware evidence. The protocol path must preserve those bytes without
claiming that they are calibrated temperatures.

## Requirements

- **MFC-01:** `message-model` owns a validated, fixed-identifier `MotorState`
  with header, resource, SI position/velocity/torque, two explicit temperature
  observations, and a stable numeric fault code.
- **MFC-02:** Motor physical values and calibrated temperature observations
  reject NaN and infinity at construction. Raw temperature observations remain
  explicit unsigned values and are never silently converted to Celsius.
- **MFC-03:** The Protobuf contract represents each temperature as either a raw
  value or calibrated Celsius. The change is additive and retains decoding of
  the original Celsius fields without reinterpreting their field numbers.
- **MFC-04:** `contract-codec` is the only owner of semantic/Protobuf motor-state
  conversion and the only workspace boundary that maps `DamiaoFeedback` into a
  protocol-module `MotorState`.
- **MFC-05:** Damiao mapping uses the feedback's local monotonic observation
  time and sequence in a locally-clocked header. Stable source/boot/resource
  identity and validity are supplied by a validated mapping context.
- **MFC-06:** Disabled/enabled feedback maps to `DataQuality::Valid` and fault
  code zero. Any fault or unknown Damiao status maps to
  `DataQuality::Faulted`; its nonzero driver-owned fault code uses a fixed
  Damiao namespace and preserves the raw status nibble in its low bits.
- **MFC-07:** The semantic Damiao mapping remains allocation-free and does not
  import SocketCAN, Linux descriptors, generated DTOs, or an application
  module.
- **MFC-08:** Buf format/lint/breaking and cross-language generation pass
  against the pre-change descriptor before the golden descriptor is updated.

## Acceptance criteria

- [x] Semantic construction accepts finite good/base cases and rejects every
      non-finite physical or calibrated-temperature field.
- [x] Semantic-to-Protobuf-to-semantic tests cover calibrated and raw
      temperatures, missing/ambiguous new observations, and legacy Celsius
      fields.
- [x] Damiao mapping tests cover enabled, disabled, known fault, and unknown
      status, with exact header identity/time/sequence, fault code, and raw
      temperature preservation.
- [x] `architecture-check` accepts the new protocol-to-hardware dependency and
      no other crate maps Damiao feedback fields.
- [x] The old golden descriptor reports the schema change as non-breaking;
      the reviewed descriptor is then regenerated deterministically.
- [x] `tools/ci.sh` passes.
- [x] Parent implementation D2 is marked complete only after all checks pass.

## Out of scope

- Calibrating Damiao temperature bytes, assigning temperature sentinels, or
  claiming physical motor-temperature evidence without hardware.
- Adding motor commands, leases/enforcement, `motion-rt` threads, systemd units,
  MCAP recording, or target-board timing qualification.
- Changing driver lifecycle, CAN transport, or the existing feedback decoder.

## Traceability

- Parent: `.trellis/tasks/07-29-rust-robot-framework/implement.md` Phase D2.
- Contracts: `.trellis/spec/backend/robot-contracts.md`,
  `.trellis/spec/backend/error-handling.md`, and
  `.trellis/spec/backend/quality-guidelines.md`.
