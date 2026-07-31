# Motor temperature boundary evidence

## Repository evidence

- `crates/hardware_module/motor-damiao/src/codec.rs` names the two feedback
  fields `mos_temperature_raw` and `winding_temperature_raw`.
- The same type documents that signedness and sentinels remain unqualified.
- `proto/robot/v1/device.proto` currently names fields 6 and 7 as Celsius, but
  no semantic `MotorState`, codec mapping, or driver adapter uses them.
- `architecture.toml` permits `protocol_module` crates to depend on
  `hardware_module` crates, so the one-time Damiao mapping belongs in
  `contract-codec` without a reverse dependency.

## Decision

Preserve the existing Celsius field numbers for compatibility and add an
explicit new temperature observation representation. Damiao emits the raw
variant until a separate hardware-backed calibration task qualifies a Celsius
conversion. Do not infer physical units from an eight-bit storage type.
