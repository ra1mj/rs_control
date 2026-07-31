# Motor feedback contract design

## Boundaries

The data flow is:

```text
motor-damiao::DamiaoFeedback
  -> contract-codec::damiao_feedback_to_motor_state
  -> message-model::MotorState
  -> contract-codec::{encode,decode}_motor_state
  -> robot.v1.MotorState
```

`motor-damiao` remains unaware of semantic messages and Protobuf.
`message-model` remains unaware of Damiao. `contract-codec` may depend on the
hardware-module driver because `architecture.toml` explicitly permits
`protocol_module -> hardware_module`.

## Semantic model

Add:

```text
TemperatureObservation::raw(u32)
TemperatureObservation::calibrated_celsius(f64) -> Result<_, SemanticError>

MotorState {
  Header,
  ResourceId,
  position_rad: f64,
  velocity_radps: f64,
  torque_nm: f64,
  mos_temperature: TemperatureObservation,
  rotor_temperature: TemperatureObservation,
  fault_code: u32,
}
```

Fields remain private and are exposed through copy getters.
`TemperatureObservation` is an opaque copyable value rather than a public enum
so callers cannot construct a calibrated NaN without validation. Construction
validates every SI value and every calibrated Celsius value with the existing
`require_finite` helper. A raw value is intentionally not assigned units or a
sentinel interpretation.

## Additive wire evolution

The original `MotorState` fields 6 and 7 remain `double` Celsius fields for
wire/source compatibility. Add a nested `TemperatureObservation` with a
`oneof` containing calibrated Celsius or raw unsigned value, new observations
at fields 9 and 10, and a selector at field 11.

- New encoders set the selector, populate both new observations, and leave the
  legacy doubles at their scalar defaults.
- New decoders with the selector set require both new observations and reject
  non-default legacy doubles so two representations cannot disagree.
- New decoders with the selector clear treat the original fields as calibrated
  Celsius, preserving the old contract exactly.

The old golden descriptor is retained until `buf breaking` and
cross-language generation have passed. It is then replaced with the reviewed
additive descriptor.

## Damiao mapping

`DamiaoMotorStateContext` is opaque and can only be created with a stable
`SourceId`, process `boot_id`, logical `ResourceId`, and nonzero validity
window. Mapping creates a local header from:

- `DamiaoFeedback::observed_monotonic` as source monotonic time;
- the driver's exact sample sequence;
- local clock status with zero uncertainty;
- no Unix/acquisition timestamp or coordinate frame;
- `Valid` quality for `Disabled`/`Enabled`, otherwise `Faulted`.

Position, velocity, and torque widen exactly from `f32` to `f64`.
Temperature bytes become `Raw(u32)`. Fault code is zero for non-fault status.
Known/unknown faults use a fixed nonzero Damiao namespace with the preserved
raw status nibble in the low bits, so even a defensively constructed
`Unknown(0)` cannot contradict `DataQuality::Faulted`.

## Compatibility and rollback

No existing field number or type changes. If any pre-update Buf or
cross-language check fails, restore `device.proto` and do not update the
golden descriptor. The semantic type and mapping can be removed without
changing the hardware decoder or middleware APIs.
