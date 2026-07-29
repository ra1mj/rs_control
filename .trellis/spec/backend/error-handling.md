# Rust Error Handling

## Error Ownership

- Libraries expose typed `thiserror` enums owned by their boundary.
- Concrete transport/protocol faults remain the source of `DeviceError<Fault>`.
- Protobuf syntax and semantic failures converge in `CodecError` at `contract-codec`.
- YAML syntax and topology failures converge in `ConfigError` at `config-model`.
- CLI binaries may erase errors to `Box<dyn Error>` only at `main`/`run`; libraries do not.

## Required Patterns

```rust
#[derive(Debug, thiserror::Error)]
pub enum DriverFault {
    #[error("feedback checksum mismatch")]
    Checksum,
}

let error = DeviceError::Protocol(DriverFault::Checksum);
```

Preserve the error source with `#[source]`/`#[from]`. Validation constructors return a valid semantic object or a typed error; do not construct a DTO and hope every consumer remembers to validate it.

## Safety Rules

- Missing/stale feedback is never converted to numeric zero.
- Unknown/`UNSPECIFIED` wire enum values are errors at the middle boundary.
- Safety logic consumes numeric/typed fault codes, never formatted log text.
- Expired commands and leases are rejected, not clamped or refreshed implicitly.
- Directory and schema discovery propagates entry errors; `filter_map(Result::ok)` is forbidden at contract/build boundaries.
- Do not panic for external input. `expect` is limited to locally proven invariants such as writing to `String` or private ASCII storage.

## Common Mistakes

- A `thiserror` field literally named `source` is automatically treated as an error source. Use a different name for ordinary data such as `source_package`.
- Serde `rename_all = "snake_case"` does not insert an underscore before digits (`Mecanum4` becomes `mecanum4`). Use explicit `#[serde(rename = "mecanum_4")]` spellings for versioned YAML values.
