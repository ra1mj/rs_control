# Architecture Module Naming Migration Design

## 1. Design intent

This is a mechanical, atomic rename. The source graph before and after the
change is isomorphic; only layer identifiers, paths and the generic core crate
name change.

```text
application_module algorithms / FSM
                  |
                  v
protocol_module ports / Zenoh / Protobuf
                  |
                  v
hardware_module transports / device protocols
```

## 2. Exact mapping

| Surface | Before | After |
| --- | --- | --- |
| first-layer directory | `crates/lower/` | `crates/hardware_module/` |
| second-layer directory | `crates/middle/` | `crates/protocol_module/` |
| third-layer directory | `crates/upper/` | `crates/application_module/` |
| architecture classes | `lower`, `middle`, `upper` | `hardware_module`, `protocol_module`, `application_module` |
| generic core package | `lower-core` | `hardware-module-core` |
| generic core import | `lower_core` | `hardware_module_core` |
| negative fixtures | old layer-pair names | new layer-pair names |

`message-model`, `proto-generated`, `contract-codec` and `control-core` already
describe their responsibility and therefore keep their package names while
moving to the canonical directory.

## 3. Dependency policy preservation

| Class | Allowed production dependencies after migration |
| --- | --- |
| `hardware_module` | `hardware_module` |
| `protocol_module` | `protocol_module`, `hardware_module` |
| `application_module` | `application_module`, `protocol_module` |
| `integration` | `integration`, `protocol_module` |
| `test-support` | test support plus all production classes |
| `app` composition root | app, integration and all production classes |

No source edge is added or removed. Runtime co-location still does not authorize
an `application_module` crate to import a `hardware_module` crate.

## 4. Migration procedure

1. Move the three directory trees in one patch.
2. Rename the generic hardware core directory/package and update Cargo workspace
   paths plus in-repository dependency references.
3. Rename architecture classes, member classifications, fixture files and test
   assertions without weakening any rejection case.
4. Replace architecture-label prose in all project-owned source and knowledge
   artifacts, including archived tasks, while leaving unrelated lexical uses
   intact.
5. Regenerate `Cargo.lock` through Cargo and run metadata, architecture and full
   repository validation.

The migration deliberately has no compatibility alias: aliases would permit the
retired terminology to survive and make future architecture checks ambiguous.

## 5. Compatibility and rollback

The Protobuf and YAML contracts remain byte-for-byte compatible. Rust package
paths are internal pre-release interfaces, so all workspace consumers move
together. The migration is isolated in its own commit and can be reverted as a
unit if a downstream checkout has not yet migrated.

## 6. Verification boundary

The verification pass distinguishes architecture labels from ordinary language
and programming operations. It must reject old paths, class names, the
`lower-core` package and phrases that describe the three retired layers, but it
must not rewrite words such as “lower bitrate”, bounds terminology, or
`to_lowercase`.
