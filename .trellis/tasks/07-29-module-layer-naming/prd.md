# Architecture module naming migration

## Goal

Replace the three existing architecture-layer names everywhere in the repository
with the user-selected module names, while preserving every runtime behavior,
ownership rule and dependency edge.

## Background

The repository currently uses `lower`, `middle` and `upper` in directory names,
architecture-policy class names, one crate name, fixtures, source documentation,
Trellis specifications and task artifacts. The requested change is strictly a
naming migration:

| Existing name | Canonical name |
| --- | --- |
| `lower` | `hardware_module` |
| `middle` | `protocol_module` |
| `upper` | `application_module` |

The mapping does not move responsibilities. Hardware transports, device wire
protocols and device lifecycles remain in the first layer; process/node
communication remains in the second layer; algorithms, control and FSM logic
remain in the third layer.

## Requirements

- **NM-1 — Canonical directories:** move production crates from
  `crates/lower`, `crates/middle` and `crates/upper` to
  `crates/hardware_module`, `crates/protocol_module` and
  `crates/application_module` respectively.
- **NM-2 — Canonical architecture classes:** rename the corresponding
  `architecture.toml` class keys, member assignments, path prefixes and allowed
  dependency values. Preserve the graph exactly:
  `application_module -> protocol_module -> hardware_module`; the
  `hardware_module` cannot depend on either higher layer, and
  `application_module` cannot directly depend on `hardware_module`.
- **NM-3 — Canonical core crate:** rename `lower-core` to
  `hardware-module-core` and its Rust import identifier to
  `hardware_module_core`. Crates whose existing semantic names contain none of
  the retired layer labels keep their package names.
- **NM-4 — Complete repository vocabulary:** update architecture fixtures,
  fixture filenames, source-level documentation, README content, platform
  documentation, Trellis specifications, active tasks, archived task artifacts
  and developer journal entries that use the retired words as architecture
  labels. Generic English uses such as “lower bitrate” or API calls such as
  `to_lowercase` are not architecture names and must not be rewritten.
- **NM-5 — Stable external contracts:** do not change Protobuf package names,
  field numbers, robot YAML schema, CLI behavior, platform behavior, public Rust
  type behavior or dependency ownership.
- **NM-6 — Atomic migration:** do not add compatibility aliases or retain a
  mixed old/new directory layout. The repository is pre-release and all
  in-repository consumers migrate in one verified change.
- **NM-7 — Future enforcement:** architecture-check fixtures and assertions use
  only the new vocabulary and continue proving every forbidden cross-layer
  edge.

## Acceptance Criteria

- [x] `cargo metadata --no-deps` resolves every workspace member from the three
      canonical module directories and contains `hardware-module-core`, not
      `lower-core`.
- [x] `cargo run -p architecture-check -- check` accepts the real dependency
      graph using `hardware_module`, `protocol_module` and
      `application_module` classes.
- [x] Negative fixtures reject
      `application_module -> hardware_module`,
      `hardware_module -> protocol_module`,
      `protocol_module -> application_module` and production dependencies on
      `test-support`.
- [x] Outside this task's explicit before/after migration record, a repository
      scan finds no remaining architecture-label use of the retired `lower`,
      `middle` or `upper` names and no old three-layer paths, class values,
      crate name or fixture filenames.
- [x] Robot configuration and Protobuf compatibility tests remain unchanged and
      pass.
- [x] `tools/ci.sh` passes with no behavior change outside naming and path
      resolution.

## Out of Scope

- Reassigning any subsystem to another layer.
- Adding SocketCAN, Damiao, Zenoh, shared-memory, FSM or algorithm behavior.
- Renaming semantically named crates that do not contain a retired layer label.
- Changing external message or deployment configuration schemas.
