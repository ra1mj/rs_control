# Rust Directory Structure

## Scenario: Add or migrate a production architecture class

### 1. Scope / Trigger

Use this contract whenever work adds, renames or moves a production crate,
changes an `architecture.toml` class/member/path, or changes a composition-root
dependency. Runtime co-location never permits a source dependency that skips
the `protocol_module` layer.

### 2. Signatures

The executable architecture checks are:

```text
cargo metadata --no-deps --format-version 1
architecture-check check [--workspace-root <path>] [--policy <path>]
```

Every workspace member is declared in the root `Cargo.toml`; its class,
path-prefix ownership and allowed internal dependencies are declared in
`architecture.toml`.

### 3. Contracts

```text
crates/hardware_module/    # transports, device protocols and device lifecycle
crates/protocol_module/    # semantic messages, ports, codecs, Protobuf and Zenoh adapters
crates/application_module/ # robot algorithms, control, FSM and mission runtime
crates/integration/        # deployment inputs and non-core adapters
apps/                      # allowlisted wiring-only executables
tools/                     # build/architecture/contract checks
proto/robot/v1/            # only source of external wire schemas
config/robots/             # minimal robot topology YAML
platform/                  # board/kernel/deployment qualification
```

| Source | Allowed internal targets |
|---|---|
| `hardware_module` | `hardware_module` |
| `protocol_module` | `protocol_module`, `hardware_module` |
| `application_module` | `application_module`, `protocol_module` |
| `integration` | `integration`, `protocol_module` |
| `app` | allowlisted composition roots may construct any layer |
| `tool` | `tool` only |
| `test-support` | dev dependency only; absent from production binaries |

Crate and directory names use kebab-case; architecture class identifiers use
the exact underscore spellings above. A class/path migration is atomic across
the filesystem, Cargo workspace paths, lockfile package identity,
`architecture.toml`, negative fixtures, source documentation and active
project specifications. Do not add compatibility aliases or retain a mixed
old/new layout.

`config-model` remains outside the production layers. Only a composition root
parses the whole YAML and converts it into narrow layer-owned startup settings.
Applications may construct multiple layers, set up processes/threads and
perform startup conversion, but robot algorithms, protocol parsing and safety
decisions remain with their owning layer.

Generated Rust lives in Cargo `OUT_DIR`; never commit or hand-edit it. A wire
field is decoded/validated only in `contract-codec`. Shared constants live with
the contract owner, not in application crates.

### 4. Validation & Error Matrix

| Condition | Required result |
|---|---|
| Workspace member is absent from `architecture.toml` | `architecture-check` reports an unclassified package |
| Manifest path does not match its class prefix | `architecture-check` reports a path/class mismatch |
| Dependency is outside the source class allowlist | `architecture-check` reports `ForbiddenDependency` |
| Production crate normally depends on `test-support` | `architecture-check` reports a production test-support dependency |
| Composition root is not explicitly allowlisted | reject it as an ordinary class rather than granting broad access |
| Rename changes external Protobuf/YAML behavior or dependency reachability | fail review and restore the pre-migration contract |
| Vocabulary sweep encounters ordinary comparison/bounds text | preserve it; never rewrite words such as “lower bitrate” or “upper bound” as architecture identifiers |

### 5. Good / Base / Bad Cases

- Good: move all members of one class, update Cargo and policy paths, rename
  every corresponding negative fixture, and preserve the same graph edges.
- Base: add a new crate under an existing class and classify it in the same
  change without adding an internal dependency.
- Bad: keep aliases for both directory names, update only Cargo paths, or
  globally replace ordinary English words while migrating class identifiers.

### 6. Tests Required

- `cargo metadata --no-deps --format-version 1` resolves every member from its
  canonical path and reports the intended package/library names.
- `cargo run -p architecture-check -- check` accepts the real graph.
- Fixture tests reject `application_module -> hardware_module`,
  `hardware_module -> protocol_module`,
  `protocol_module -> application_module`, and production use of
  `test-support`.
- `tools/ci.sh` proves formatting, Clippy, workspace tests, schema checks and
  robot YAML behavior remain unchanged.
- A targeted repository scan classifies every remaining retired identifier;
  migration documentation and ordinary comparison/bounds language are the only
  acceptable exceptions.

### 7. Wrong vs Correct

#### Wrong

```toml
# Cargo moved, but the policy still owns the old path and an alias keeps both
# names alive.
members = ["crates/new_name/device"]
[classes.old_name]
path_prefixes = ["crates/old_name/"]
```

#### Correct

```toml
# Cargo.toml
[workspace]
members = ["crates/hardware_module/device"]
```

```toml
# architecture.toml
[classes.hardware_module]
allowed_dependencies = ["hardware_module"]
path_prefixes = ["crates/hardware_module/"]

[members]
device = "hardware_module"
```
