# Rust Directory Structure

## Overview

Production code follows compile-time dependency classes, while executables are vertical composition roots. Runtime co-location never permits a source dependency that skips the middle layer.

## Directory Layout

```text
crates/lower/         # transports, device protocols and device lifecycle
crates/middle/        # semantic messages, ports, codecs, Protobuf and Zenoh adapters
crates/upper/         # robot algorithms, control, FSM and mission runtime
crates/integration/   # deployment inputs and non-core adapters
apps/                 # allowlisted wiring-only executables
tools/                # build/architecture/contract checks
proto/robot/v1/       # only source of external wire schemas
config/robots/        # minimal robot topology YAML
platform/             # board/kernel/deployment qualification (later tasks)
```

The authoritative member-to-class map and allowed edges are in `architecture.toml`. Every new workspace member must be added there in the same change.

## Dependency Classes

| Source | Allowed internal targets |
|---|---|
| lower | lower |
| middle | middle, lower |
| upper | upper, middle |
| integration | integration, middle |
| app | allowlisted composition roots may construct any layer |
| tool | tool only |
| test-support | dev dependency only; absent from production binaries |

`config-model` is deliberately outside the production layers. Only a composition root parses the whole YAML, then converts it into narrow layer-owned startup settings. A layer must never import the deployment AST.

## Naming and Ownership

- Crate and directory names use kebab-case.
- Generated Rust lives in Cargo `OUT_DIR`; never commit or hand-edit it.
- A wire field is decoded/validated only in `contract-codec`.
- A morphology owns its required logical roles in one compiled profile.
- Shared constants live with the contract owner, not in application crates.

## Composition Roots

Applications may depend on multiple layers only for object construction, thread/process setup and startup conversion. Robot algorithms, protocol parsing and safety decisions belong to their owning layer. `architecture-check` rejects unclassified or non-allowlisted roots.
