# Backend Development Guidelines

> Best practices for backend development in this project.

---

## Overview

This directory defines the executable Rust architecture, robot contract, error, and quality rules for the framework. The active guides are normative for code under `crates/`, `apps/`, `tools/`, `proto/`, and `config/robots/`.

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Rust layer and composition-root layout | Active |
| [Robot Contracts](./robot-contracts.md) | Layer, YAML and Protobuf executable contracts | Active |
| [Error Handling](./error-handling.md) | Typed boundary and safety-error conventions | Active |
| [Quality Guidelines](./quality-guidelines.md) | Required checks and forbidden patterns | Active |
| [Logging Guidelines](./logging-guidelines.md) | Structured logging and RT drain rules | Planned |
| [Database Guidelines](./database-guidelines.md) | ORM patterns, queries, migrations | Not applicable yet |

---

## Pre-Development Checklist

Before changing Rust code or a cross-layer contract:

1. Read `directory-structure.md` and preserve the dependency class of every changed crate.
2. Read `robot-contracts.md` when changing YAML, Protobuf, device traits, command authority or a composition root.
3. Read `error-handling.md` when adding a fallible boundary.
4. Read `quality-guidelines.md` and run the applicable commands before review.

## Quality Check

Run `tools/ci.sh` for a full host-side gate. Hardware and target-board gates are additive and cannot be replaced by host tests.

---

**Language**: All documentation should be written in **English**.
