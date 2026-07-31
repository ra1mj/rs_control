# Rust Development Guidelines

---

## Overview

This directory defines the executable Rust architecture, robot contract,
diagnostic, error, and quality rules for the framework. The active guides are
normative for code under `crates/`, `apps/`, `tools/`, `proto/`, and
`config/robots/`.

---

## Guidelines Index

| Guide | Description | Status |
|-------|-------------|--------|
| [Directory Structure](./directory-structure.md) | Rust layer and composition-root layout | Active |
| [Robot Contracts](./robot-contracts.md) | Layer, YAML, Protobuf and middleware delivery contracts | Active |
| [Error Handling](./error-handling.md) | Typed boundary and safety-error conventions | Active |
| [Diagnostics and CLI Output](./logging-guidelines.md) | Typed runtime diagnostics and deterministic process output | Active |
| [Lifecycle FSM](./fsm-guidelines.md) | Exhaustive lifecycle reducer, restart and graph contracts | Active |
| [Mission IR and Runtime](./mission-guidelines.md) | Bounded mission validation, action correlation and cancellation | Active |
| [Chassis Kinematics](./chassis-kinematics.md) | Body-frame SI models, morphology maps and constraint evidence | Active |
| [Chassis Estimation and Control](./chassis-estimation-control.md) | Local-time fusion, degradation and SE(2) tracking | Active |
| [Quality Guidelines](./quality-guidelines.md) | Required checks and forbidden patterns | Active |
| [Realtime Platform](./realtime-platform.md) | Kernel, affinity, CAN and qualification contracts | Active |

---

## Pre-Development Checklist

Before changing Rust code or a cross-layer contract:

1. Read `directory-structure.md` and preserve the dependency class of every changed crate.
2. Read `robot-contracts.md` when changing YAML, Protobuf, middleware ports/routes, device traits, command authority or a composition root.
3. Read `error-handling.md` when adding a fallible boundary.
4. Read `logging-guidelines.md` when changing health, faults, counters, CLI
   output, evidence records, or any code that might run on an RT path.
5. Read `quality-guidelines.md` and run the applicable commands before review.
6. Read `realtime-platform.md` before changing `platform/`, RT placement, privileged preparation or hardware evidence.
7. Read `fsm-guidelines.md` before changing lifecycle states, events,
   transitions, restart semantics or mission-cancellation boundaries.
8. Read `mission-guidelines.md` before changing mission nodes, graph
   validation, action correlation, retry/timeout or cancellation behavior.
9. Read `chassis-kinematics.md` before changing chassis geometry, wheel/module
   order, forward/inverse maps, saturation or steering optimization.
10. Read `chassis-estimation-control.md` before changing chassis sample
    alignment, estimator quality, body/heading control or SE(2) trajectories.

## Quality Check

Run `tools/ci.sh` for a full host-side gate. Hardware and target-board gates are additive and cannot be replaced by host tests.

---

**Language**: All documentation should be written in **English**.
