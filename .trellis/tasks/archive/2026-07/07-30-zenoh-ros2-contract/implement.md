# Zenoh and ROS 2 Protocol Contract — Implementation Plan

## 1. Preconditions

- [x] Confirm the task remains backend-neutral: no ROS 2, DDS or Zenoh runtime
      dependencies and no changes to `config/robots/*.yaml`.
- [x] Load the backend protocol, directory, error and quality specifications
      before editing.
- [x] Record the current Protobuf golden baseline and run the existing protocol
      tests before amendments.

## 2. Phase A — Additive canonical protocol

- [x] Add acquisition timestamp/domain and 3D primitives to `common.proto`
      using only unused field numbers.
- [x] Add time-aligned joint aggregation to `device.proto`.
- [x] Add odometry and static/dynamic transform contracts in a new
      `navigation.proto`, then include it in Rust generation, descriptor output
      and cross-language generation.
- [x] Extend `control.proto` with typed action goal/feedback/result payloads,
      cancel request/decision and concrete bounded chassis trajectory payloads;
      preserve all existing enum numbers and messages.
- [x] Add bounded health snapshot request/response messages.
- [x] Run Buf format/lint/breaking checks before updating the golden descriptor.

Validation:

```bash
tools/bufw format --diff --exit-code
tools/bufw lint
cargo test -p proto-generated -p proto-compat-check
```

Rollback point: revert only additive schema changes; do not alter or renumber
existing v1 fields to make a compatibility test pass.

## 3. Phase B — Semantic values and codec

- [x] Extend `message-model` with acquisition time, finite 3D values, checked
      unit quaternions, covariance matrices and portable frame IDs.
- [x] Add validated semantic IMU, joint aggregate, odometry, transform, health
      snapshot, command ingress and action lifecycle values.
- [x] Add explicit IMU unavailable/unknown covariance projection state.
- [x] Implement the pure action transition model, UUID/command-ID correlation
      and bounded terminal-result policy validation.
- [x] Extend `contract-codec` with fallible conversions for every Protobuf type
      referenced by the initial mapping profile.
- [x] Cover unknown enums, missing values, NaN/infinity, quaternion tolerance,
      covariance length/symmetry/diagonal, frame and action-order edge cases.

Validation:

```bash
cargo test -p message-model -p contract-codec
```

Rollback point: keep existing public semantic constructors compatible; new
validated types may be removed independently if the manifest crate has not
been released.

## 4. Phase C — ROS mapping registry

- [x] Create `crates/integration/ros2-mapping` and register it in the workspace
      and architecture policy as `integration`.
- [x] Define a strict Serde/TOML schema for profile, deployment, frame aliases,
      TF authorities, routes, QoS, clock/frame and authorization policies.
- [x] Validate required nonzero bounds, direction/interaction/QoS
      compatibility, command authority, lossy mapping declarations, unique
      ownership and deterministic lookup.
- [x] Validate native/ros2dds prefix disjointness, DDS-island path uniqueness,
      route loop identities, TF child ownership and cycle freedom.
- [x] Verify Protobuf descriptor and ROS interface-source SHA-256 identities;
      reject path traversal and unregistered Protobuf types.
- [x] Expose a stable manifest fingerprint and queryable route descriptors.
- [x] Add focused invalid-manifest fixtures for every rejection class.

Validation:

```bash
cargo test -p ros2-mapping
cargo run -p architecture-check -- --workspace-root .
```

Rollback point: the crate has no runtime side effects; remove its workspace and
architecture entries together if its schema must be redesigned.

## 5. Phase D — Checked-in Humble profile

- [x] Add reviewed Humble interface source snapshots for every route and a
      concrete custom health service/action.
- [x] Add `config/protocol/ros2-humble/routes.toml` with all first-profile
      routes, exact bounds, hashes, explicit QoS and pinned compatibility data.
- [x] Include separate `/tf` and `/tf_static` policies, disabled unstamped Twist
      ingress, local lease/deadline requirements and bounded action result
      retention.
- [x] Add a golden-profile test that loads the repository file, verifies every
      hash/type and compares the deterministic fingerprint.
- [x] Assert no protocol keys/settings entered the minimal robot YAML schema.

Validation:

```bash
cargo test -p ros2-mapping --test profile
cargo test -p config-model
```

## 6. Phase E — Compatibility and quality gates

- [x] Regenerate and review the Protobuf golden descriptor only after the
      breaking check passes against the previous baseline.
- [x] Run formatting, Clippy, workspace tests, architecture policy, Protobuf
      cross-language generation, Buf lint/breaking, dependency and YAML gates.
- [x] Review the diff for accidental ROS/Zenoh runtime dependencies, unsafe
      code, unbounded storage, generated artifacts and robot-YAML drift.
- [x] Update executable Trellis specifications with durable new protocol/route
      conventions learned during implementation.

Full validation:

```bash
./tools/ci.sh
```

## 7. Completion and handoff

- [x] Mark this checklist and PRD acceptance criteria with evidence-backed
      completion only; runtime-only criteria remain explicitly delegated.
- [ ] Commit the implementation and task/spec records in reviewable commits.
- [ ] Archive the completed task, record the developer journal and push the
      resulting branch.
- [x] Leave a named follow-up boundary for the isolated ROS 2 runtime adapter
      and target-board/network qualification.
