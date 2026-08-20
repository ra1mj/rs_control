# Write detailed usage guide

## Goal

Create one detailed, safety-accurate operational guide that lets developers
build, validate, configure, run host-safe demonstrations, inspect recorded
data, and prepare an Orange Pi target without turning host instructions into
hardware, realtime, or motor-enablement claims.

## Confirmed Facts

- The repository is a Rust workspace with enforced
  `application_module -> protocol_module -> hardware_module` boundaries.
- `tools/ci.sh` is the complete host gate. It includes platform fixture tests,
  Rust formatting/lint/tests, architecture, Protobuf compatibility, dependency,
  and robot-YAML checks. It must not be represented as a target qualification.
- `robotctl` validates and prints normalized robot YAML. YAML carries topology
  only; it must not contain CAN protocol details, tuning, device paths, Zenoh
  deployment, or realtime scheduling.
- The arm simulation, behavior-tree/navigation and `motion-rt` runtime offer
  host-verifiable, motor-disabled behavior. The runtime's executable currently
  has no production CLI workflow, so the guide must not invent one.
- Orange Pi scripts separate safe inspection/fixture commands from explicit
  privileged target operations. The checked-in profile is a candidate only and
  production preparation requires immutable target qualification evidence.
- The target work remains blocked on real Orange Pi/RK3588, CAN, motor and
  safety-fixture evidence. Existing runbooks are authoritative for detailed
  platform build, bring-up, qualification and rollback procedures.
- Existing project documentation and backend guidance are written in English.
- The user requires both Simplified Chinese and English documentation.

## Requirements

- R1: Publish a bilingual entry-point guide under `docs/` and link both
  language versions from the root README without duplicating detailed platform
  or subsystem runbooks.
- R2: Cover repository orientation, prerequisites, setup, configured host
  checks, robot-YAML validation, simulation, behavior-tree/navigation
  observability, host motion/telemetry boundaries, and complete host CI.
- R3: Give only commands that exist in the repository and state their expected
  safety scope, inputs, output behavior, and failure/rollback path where
  relevant.
- R4: Separate three execution classes visibly: read-only/host-safe,
  target-only but motor-disarmed, and physical-motor/safety-fixture gates.
- R5: Link rather than restate the Orange Pi kernel build, board bring-up,
  realtime/middleware qualification, rollback, navigation and arm-calibration
  runbooks. Preserve their warnings about candidate profiles, absent evidence,
  immutable manifests and no automatic fallback.
- R6: Explain what the system does not yet provide, including the absence of a
  hardware qualification and of a production `motion-rt` CLI operation.
- R7: Keep terminology, paths, Cargo package names, configuration examples and
  safety claims consistent with source, tests and existing docs.
- R8: Provide equivalent operational content in Simplified Chinese and English,
  with a visible cross-link and stable technical identifiers in both versions.

## Acceptance Criteria

- [x] A developer can reach every supported host workflow from a root README
      link and run the documented commands from an arbitrary working directory
      or understands the required repository-root context.
- [x] Every executable command in the guide is verified against the checked-in
      CLI/help text, script contract or test.
- [x] The guide distinguishes host evidence from target qualification and never
      promises CAN transmission, motor enablement, physical stopping, or 1 kHz
      performance from host tests.
- [x] Target sections state the required operator prerequisites, explicit
      disarmed boundary, evidence/manifest handling and links to rollback.
- [x] Existing topical and platform runbooks remain the detailed source of
      truth and are linked correctly.
- [x] Markdown links and fenced command examples are checked, and the relevant
      focused documentation/CLI checks pass.
- [x] Both language versions cover the same supported workflows and cross-link
      to one another without translating commands, paths or identifiers.

## Scope Boundaries

- Do not add a new runtime CLI, systemd unit, platform profile, target
  qualification result, hardware operation or motor behavior.
- Do not translate or duplicate all existing runbooks; the new bilingual guide
  is an indexed operational path.
- Do not change backend contracts, schemas, robot YAML or safety thresholds.

## Documentation Decision

- Publish complete paired documents: `docs/usage-guide.md` in English and
  `docs/usage-guide.zh-CN.md` in Simplified Chinese. Each file links to the
  other at the top.
- English is the technical maintenance baseline because existing source-facing
  documentation and project guidelines are English. The Chinese document has
  equivalent operational scope; commands, paths, Cargo package names, flags,
  environment names, manifest fields and evidence labels remain untranslated.

## Notes

- This is a documentation-only task, but its cross-cutting operational and
  safety scope requires `design.md` and `implement.md` before implementation.

## Completion Evidence

- `tools/ci.sh` passed as the full host-side quality gate.
- Relative Markdown links in `README.md` and both guides resolve from their
  containing files; the two guide files have identical fenced code-block
  contents.
- The review rechecked the documented candidate-profile, target-evidence,
  motor-disabled, and no-production-CLI claims against the source-owned
  platform and runtime contracts.
