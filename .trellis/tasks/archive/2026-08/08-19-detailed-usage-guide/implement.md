# Detailed Usage Guide Implementation Plan

## 1. Source Inventory

- [x] Verify every quick-start command against the relevant CLI source, script
      help, or existing test.
- [x] Record the supported/unsupported runtime boundaries from README, app
      sources and platform runbooks.
- [x] Confirm all outbound runbook paths and examples before linking.

## 2. English Guide

- [x] Add `docs/usage-guide.md` with the shared section structure.
- [x] Include host-safe setup, configuration, checks, simulation and
      observability commands with their exact scope.
- [x] Describe target handoff and physical exclusions without duplicating or
      weakening the platform runbooks.

## 3. Simplified Chinese Guide

- [x] Add `docs/usage-guide.zh-CN.md` with the same workflow, warnings,
      commands, paths and links as the English guide.
- [x] Translate prose for an operator/developer audience while preserving all
      technical identifiers exactly.
- [x] Add bidirectional language links.

## 4. Entry Point and Review

- [x] Add a compact bilingual link from `README.md`.
- [x] Compare headings, code fences, paths and Markdown links between the two
      documents.
- [x] Run Markdown whitespace checks, focused command/help checks, applicable
      host documentation tests and `git diff --check`.
- [x] Review every safety sentence against the source-owned platform and
      runtime contracts; no command may imply motor operation or target
      qualification.

## Validation

```bash
cargo run --locked -p robotctl -- config validate config/robots/example.yaml
cargo run --locked -p btctl -- example
tools/run-arm-simulation.sh verify
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/tests/host-test.sh
git diff --check
```

Run only the host-safe commands above during this task. Target qualification,
CAN setup, affinity application and physical safety tests remain out of scope.

## Completion Record

- Full host quality gate: `tools/ci.sh` passed.
- Documentation checks: `git diff --check`, relative-link validation, and a
  paired fenced-code-block diff passed.
- Spec update decision: no `.trellis/spec/` update is required. The change
  introduces no executable contract, workflow convention, or reusable
  engineering pattern; it indexes existing contracts without changing them.

## Rollback

Revert the README link and both guide files as one documentation unit. Do not
modify generated evidence, candidate profiles or target configuration.
