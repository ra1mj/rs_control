# Bootstrap project development guidelines

## Goal

Make `.trellis/spec/` describe the current `rs_control` Rust workspace so
future implementation and review sessions load repository-specific rules
instead of generic full-stack templates.

## Scope

- Inspect the Cargo workspace, architecture policy, representative libraries,
  command-line tools, tests, protocol schemas, and platform evidence tooling.
- Keep the existing source-backed Rust architecture, robot contract, error,
  realtime, and quality guides.
- Add the missing current diagnostic and CLI-output contract.
- Replace generic thinking guides with robot-framework-specific boundary and
  reuse guidance.
- Remove database and frontend templates because the repository contains no
  database, web frontend, TypeScript, component, hook, or UI-state layer.

## Requirements

- Important rules cite real source/test paths or an executable project policy.
- The backend index lists every active guide and its pre-development trigger.
- No placeholder text, empty template heading, or copied Trellis product
  example remains.
- The specification must not invent a logging runtime, database, frontend, or
  other architecture that is absent from the repository.
- Product source code is out of scope.

## Acceptance criteria

- [x] Rust/backend guidelines cover directory ownership, cross-layer robot
      contracts, typed errors, diagnostics/CLI output, quality, and realtime
      platform work.
- [x] Non-applicable database and frontend templates are removed.
- [x] Shared guides use real `rs_control` ownership boundaries and file
      examples.
- [x] A repository scan finds no template placeholder text in `.trellis/spec/`.
- [x] Trellis reports only the backend spec layer.
- [x] Task and full host validation pass.
