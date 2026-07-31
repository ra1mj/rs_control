# Implementation plan

## 1. Immutable IR

- [x] Add typed action descriptor and bounded node/builder types.
- [x] Add action/guard registries and immutable validated IR.
- [x] Validate references, reachability, cycles, identities and finite policy.
- [x] Validate transitive exclusive-resource conflicts for parallel branches.

## 2. Deterministic runtime

- [x] Add generation-tagged activation slots and bounded effect batches.
- [x] Implement sequence/selector/parallel/guard/subflow propagation.
- [x] Implement timeout/retry/loop policy.
- [x] Implement action dispatch, acknowledgement, result and exact correlation.
- [x] Implement scope/global cancellation and wait for terminal cancellation.

## 3. Verification and example

- [x] Cover every validator rejection and accepted base case.
- [x] Cover composites, retries, loops, deadlines and cancellation races.
- [x] Assert deterministic replay and stale correlation rejection.
- [x] Build and execute one representative automatic project.
- [x] Run formatting, Clippy, package tests, architecture and full CI.

## 4. Knowledge and delivery

- [x] Add mission IR/runtime specification and index entry.
- [x] Update parent G4 evidence.
- [x] Commit implementation separately from task archival and push `main`.
