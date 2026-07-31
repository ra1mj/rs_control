# Mission DSL and deterministic runtime

## Goal

Implement the software-only G4 typed mission DSL, immutable validated IR, deterministic event reducer, action correlation and cancellation scopes.

## Requirements

- Add bounded application-owned node types for Action, Sequence, Selector,
  ParallelAll, ParallelAny, Timeout, Retry, Loop, Guard, CancelScope and named
  Subflow calls.
- Build an immutable IR only after validating all node references, reachability,
  acyclicity, registered action/guard identities and fixed capacity limits.
- Reject duplicate IDs/names/resources, empty composites, missing action or
  wrapper timeouts, unbounded/zero retry or loop policy, and exclusive-resource
  overlap among parallel branches.
- Implement one serialized, deterministic activation reducer. It may emit
  effects but performs no I/O, clock reads, random-ID generation or unbounded
  queueing.
- Bind externally generated action `command_id` values through recorded reducer
  events, then require exact command correlation for acknowledgement,
  cancellation decision and terminal result.
- Keep cancel acceptance distinct from terminal cancellation. Scope/global
  cancellation and timeout request cancellation for all active descendants.
- Release runtime activation state only after terminal action evidence; never
  infer completion from publishing a cancel request.
- Provide a representative project containing sequence, guard branch,
  parallel, timeout, retry and cancel scope nodes.
- Keep hardware and Orange Pi qualification out of scope.

## Acceptance Criteria

- [x] A validated IR cannot be mutated through its public API.
- [x] Every invalid graph/policy class above has a typed rejection test.
- [x] Parallel resource conflicts are found transitively across branches.
- [x] Runtime replay over the same ordered events produces identical effects
  and terminal outcome.
- [x] Wrong/duplicate/stale activation or command correlations are rejected
  without advancing mission state.
- [x] Retry/loop bounds, selector/guard routing and parallel completion semantics
  are tested.
- [x] Cancel scopes target only descendants in that scope; global cancellation
  targets all active actions.
- [x] Timeout and cancellation wait for correlated terminal action evidence.
- [x] Representative project validates and completes under a scripted trace.
- [x] Warnings-denied Clippy, architecture policy and `tools/ci.sh` pass.

## Notes

All runtime queues, child lists, resource sets, active slots and effect batches
have compile-time maximum capacities.
