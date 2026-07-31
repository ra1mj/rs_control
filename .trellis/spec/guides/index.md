# Shared Design Guides

Use these short guides before changing a contract or introducing a helper that
spans crates. They complement the executable rules in `../backend/`.

---

## Available Guides

| Guide | Purpose | When to Use |
|-------|---------|-------------|
| [Code Reuse](./code-reuse-thinking-guide.md) | Find the existing contract owner before adding helpers or constants | Repeated logic, constants, codecs, or backend behavior |
| [Cross-Layer Data Flow](./cross-layer-thinking-guide.md) | Trace validation, ownership, timing, and failure across robot layers | YAML, Protobuf, middleware, device, command, or health changes |

---

## Quick Reference: Thinking Triggers

### Read the cross-layer guide when

- [ ] a value crosses YAML, semantic Rust, Protobuf, local middleware, Zenoh, or
      a hardware boundary;
- [ ] a timestamp, freshness rule, sequence, lease, fault, or quality field
      changes;
- [ ] an application starts importing a generated DTO, transport type, or
      concrete middleware backend;
- [ ] a new failure path needs a deterministic safe action.

→ Read [Cross-Layer Thinking Guide](./cross-layer-thinking-guide.md)

### Read the reuse guide when

- [ ] adding a constant, validation rule, codec, key, counter, or lifecycle
      transition;
- [ ] implementing another `middleware-api` backend;
- [ ] two crates interpret the same raw field or error independently;
- [ ] adding a utility whose ownership layer is unclear.

→ Read [Code Reuse Thinking Guide](./code-reuse-thinking-guide.md)

### Review discipline

- Verify every finding against the owning type, call path, and tests.
- Distinguish host evidence from physical target qualification.
- A test must fail when the behavior it claims to prove is removed or broken.
- Preserve deliberate unavailable/fail-closed states; do not “fix” them into
  optimistic defaults.
