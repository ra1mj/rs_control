# RTen 0.24.0 dependency admission

## Decision

Admit the exact-pinned pure-Rust RTen 0.24.0 family for CPU-only ONNX policy
inference:

```toml
rten = { version = "=0.24.0", default-features = false, features = ["onnx_format"] }
rten-onnx = "=0.24.0"
rten-tensor = "=0.24.0"
```

`rten-onnx` is a direct dependency because the runtime inspects the ONNX
protobuf before RTen readiness for external-data, static-shape, f32 and
non-finite-constant rejection. `rten-tensor` is a direct dependency for exact
runtime output shape inspection.

## Reviewed properties

- Registry metadata reports `MIT OR Apache-2.0`.
- RTen 0.24.0 declares Rust 1.89; the workspace pins Rust 1.97.
- The selected feature surface enables `onnx_format` only. It excludes RTen's
  `rten_format`, `mmap`, `fft`, `random`, `all-ops` and `wasm_api` features.
- Production calls safe bounded-byte `Model::load`; it never calls unsafe
  `Model::load_mmap`.
- Inference passes a dedicated `ThreadPool::with_num_threads` through
  `RunOptions`; it does not use RTen's process-global pool.
- The dependency graph contains one 0.24.0 version for every RTen-family
  crate. Existing workspace duplicates (including the intentional nalgebra
  0.33/0.35 adapter boundary) are unrelated to this admission.
- `cargo deny check` reports advisories, bans, licenses and sources as OK.

## Runtime boundary

No native ONNX Runtime library, GPU provider, Python interpreter, online
trainer, KINETIC source, hardware driver or CAN protocol dependency enters the
RL crate. Fixture generation is a development-only Rust example; CI consumes
checked-in ONNX bytes.

## Operator surface

The checked deterministic fixtures use only `Identity`, `Mul` and `Add`.
Arbitrary unsupported production ONNX operators fail during bounded startup
loading; there is no automatic backend fallback.
