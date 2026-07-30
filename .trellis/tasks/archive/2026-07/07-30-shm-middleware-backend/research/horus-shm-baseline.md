# Horus and shared-memory source baseline

Research date: 2026-07-30

## Pinned source

- Repository: `https://github.com/softmata/horus.git`
- Commit: `fac3cfb8704e275bde9daec6d554a17c59dd8532`
- Commit timestamp: `2026-07-27T00:16:56Z`
- Role: comparison/research input only; not a Cargo dependency and no source is
  copied into this workspace.

The identity is already pinned in `platform/orangepi5plus/sources.lock` and is
the exact commit required by framework R104.

## Reviewed patterns worth reproducing independently

At the pinned commit:

- `horus_core/src/communication/topic/header.rs` uses a versioned header and
  separates producer head and consumer tail onto distinct 64-byte lines.
- `horus_core/src/communication/topic/seqlock.rs` marks a slot odd while being
  written and even when complete, tags the stamp with the logical position,
  checks the stamp on both sides of a byte copy, and bounds a receive call to
  16 attempts instead of spinning forever.
- `horus_core/src/communication/topic/shm_fanout.rs` builds a fixed
  publisher-by-subscriber SPSC matrix, uses reusable endpoint bitmasks, holds a
  per-endpoint exclusive lock file, treats that lock as ownership authority,
  and resets a reclaimed subscriber tail to head so stale backlog is not
  replayed.
- The same file explicitly notes that a real `SIGKILL` integration was not
  exercised in its headless sandbox. This project therefore requires its own
  real subprocess kill/reclaim regression rather than inheriting the claim.

These are protocol ideas. The project implements a narrower SPSC layout behind
its already-established API, with independent code, errors and tests.

## Rejected Horus assumptions

`horus_core/src/communication/pod.rs::is_pod` selects its automatic raw-copy
path when `!needs_drop::<T>()`, size is greater than one, and some single-byte
invalid types are excluded. The source itself documents that enums and
`NonZeroU32` remain counterexamples. That heuristic is insufficient because a
destructor-free type may contain padding, pointers, invalid bit patterns,
interior mutability or unstable layout.

This project instead requires:

1. `bytemuck::Pod`, whose safety contract excludes padding, invalid bit
   patterns, pointers and interior mutability;
2. a stable nonzero 16-byte schema ID and schema version;
3. layout-v1 little-endian/atomic/size/alignment validation at attach; and
4. a private reviewed raw-copy boundary.

Horus message type names, equal size/alignment or a PID are never treated as
schema or ownership authority.

## Crash ownership finding

Rust 1.97 `std::fs::File::try_lock` maps to nonblocking `flock` on Unix. Linux
associates the lock with the open file description, treats separately opened
descriptors independently, and releases the lock after the final descriptor
closes. Process termination closes those descriptors. Therefore a newly
acquired endpoint lock is the abandonment proof and naturally avoids PID reuse.

PID/epoch fields remain useful diagnostics and stale-metadata repair inputs,
but they cannot grant or revoke a live capability.

Primary references:

- Rust `File::try_lock`: <https://doc.rust-lang.org/stable/std/fs/struct.File.html#method.try_lock>
- Linux `flock(2)`: <https://man7.org/linux/man-pages/man2/flock.2.html>

## Mmap and payload dependency review

- `memmap2` `0.9.11`, MIT OR Apache-2.0, Rust 1.65 minimum. File-backed map
  construction is unsafe because other actors could mutate/truncate the file;
  `populate()` and whole-map `lock()` support the startup prefault/lock policy.
- `bytemuck` `1.25.2`, Zlib OR Apache-2.0 OR MIT. Only the `derive` feature is
  required. Its `Pod` derive enforces `repr(C)`/transparent, Pod fields and no
  padding; manual unsafe payload implementations are not needed by project
  examples.
- Rust atomics are lock-free when available; the crate is limited to targets
  with 64-bit atomics and validates layout alignment before atomic views.

Primary references:

- `memmap2::MmapOptions` safety/populate:
  <https://docs.rs/memmap2/0.9.11/memmap2/struct.MmapOptions.html>
- `bytemuck::Pod` safety: <https://docs.rs/bytemuck/1.25.2/bytemuck/trait.Pod.html>
- Rust atomic portability: <https://doc.rust-lang.org/std/sync/atomic/index.html#portability>

Exact versions and minimum features will be committed in the workspace and
rechecked by `cargo deny`; this document is research evidence, not a waiver.
