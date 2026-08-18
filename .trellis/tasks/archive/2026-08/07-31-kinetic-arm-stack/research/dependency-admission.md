# Arm-stack dependency admission

Reviewed on 2026-07-31 for the host-only software implementation.

| Crate | Version | License | Rust | Decision |
|---|---:|---|---:|---|
| `nalgebra` | 0.35.0 | Apache-2.0 | 1.89 | Accept exact pin for native f64 kinematics and bounded non-RT matrices |
| `urdf-rs` | 0.9.0 | Apache-2.0 | not declared | Reject: mandatory vulnerable `quick-xml 0.36.2` |
| `quick-xml` | 0.41.0 | MIT | 1.79 | Accept exact pin behind a bounded project-owned event parser |
| `rand_chacha` | 0.10.0 | MIT OR Apache-2.0 | 1.85 | Accept exact pin for explicit seeded non-RT sampling |
| `featherstone` | 0.1.0 | Apache-2.0 | 1.70 | Conditionally accept exact pin with default features disabled and a strict f64/f32 adapter |

## Featherstone constraints

`featherstone` stores generalized state and results as `f32`, uses
`nalgebra 0.33`, and exposes unvalidated dynamic vectors. Its `add_body`
constructor also asserts parent ordering. The project adapter must therefore:

- validate topology, dimensions, finiteness, inertia, and parent order before
  constructing the backend;
- reject f64 values that cannot be represented as finite f32;
- convert every result back to f64 and revalidate it;
- expose only project-owned errors and bounded matrices/vectors;
- never pass external or policy input directly to a backend method;
- keep the dependency out of realtime, protocol, and hardware crates.

The Featherstone default `urdf` feature is disabled to avoid a second URDF
parser and to ensure only `arm-model` owns URDF validation. Collision/contact
modules are not used or qualified.

The source review found no `unsafe` block. Production `add_body` contains an
assertion, so construction is legal only after the project compiler proves the
asserted invariant. If Cargo dependency policy or adapter golden tests fail,
the backend is removed and dynamics remains typed unavailable; no dependency
policy exception is permitted.

## URDF parser decision

`cargo deny check` rejected `urdf-rs 0.9.0` because it requires
`quick-xml 0.36.2`, affected by `RUSTSEC-2026-0194` (quadratic duplicate
attribute checking) and `RUSTSEC-2026-0195` (unbounded namespace allocation).
No advisory exception is added.

The replacement is exact `quick-xml 0.41.0`, which contains both remediations.
A project-owned event parser rejects DTDs, bounds source bytes, nesting,
events, attributes, links and joints, and retains only kinematic/dynamics URDF
fields. Its private parse tree is then compiled with portable names, topology
checks, finite SI checks, active-joint limits, explicit acceleration limits,
mimic validation, and physical-inertia validation.

The replacement graph passed `cargo deny check` on 2026-07-31: advisories,
bans, licenses and sources were all accepted. The workspace's pre-existing
duplicate-version findings remain warnings under the checked-in policy.
