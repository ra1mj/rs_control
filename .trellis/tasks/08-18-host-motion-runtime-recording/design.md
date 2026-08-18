# Host motion runtime and telemetry recording design

## Boundaries

Synthetic I/O publishes a latest fixed-size state to control. Control validates
an ordered command against LeaseEnforcementCache at an injected monotonic tick
and emits either Apply or SafeStop to a fake sink. Separate bounded SPSC paths
carry diagnostics to a drain worker and telemetry records to a recorder. A full
or absent consumer increments typed/drop evidence and never waits or alters the
control decision.

apps/motion-rt is an app composition root only. Host I/O and fake-sink
implementations are explicit test doubles; there is no default hardware
transport, driver enable path, or raw CAN access.

crates/integration/telemetry-recorder owns MCAP write, sealing, recovery and
replay. It depends only on protocol contracts, the generated descriptor set,
and the admitted mcap dependency. An optional app just parses paths/options
and runs the non-RT service.

## Motion and diagnostic contracts

All worker storage is allocated before threads start. Latest state is Copy and
a missed value is counted. Commands are bounded ordered values. The host
scheduler calls step_at directly; a future target runtime must supply its own
qualified scheduling/affinity/memory-lock boundary.

The fake sink stores fixed-capacity decisions. Apply requires a current accepted
command. Rejection, expiration, stale state, peer loss, shutdown, and worker
loss all produce SafeStop. Controlled join emits the local stop before drain
or recorder teardown.

RtDiagnosticRecord is Copy with typed code, sequence, injected tick, and
saturating numeric fields. RuntimeDiagnostics includes availability,
source/boot identity, counters, health/fault evidence, and safe-stop reason.
It is populated after the safety decision and never read by authority. Only a
non-RT drain may format a record or emit tracing events. Existing
message-model ComponentHealth and Fault stay the semantic owners.

## Recorder and pressure policy

TelemetryRecord contains bounded topic and payload, nonzero sequence, explicit
source/publish/record clock domains, schema identity, and original encoded
Protobuf bytes. The writer emits proto-generated FILE_DESCRIPTOR_SET once per
schema identity and channel metadata for every canonical topic.

Separate bounded queues reserve capacity for three classes: critical
fault/lifecycle/command evidence, control state, and bulk input. A full class
coalesces typed gap evidence for that class. Disk errors, quota exhaustion, and
write failure mark the recorder degraded/unavailable. They never block the
producer or affect a fake-sink decision.

The writer creates a new staging name, writes bounded chunks and indexes,
seals to a new final path, then writes a sorted SHA-256 manifest. Startup scans
only task-owned staging names. A valid complete staging file is sealed
atomically; a malformed/truncated file is retained under a failed evidence
name. Existing output is never overwritten.

## Replay and compatibility

TelemetryReplay requires a sealed path, expected descriptor digest, and topic
allowlist. It checks magic, schema bytes, channel metadata, sequence
monotonicity, and time order, then yields bounded ReplayInput test values. Its
public API has no driver, sink, lease grant, or send method.

Before adding mcap version 0.25.0, inspect exact resolved features, license and
source. Keep it in the recorder integration crate; do not substitute another
format if admission fails. Add every new workspace member/class atomically.
Existing protocol schemas, local IPC layouts, YAML, and navigation diagnostics
remain unmodified. No unsafe, mmap, direct device descriptor, or global logger
is needed.

## Qualification and rollback

Host tests prove software bounds/failure behavior only, not SCHED_FIFO,
mlockall, affinity, kernel jitter, disk hardware throughput, target performance,
CAN behavior, or physical motor stopping. Rollback removes the new members while
preserving protocol/lifecycle contracts. Disabling a failed recorder retains the
independent local SafeStop path.
