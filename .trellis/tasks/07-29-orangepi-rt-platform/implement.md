# Orange Pi 5 Plus realtime platform — implementation plan

## 1. Planning and provenance

- [x] Persist the approved scope, host gate, target-board gate and standing implementation authorization.
- [x] Record official Orange Pi, kernel.org, Arm toolchain, Ubuntu container, `rt-tests` and Horus source identities in `research/platform-baseline.md`.
- [x] Update the parent PRD/design/implementation plan so the Horus-inspired affinity/SHM/queue requirements survive into the protocol-module child.
- [x] Activate this child task only after `prd.md`, `design.md` and this plan are complete.

Review gate: planning artifacts must have no unresolved design choice that changes host implementation. Unknown physical transceiver/board evidence is an explicit target gate, not a reason to invent a model.

## 2. Source lock and kernel build

- [x] Add `platform/orangepi5plus/sources.lock` with an allow-listed data format and all immutable identities/hashes.
- [x] Add a pinned Ubuntu 22.04 AMD64 build `Containerfile`.
- [x] Verify and use the pinned build repository's complete `linux-rockchip-rk3588-legacy-rt.config`, then apply the project fragment including `CONFIG_CPU_FREQ_THERMAL=y` last.
- [x] Add an independent required-config assertion file.
- [x] Implement `kernel/build-kernel.sh` with `--verify-only`, `--print-plan`, `--fetch` and `--build` modes plus explicit work/output directories.
- [x] Make fetch/build use the exact kernel commit and verified Arm toolchain; do not reapply the upstream RT patch to the already-integrated vendor tree.
- [x] Emit final `.config`, build metadata, Debian packages and SHA-256 files without installing them.

Fast validation:

```bash
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/kernel/build-kernel.sh --print-plan
```

Heavy/manual validation:

```bash
platform/orangepi5plus/kernel/build-kernel.sh \
  --fetch --build \
  --work-dir /absolute/path/to/work \
  --output-dir /absolute/path/to/artifacts
```

Rollback point: this stage writes only caller-selected build/output directories and never changes the workstation or target boot configuration.

## 3. Runtime profile and exact affinity

- [x] Add a candidate RK3588 profile separating housekeeping, CAN I/O/IRQ and motion-control CPU roles.
- [x] Implement strict CPU-list parsing and validation against online/allowed CPUs, role conflicts and non-empty housekeeping.
- [x] Implement `plan`, `check`, `apply` and `snapshot` operations for governor/IRQ/profile state.
- [x] Resolve the selected CAN IRQ from target sysfs/proc data, apply only the intended IRQ affinity and verify readback.
- [x] Never implement a CPU candidate list as “first successful single-core pin”; preserve exact single CPU vs exact CPU-set semantics.
- [x] Keep RT throttling enabled and make boot isolation parameters a measured candidate, not an automatic edit.
- [x] Make qualified checks fail closed; allow an explicitly labelled development-only check mode.

Host tests use fixture roots instead of `/sys` and `/proc`. No host test needs root or writes real affinity.

## 4. SocketCAN platform manager

- [x] Add the minimal platform CAN config for interface/bitrate only; do not add robot motor IDs or protocols.
- [x] Implement `plan`, `up`, `status`, `recover` and `down` with interface validation and command dry-run.
- [x] Force classic CAN 1 Mbps, bus-error reporting and `restart-ms 0`.
- [x] Require explicit disarmed confirmation for disruptive `recover`/`down` operations.
- [x] Add systemd example ordering the privileged CAN preparation before later motion services.
- [x] Document CAN0/CAN1 overlays, physical pins, 3.3 V isolated transceiver boundary and two-endpoint termination.

Host validation must assert that no command enables a motor, sends a CAN frame or configures automatic bus-off restart.

## 5. Evidence and realtime qualification

- [x] Implement a collector that creates a new evidence directory and records source/build/board/kernel/DT/CPU/IRQ/governor/thermal/CAN/tool identities.
- [x] Implement `qualify-rt.sh` profiles for 1 kHz and explicit 500 Hz fallback.
- [x] Pin cyclictest invocation to FIFO priority 90, `mlockall`, nanosecond JSON, an exact measurement CPU and housekeeping main thread.
- [x] Parse raw JSON with strict return-code/unit/cycle/max checks; boundary-test `100000 ns` pass vs `100001 ns` fail.
- [x] Add baseline and concurrent stress modes; require real network peer and accelerator runner for a complete stress qualification.
- [x] Monitor temperature/frequency and propagate stress/monitor failures to `fail` or `incomplete`.
- [x] Generate a stable verdict plus `SHA256SUMS`, never overwriting raw evidence.
- [x] Document the eight-hour soak and the distinction between kernel wake latency and future application deadline evidence.

Fast fixture validation:

```bash
platform/orangepi5plus/tests/host-test.sh
```

Target validation (not run from the development host):

```bash
sudo platform/orangepi5plus/scripts/qualify-rt.sh \
  --profile 1khz \
  --mode concurrent \
  --duration 8h \
  --output /var/lib/rs-control/qualification/<unique-run-id> \
  --hardware-manifest /absolute/path/hardware-evidence.env \
  --overlay-file /absolute/path/rk3588-can0-m0.dtbo \
  --network-peer <test-peer> \
  --network-interface <ethernet-interface> \
  --accelerator-runner /absolute/path/validated-runner \
  --storage-path /absolute/path/on-target-storage \
  --app-build-id <immutable-build-id>
```

Rollback point: stop only qualification-owned stress processes, restore the snapshot of governor/IRQ settings, and retain the evidence directory.

## 6. Documentation and CI integration

- [x] Add a platform README with quick navigation and a prominent “not hardware-qualified until evidence exists” banner.
- [x] Add build/install/bring-up instructions that preserve a bootable vendor kernel and require serial recovery.
- [x] Add realtime qualification interpretation and rollback runbooks.
- [x] Add systemd unit examples with minimal privileges and observable failures.
- [x] Add the fast platform host test to `tools/ci.sh`; keep kernel builds and target mutation out of ordinary CI.
- [x] Ensure shell scripts pass `sh -n`; use `shellcheck` when installed without making an absent optional tool fail the repository gate.

## 7. Quality gate

Run the project-scoped `trellis-check` workflow and at minimum:

```bash
platform/orangepi5plus/tests/host-test.sh
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
sh -n platform/orangepi5plus/kernel/build-kernel.sh
sh -n platform/orangepi5plus/scripts/*.sh
tools/ci.sh
git diff --check
```

Review for:

- no hidden network or root operation in fast CI;
- no blind source branch, container tag or unchecked download;
- no auto-install/bootloader/old-kernel deletion;
- no motor enable/frame transmission;
- no silent RT/affinity/governor/IRQ fallback in a qualified profile;
- no expansion of robot topology YAML;
- no dependency-boundary change.

## 8. Commit and ongoing state

- [ ] Commit planning/provenance and implementation in reviewable logical commits.
- [ ] Push to the authorized GitHub remote after the full host gate passes.
- [ ] Do not archive this child after host-only success. Keep it `in_progress` with a target-evidence checklist until a real Orange Pi 5 Plus supplies the complete hardware/8-hour qualification bundle.
- [ ] Proceed to the SocketCAN/Damiao child with an explicit note that hard-realtime performance remains pending target qualification.
