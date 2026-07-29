# Pinned kernel build and guarded installation

## What is pinned

`../sources.lock` fixes the Orange Pi kernel and build repositories, the complete
vendor RT config and its hash, the Arm GNU cross-toolchain archive and hash, the
Ubuntu 22.04 container digest, rt-tests v2.10 and the reviewed Horus commit. The
project fragment is applied after the complete vendor config, then the final
`.config` is checked independently against `kernel/required.config`.

The build script never installs packages or changes target boot files.

## Verify before network or build

From the repository root:

```bash
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/kernel/build-kernel.sh --print-plan
```

Both commands are offline. A changed digest, incomplete source lock or missing
thermal/RT/CAN/media requirement fails before any fetch.

## Reproducible container entry point

Use fresh absolute directories. The repository mount may be read-only because
all source, toolchain and build output lives in the two explicit writable mounts.

```bash
docker build \
  --file platform/orangepi5plus/container/Containerfile \
  --tag rs-control-orangepi-kernel:local \
  .

mkdir -p /absolute/path/rs-kernel-work /absolute/path/rs-kernel-output

docker run --rm \
  --volume "$PWD:/workspace:ro" \
  --volume /absolute/path/rs-kernel-work:/work \
  --volume /absolute/path/rs-kernel-output:/output \
  rs-control-orangepi-kernel:local \
  --fetch --build \
  --work-dir /work \
  --output-dir /output \
  --jobs 8
```

The output directory must be absent or empty. The work directory must not
contain old `.deb` files, and both fetched repositories must remain at their
pinned commit with no tracked modifications. Successful output contains Debian
packages, `kernel.config`, `build-metadata.env` and `SHA256SUMS`.

## Target installation gate

Installing a kernel is an explicit target-board operation, not part of CI.
Before installation:

1. verify every output against `SHA256SUMS`;
2. record the current kernel, `/boot` layout and installed kernel packages;
3. verify a working 3.3 V serial console and vendor-kernel boot path;
4. keep the current vendor kernel and its device tree installed;
5. copy the evidence template and fill every physical field;
6. ensure motion power is removed and no motor is connected.

Install only the newly built image, modules, headers and DT packages selected
from the manifest. Do not purge the vendor kernel. Do not make an unverified
entry the only bootable choice. Reboot first through the serial console, select
the new entry explicitly, and immediately collect a pre-tuning evidence bundle.

On the target, the minimum post-boot checks are:

```bash
uname -a
cat /sys/kernel/realtime
zcat /proc/config.gz | grep -E 'CONFIG_(PREEMPT_RT|CPU_FREQ_THERMAL|CAN_ROCKCHIP)='
```

Continue only when PREEMPT_RT reports active and the installed config retains
thermal protection. A kernel package that boots is not yet a qualified realtime
platform.
