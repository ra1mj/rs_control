#!/bin/sh
set -eu

kernel_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
kernel_platform_root=$(CDPATH= cd -- "$kernel_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$kernel_platform_root/lib/common.sh"

kernel_source_keys='LOCK_FORMAT_VERSION KERNEL_REPOSITORY KERNEL_BRANCH KERNEL_COMMIT KERNEL_REPORTED_VERSION KERNEL_RT_SUFFIX KERNEL_RT_INTEGRATION_COMMIT KERNEL_ROCKCHIP_FIX_COMMIT KERNEL_LATENCY_CONFIG_COMMIT RT_PATCH_URL RT_PATCH_SHA256 ORANGEPI_BUILD_REPOSITORY ORANGEPI_BUILD_COMMIT ORANGEPI_RT_CONFIG_PATH ORANGEPI_RT_CONFIG_SHA256 UBOOT_REPOSITORY UBOOT_BRANCH UBOOT_COMMIT RKBIN_REPOSITORY RKBIN_COMMIT TOOLCHAIN_URL TOOLCHAIN_SHA256 TOOLCHAIN_DIRECTORY BUILD_CONTAINER RT_TESTS_REPOSITORY RT_TESTS_TAG RT_TESTS_COMMIT HORUS_REPOSITORY HORUS_COMMIT HORUS_LICENSE'
platform_load_data_file "$kernel_platform_root/sources.lock" "$kernel_source_keys"

kernel_usage() {
    printf '%s\n' \
        'Usage: build-kernel.sh [operations] [options]' \
        '' \
        'Operations (one or more):' \
        '  --verify-only          Validate pinned inputs without network or build' \
        '  --print-plan           Print immutable sources and build commands' \
        '  --fetch                Fetch exact sources/toolchain into --work-dir' \
        '  --build                Configure and build Debian kernel packages' \
        '' \
        'Options:' \
        '  --work-dir ABSOLUTE    Explicit source/build directory for fetch/build' \
        '  --output-dir ABSOLUTE  Fresh output directory required by --build' \
        '  --jobs N               Parallel build jobs (default: online CPU count)' \
        '  -h, --help             Show this help' \
        '' \
        'This script never installs a kernel or modifies target boot files.'
}

kernel_verify_lock_shape() {
    [ "$LOCK_FORMAT_VERSION" = 1 ] || platform_die "unsupported source lock format: $LOCK_FORMAT_VERSION"
    for kernel_commit in \
        "$KERNEL_COMMIT" \
        "$KERNEL_RT_INTEGRATION_COMMIT" \
        "$KERNEL_ROCKCHIP_FIX_COMMIT" \
        "$KERNEL_LATENCY_CONFIG_COMMIT" \
        "$ORANGEPI_BUILD_COMMIT" \
        "$UBOOT_COMMIT" \
        "$RKBIN_COMMIT" \
        "$RT_TESTS_COMMIT" \
        "$HORUS_COMMIT"; do
        printf '%s\n' "$kernel_commit" | grep -Eq '^[0-9a-f]{40}$' ||
            platform_die "invalid pinned commit: $kernel_commit"
    done
    for kernel_hash in "$RT_PATCH_SHA256" "$ORANGEPI_RT_CONFIG_SHA256" "$TOOLCHAIN_SHA256"; do
        printf '%s\n' "$kernel_hash" | grep -Eq '^[0-9a-f]{64}$' ||
            platform_die "invalid pinned SHA-256: $kernel_hash"
    done
    printf '%s\n' "$BUILD_CONTAINER" | grep -Eq '^[A-Za-z0-9._/-]+:[A-Za-z0-9._-]+@sha256:[0-9a-f]{64}$' ||
        platform_die "build container must use an exact sha256 digest"
}

kernel_verify_config_contract() {
    kernel_fragment=$kernel_script_dir/rs-control-rt.fragment
    kernel_required=$kernel_script_dir/required.config
    [ -s "$kernel_fragment" ] || platform_die "missing project kernel fragment"
    [ -s "$kernel_required" ] || platform_die "missing required kernel config"

    while IFS= read -r kernel_requirement; do
        case "$kernel_requirement" in
            ''|'#'*) continue ;;
        esac
        grep -Fqx "$kernel_requirement" "$kernel_fragment" ||
            platform_die "required setting is not forced by project fragment: $kernel_requirement"
    done < "$kernel_required"

    grep -Fqx 'CONFIG_CPU_FREQ_THERMAL=y' "$kernel_fragment" ||
        platform_die 'thermal protection is not forced on'
    if grep -Eq '^# CONFIG_CPU_FREQ_THERMAL is not set$' "$kernel_fragment"; then
        platform_die 'project fragment disables thermal protection'
    fi
}

kernel_verify_inputs() {
    kernel_verify_lock_shape
    kernel_verify_config_contract
    printf 'source lock: %s\n' "$(platform_sha256 "$kernel_platform_root/sources.lock")"
    printf 'kernel fragment: %s\n' "$(platform_sha256 "$kernel_script_dir/rs-control-rt.fragment")"
    printf 'required config: %s\n' "$(platform_sha256 "$kernel_script_dir/required.config")"
    printf '%s\n' 'kernel input verification: PASS'
}

kernel_print_plan() {
    cat <<EOF
Orange Pi 5 Plus kernel build plan
kernel_repository=$KERNEL_REPOSITORY
kernel_branch_provenance=$KERNEL_BRANCH
kernel_commit=$KERNEL_COMMIT
kernel_reported_version=$KERNEL_REPORTED_VERSION$KERNEL_RT_SUFFIX
rt_patch_provenance=$RT_PATCH_URL
rt_patch_sha256=$RT_PATCH_SHA256
orangepi_build_repository=$ORANGEPI_BUILD_REPOSITORY
orangepi_build_commit=$ORANGEPI_BUILD_COMMIT
vendor_config=$ORANGEPI_RT_CONFIG_PATH
vendor_config_sha256=$ORANGEPI_RT_CONFIG_SHA256
toolchain=$TOOLCHAIN_URL
toolchain_sha256=$TOOLCHAIN_SHA256
container=$BUILD_CONTAINER
configuration=verified-vendor-config + rs-control-rt.fragment + olddefconfig
build_target=bindeb-pkg
installation=never-performed-by-this-script
EOF
}

kernel_fetch_git_commit() {
    kernel_fetch_repository=$1
    kernel_fetch_commit=$2
    kernel_fetch_destination=$3

    if [ -d "$kernel_fetch_destination/.git" ]; then
        kernel_fetch_head=$(git -C "$kernel_fetch_destination" rev-parse HEAD)
        [ "$kernel_fetch_head" = "$kernel_fetch_commit" ] ||
            platform_die "existing repository has unexpected HEAD: $kernel_fetch_destination ($kernel_fetch_head)"
        kernel_fetch_origin=$(git -C "$kernel_fetch_destination" config --get remote.origin.url)
        [ "$kernel_fetch_origin" = "$kernel_fetch_repository" ] ||
            platform_die "existing repository has unexpected origin: $kernel_fetch_destination ($kernel_fetch_origin)"
        return
    fi
    [ ! -e "$kernel_fetch_destination" ] ||
        platform_die "fetch destination exists but is not the expected repository: $kernel_fetch_destination"

    mkdir -p "$kernel_fetch_destination"
    git -C "$kernel_fetch_destination" init
    git -C "$kernel_fetch_destination" remote add origin "$kernel_fetch_repository"
    git -C "$kernel_fetch_destination" fetch --depth 1 origin "$kernel_fetch_commit"
    git -C "$kernel_fetch_destination" -c advice.detachedHead=false switch --detach FETCH_HEAD
    kernel_fetch_head=$(git -C "$kernel_fetch_destination" rev-parse HEAD)
    [ "$kernel_fetch_head" = "$kernel_fetch_commit" ] ||
        platform_die "fetched repository did not resolve to pinned commit: $kernel_fetch_destination"
}

kernel_fetch_all() {
    platform_require_command curl
    platform_require_command git
    platform_require_command tar
    platform_require_command xz

    mkdir -p "$kernel_work_dir/sources" "$kernel_work_dir/downloads" "$kernel_work_dir/toolchains"
    kernel_fetch_git_commit "$KERNEL_REPOSITORY" "$KERNEL_COMMIT" "$kernel_work_dir/sources/linux-orangepi"
    kernel_fetch_git_commit "$ORANGEPI_BUILD_REPOSITORY" "$ORANGEPI_BUILD_COMMIT" "$kernel_work_dir/sources/orangepi-build"

    kernel_toolchain_archive=$kernel_work_dir/downloads/$TOOLCHAIN_DIRECTORY.tar.xz
    if [ ! -f "$kernel_toolchain_archive" ]; then
        curl --fail --location --proto '=https' --tlsv1.2 \
            --output "$kernel_toolchain_archive.part" "$TOOLCHAIN_URL"
        platform_verify_sha256 "$TOOLCHAIN_SHA256" "$kernel_toolchain_archive.part"
        mv "$kernel_toolchain_archive.part" "$kernel_toolchain_archive"
    fi
    platform_verify_sha256 "$TOOLCHAIN_SHA256" "$kernel_toolchain_archive"

    if [ ! -x "$kernel_work_dir/toolchains/$TOOLCHAIN_DIRECTORY/bin/aarch64-none-linux-gnu-gcc" ]; then
        [ ! -e "$kernel_work_dir/toolchains/$TOOLCHAIN_DIRECTORY" ] ||
            platform_die "incomplete toolchain directory exists: $kernel_work_dir/toolchains/$TOOLCHAIN_DIRECTORY"
        tar -xJf "$kernel_toolchain_archive" -C "$kernel_work_dir/toolchains"
    fi

    kernel_vendor_config=$kernel_work_dir/sources/orangepi-build/$ORANGEPI_RT_CONFIG_PATH
    platform_verify_sha256 "$ORANGEPI_RT_CONFIG_SHA256" "$kernel_vendor_config"
    printf '%s\n' 'pinned sources and toolchain are ready'
}

kernel_verify_final_config() {
    kernel_final_config=$1
    while IFS= read -r kernel_requirement; do
        case "$kernel_requirement" in
            ''|'#'*) continue ;;
        esac
        grep -Fqx "$kernel_requirement" "$kernel_final_config" ||
            platform_die "final kernel config is missing requirement: $kernel_requirement"
    done < "$kernel_script_dir/required.config"
}

kernel_build_packages() {
    platform_require_command make
    kernel_source_dir=$kernel_work_dir/sources/linux-orangepi
    kernel_build_source_dir=$kernel_work_dir/sources/orangepi-build
    kernel_cross_prefix=$kernel_work_dir/toolchains/$TOOLCHAIN_DIRECTORY/bin/aarch64-none-linux-gnu-
    kernel_vendor_config=$kernel_build_source_dir/$ORANGEPI_RT_CONFIG_PATH
    kernel_build_dir=$kernel_work_dir/kernel-build

    [ "$(git -C "$kernel_source_dir" rev-parse HEAD)" = "$KERNEL_COMMIT" ] ||
        platform_die 'kernel source is not at the pinned commit'
    [ "$(git -C "$kernel_build_source_dir" rev-parse HEAD)" = "$ORANGEPI_BUILD_COMMIT" ] ||
        platform_die 'Orange Pi build source is not at the pinned commit'
    [ -z "$(git -C "$kernel_source_dir" status --porcelain --untracked-files=no)" ] ||
        platform_die 'kernel source has tracked modifications'
    [ -z "$(git -C "$kernel_build_source_dir" status --porcelain --untracked-files=no)" ] ||
        platform_die 'Orange Pi build source has tracked modifications'
    [ -x "${kernel_cross_prefix}gcc" ] || platform_die 'pinned cross compiler is missing'
    platform_verify_sha256 "$ORANGEPI_RT_CONFIG_SHA256" "$kernel_vendor_config"
    for kernel_existing_package in "$kernel_work_dir"/*.deb; do
        [ ! -e "$kernel_existing_package" ] ||
            platform_die "work directory contains a stale Debian package: $kernel_existing_package"
    done

    mkdir -p "$kernel_build_dir"
    "$kernel_source_dir/scripts/kconfig/merge_config.sh" -m -O "$kernel_build_dir" \
        "$kernel_vendor_config" "$kernel_script_dir/rs-control-rt.fragment"

    kernel_source_epoch=$(git -C "$kernel_source_dir" show -s --format=%ct "$KERNEL_COMMIT")
    export ARCH=arm64
    export CROSS_COMPILE=$kernel_cross_prefix
    export KBUILD_BUILD_USER=rs-control
    export KBUILD_BUILD_HOST=orangepi5plus
    export SOURCE_DATE_EPOCH=$kernel_source_epoch
    export LC_ALL=C
    export TZ=UTC

    make -C "$kernel_source_dir" O="$kernel_build_dir" olddefconfig
    kernel_verify_final_config "$kernel_build_dir/.config"

    cp "$kernel_build_dir/.config" "$kernel_output_dir/kernel.config"
    {
        printf 'kernel_commit=%s\n' "$KERNEL_COMMIT"
        printf 'kernel_source_epoch=%s\n' "$kernel_source_epoch"
        printf 'kernel_reported_version=%s%s\n' "$KERNEL_REPORTED_VERSION" "$KERNEL_RT_SUFFIX"
        printf 'orangepi_build_commit=%s\n' "$ORANGEPI_BUILD_COMMIT"
        printf 'vendor_config_sha256=%s\n' "$ORANGEPI_RT_CONFIG_SHA256"
        printf 'source_lock_sha256=%s\n' "$(platform_sha256 "$kernel_platform_root/sources.lock")"
        printf 'final_config_sha256=%s\n' "$(platform_sha256 "$kernel_build_dir/.config")"
        printf 'cross_compiler=%s\n' "$("${kernel_cross_prefix}gcc" --version | sed -n '1p')"
    } > "$kernel_output_dir/build-metadata.env"

    make -C "$kernel_source_dir" O="$kernel_build_dir" -j "$kernel_jobs" \
        LOCALVERSION=-rscontrol-rt \
        KDEB_PKGVERSION="$KERNEL_REPORTED_VERSION$KERNEL_RT_SUFFIX+rscontrol.1" \
        bindeb-pkg

    kernel_package_count=0
    for kernel_package in "$kernel_work_dir"/*.deb; do
        [ -f "$kernel_package" ] || continue
        cp "$kernel_package" "$kernel_output_dir/"
        kernel_package_count=$((kernel_package_count + 1))
    done
    [ "$kernel_package_count" -gt 0 ] || platform_die 'kernel build produced no Debian packages'

    (
        cd "$kernel_output_dir"
        sha256sum ./*.deb kernel.config build-metadata.env > SHA256SUMS
    )
    printf 'kernel packages written to %s\n' "$kernel_output_dir"
}

kernel_do_verify=0
kernel_do_print=0
kernel_do_fetch=0
kernel_do_build=0
kernel_work_dir=
kernel_output_dir=
kernel_jobs=$(getconf _NPROCESSORS_ONLN 2>/dev/null || printf '1\n')

while [ "$#" -gt 0 ]; do
    case "$1" in
        --verify-only) kernel_do_verify=1 ;;
        --print-plan) kernel_do_print=1 ;;
        --fetch) kernel_do_fetch=1 ;;
        --build) kernel_do_build=1 ;;
        --work-dir)
            [ "$#" -ge 2 ] || platform_die '--work-dir requires a value'
            kernel_work_dir=$2
            shift
            ;;
        --output-dir)
            [ "$#" -ge 2 ] || platform_die '--output-dir requires a value'
            kernel_output_dir=$2
            shift
            ;;
        --jobs)
            [ "$#" -ge 2 ] || platform_die '--jobs requires a value'
            kernel_jobs=$2
            shift
            ;;
        -h|--help)
            kernel_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

[ "$kernel_do_verify$kernel_do_print$kernel_do_fetch$kernel_do_build" != 0000 ] || {
    kernel_usage >&2
    exit 2
}
case "$kernel_jobs" in
    ''|*[!0-9]*) platform_die "--jobs must be a positive integer: $kernel_jobs" ;;
esac
[ "$kernel_jobs" -gt 0 ] || platform_die '--jobs must be greater than zero'

kernel_verify_inputs
[ "$kernel_do_print" -eq 0 ] || kernel_print_plan

if [ "$kernel_do_fetch" -eq 1 ] || [ "$kernel_do_build" -eq 1 ]; then
    [ -n "$kernel_work_dir" ] || platform_die '--work-dir is required for fetch/build'
    platform_require_absolute_path '--work-dir' "$kernel_work_dir"
    mkdir -p "$kernel_work_dir"
fi

if [ "$kernel_do_build" -eq 1 ]; then
    [ -n "$kernel_output_dir" ] || platform_die '--output-dir is required for --build'
    platform_require_absolute_path '--output-dir' "$kernel_output_dir"
    if [ -d "$kernel_output_dir" ] && [ -n "$(find "$kernel_output_dir" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
        platform_die "output directory must be absent or empty: $kernel_output_dir"
    fi
    mkdir -p "$kernel_output_dir"
fi

[ "$kernel_do_fetch" -eq 0 ] || kernel_fetch_all
[ "$kernel_do_build" -eq 0 ] || kernel_build_packages
