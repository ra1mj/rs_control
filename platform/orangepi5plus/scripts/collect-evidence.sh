#!/bin/sh
set -eu

evidence_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
evidence_platform_root=$(CDPATH= cd -- "$evidence_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$evidence_platform_root/lib/common.sh"

evidence_usage() {
    printf '%s\n' \
        'Usage: collect-evidence.sh --output ABSOLUTE_DIR [options]' \
        '' \
        'Options:' \
        '  --profile FILE             RT profile (default: config/rt-profile.env)' \
        '  --hardware-manifest FILE   Filled target hardware evidence file' \
        '  --overlay-file FILE        Active CAN DT overlay file to hash' \
        '  --can-interface IFACE      CAN interface (default: can0)' \
        '  --network-peer HOST        Identified qualification peer used for route evidence' \
        '  --network-interface IFACE  Physical Ethernet interface used for qualification' \
        '  --app-build-id TOKEN       Application/build identity (default: unavailable)' \
        '  --sys-root PATH            sysfs root for fixtures (default: /sys)' \
        '  --proc-root PATH           procfs root for fixtures (default: /proc)' \
        '  --etc-root PATH            etc root for fixtures (default: /etc)' \
        '  -h, --help                 Show this help' \
        '' \
        'Collection is read-only. Missing observations are recorded, never invented.'
}

evidence_output=
evidence_profile=$evidence_platform_root/config/rt-profile.env
evidence_hardware_manifest=
evidence_overlay_file=
evidence_can_interface=can0
evidence_network_peer=
evidence_network_interface=
evidence_app_build_id=unavailable
evidence_sys_root=/sys
evidence_proc_root=/proc
evidence_etc_root=/etc

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output)
            [ "$#" -ge 2 ] || platform_die '--output requires a value'
            evidence_output=$2
            shift
            ;;
        --profile)
            [ "$#" -ge 2 ] || platform_die '--profile requires a value'
            evidence_profile=$2
            shift
            ;;
        --hardware-manifest)
            [ "$#" -ge 2 ] || platform_die '--hardware-manifest requires a value'
            evidence_hardware_manifest=$2
            shift
            ;;
        --overlay-file)
            [ "$#" -ge 2 ] || platform_die '--overlay-file requires a value'
            evidence_overlay_file=$2
            shift
            ;;
        --can-interface)
            [ "$#" -ge 2 ] || platform_die '--can-interface requires a value'
            evidence_can_interface=$2
            shift
            ;;
        --network-peer)
            [ "$#" -ge 2 ] || platform_die '--network-peer requires a value'
            evidence_network_peer=$2
            shift
            ;;
        --network-interface)
            [ "$#" -ge 2 ] || platform_die '--network-interface requires a value'
            evidence_network_interface=$2
            shift
            ;;
        --app-build-id)
            [ "$#" -ge 2 ] || platform_die '--app-build-id requires a value'
            evidence_app_build_id=$2
            shift
            ;;
        --sys-root)
            [ "$#" -ge 2 ] || platform_die '--sys-root requires a value'
            evidence_sys_root=$2
            shift
            ;;
        --proc-root)
            [ "$#" -ge 2 ] || platform_die '--proc-root requires a value'
            evidence_proc_root=$2
            shift
            ;;
        --etc-root)
            [ "$#" -ge 2 ] || platform_die '--etc-root requires a value'
            evidence_etc_root=$2
            shift
            ;;
        -h|--help)
            evidence_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

[ -n "$evidence_output" ] || platform_die '--output is required'
platform_require_absolute_path '--output' "$evidence_output"
platform_require_absolute_path '--sys-root' "$evidence_sys_root"
platform_require_absolute_path '--proc-root' "$evidence_proc_root"
platform_require_absolute_path '--etc-root' "$evidence_etc_root"
[ -r "$evidence_profile" ] || platform_die "profile is not readable: $evidence_profile"
[ -z "$evidence_hardware_manifest" ] || [ -r "$evidence_hardware_manifest" ] ||
    platform_die "hardware manifest is not readable: $evidence_hardware_manifest"
printf '%s\n' "$evidence_can_interface" | grep -Eq '^can[0-9]+$' || platform_die "invalid CAN interface: $evidence_can_interface"
[ -z "$evidence_network_peer" ] || printf '%s\n' "$evidence_network_peer" | grep -Eq '^[A-Za-z0-9][A-Za-z0-9.:-]*$' ||
    platform_die 'network peer must be a hostname or address token'
[ -z "$evidence_network_interface" ] || printf '%s\n' "$evidence_network_interface" | grep -Eq '^[A-Za-z0-9][A-Za-z0-9_.:-]*$' ||
    platform_die 'network interface must be a link-name token'
printf '%s\n' "$evidence_app_build_id" | grep -Eq '^[A-Za-z0-9._:+-]+$' || platform_die 'app build ID must be a data token'

if [ -d "$evidence_output" ] && [ -n "$(find "$evidence_output" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    platform_die "evidence output must be absent or empty: $evidence_output"
fi
mkdir -p "$evidence_output"
evidence_status_file=$evidence_output/collection-status.env
: > "$evidence_status_file"

evidence_record_status() {
    printf '%s=%s\n' "$1" "$2" >> "$evidence_status_file"
}

evidence_capture_command() {
    evidence_name=$1
    shift
    if "$@" > "$evidence_output/$evidence_name.txt" 2> "$evidence_output/$evidence_name.stderr"; then
        evidence_record_status "$evidence_name" 0
    else
        evidence_record_status "$evidence_name" "$?"
    fi
}

evidence_capture_file() {
    evidence_name=$1
    evidence_source=$2
    if [ -r "$evidence_source" ]; then
        cp "$evidence_source" "$evidence_output/$evidence_name.txt"
        evidence_record_status "$evidence_name" 0
    else
        printf 'unavailable: %s\n' "$evidence_source" > "$evidence_output/$evidence_name.stderr"
        evidence_record_status "$evidence_name" 1
    fi
}

evidence_capture_device_tree() {
    evidence_dt_source=$1
    evidence_dt_target=$2
    if [ -r "$evidence_dt_source" ]; then
        tr '\000' '\n' < "$evidence_dt_source" > "$evidence_dt_target"
        return 0
    fi
    return 1
}

evidence_capture_cpu_tree() {
    evidence_cpu_count=0
    for evidence_cpu_dir in "$evidence_sys_root"/devices/system/cpu/cpu[0-9]*; do
        [ -d "$evidence_cpu_dir" ] || continue
        evidence_cpu_count=$((evidence_cpu_count + 1))
        evidence_cpu=$(basename "$evidence_cpu_dir")
        for evidence_field in cpu_capacity topology/core_id topology/physical_package_id cpufreq/scaling_driver cpufreq/scaling_governor cpufreq/scaling_cur_freq cpufreq/scaling_max_freq; do
            if [ -r "$evidence_cpu_dir/$evidence_field" ]; then
                printf '%s/%s=%s\n' "$evidence_cpu" "$evidence_field" "$(tr -d '[:space:]' < "$evidence_cpu_dir/$evidence_field")"
            fi
        done
    done
    [ "$evidence_cpu_count" -gt 0 ]
}

evidence_capture_thermal_tree() {
    evidence_zone_count=0
    for evidence_zone in "$evidence_sys_root"/class/thermal/thermal_zone*; do
        [ -d "$evidence_zone" ] || continue
        evidence_zone_count=$((evidence_zone_count + 1))
        evidence_zone_name=$(basename "$evidence_zone")
        evidence_zone_type=unavailable
        evidence_zone_temp=unavailable
        [ ! -r "$evidence_zone/type" ] || evidence_zone_type=$(tr -d '[:space:]' < "$evidence_zone/type")
        [ ! -r "$evidence_zone/temp" ] || evidence_zone_temp=$(tr -d '[:space:]' < "$evidence_zone/temp")
        printf '%s type=%s temp_millic=%s\n' "$evidence_zone_name" "$evidence_zone_type" "$evidence_zone_temp"
        for evidence_trip_type_file in "$evidence_zone"/trip_point_*_type; do
            [ -r "$evidence_trip_type_file" ] || continue
            evidence_trip_name=$(basename "${evidence_trip_type_file%_type}")
            evidence_trip_type=$(tr -d '[:space:]' < "$evidence_trip_type_file")
            evidence_trip_temp=unavailable
            [ ! -r "${evidence_trip_type_file%_type}_temp" ] ||
                evidence_trip_temp=$(tr -d '[:space:]' < "${evidence_trip_type_file%_type}_temp")
            printf '%s %s type=%s temp_millic=%s\n' "$evidence_zone_name" "$evidence_trip_name" "$evidence_trip_type" "$evidence_trip_temp"
        done
    done
    [ "$evidence_zone_count" -gt 0 ]
}

cp "$evidence_platform_root/sources.lock" "$evidence_output/sources.lock"
cp "$evidence_profile" "$evidence_output/rt-profile.env"
[ -z "$evidence_hardware_manifest" ] || cp "$evidence_hardware_manifest" "$evidence_output/hardware-evidence.env"

if [ -n "$evidence_overlay_file" ]; then
    if [ -r "$evidence_overlay_file" ]; then
        cp "$evidence_overlay_file" "$evidence_output/can-overlay.dtbo"
        evidence_record_status can_overlay 0
    else
        evidence_record_status can_overlay 1
    fi
else
    evidence_record_status can_overlay 1
fi

{
    printf 'collection_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'app_build_id=%s\n' "$evidence_app_build_id"
    printf 'sources_lock_sha256=%s\n' "$(platform_sha256 "$evidence_platform_root/sources.lock")"
    printf 'rt_profile_sha256=%s\n' "$(platform_sha256 "$evidence_profile")"
} > "$evidence_output/manifest.env"
printf '%s  sources.lock\n' "$(platform_sha256 "$evidence_output/sources.lock")" > "$evidence_output/source-lock.sha256"
printf '%s  manifest.env\n' "$(platform_sha256 "$evidence_output/manifest.env")" > "$evidence_output/manifest.sha256"

evidence_capture_command uname uname -a
evidence_capture_file os_release "$evidence_etc_root/os-release"
evidence_capture_file kernel_cmdline "$evidence_proc_root/cmdline"
evidence_capture_file kernel_version "$evidence_proc_root/version"
evidence_capture_file kernel_realtime "$evidence_sys_root/kernel/realtime"
evidence_capture_file cpu_online "$evidence_sys_root/devices/system/cpu/online"
evidence_capture_file cpu_isolated "$evidence_sys_root/devices/system/cpu/isolated"
evidence_capture_file cpu_nohz_full "$evidence_sys_root/devices/system/cpu/nohz_full"
evidence_capture_file interrupts "$evidence_proc_root/interrupts"

if evidence_capture_device_tree "$evidence_proc_root/device-tree/model" "$evidence_output/device-tree-model.txt"; then
    evidence_record_status device_tree_model 0
else
    evidence_record_status device_tree_model 1
fi
if evidence_capture_device_tree "$evidence_proc_root/device-tree/compatible" "$evidence_output/device-tree-compatible.txt"; then
    evidence_record_status device_tree_compatible 0
else
    evidence_record_status device_tree_compatible 1
fi

if evidence_capture_cpu_tree > "$evidence_output/cpu-topology.txt" 2> "$evidence_output/cpu-topology.stderr"; then
    evidence_record_status cpu_topology 0
else
    evidence_record_status cpu_topology "$?"
fi
if evidence_capture_thermal_tree > "$evidence_output/thermal-snapshot.txt" 2> "$evidence_output/thermal-snapshot.stderr"; then
    evidence_record_status thermal_snapshot 0
else
    evidence_record_status thermal_snapshot "$?"
fi

evidence_capture_command rt_affinity "$evidence_script_dir/rt-affinity.sh" snapshot \
    --profile "$evidence_profile" --sys-root "$evidence_sys_root" --proc-root "$evidence_proc_root" \
    --can-interface "$evidence_can_interface"
evidence_capture_command can_contract "$evidence_script_dir/can-link.sh" plan
evidence_capture_command can_status ip -details -statistics link show dev "$evidence_can_interface"
evidence_capture_command can_check "$evidence_script_dir/can-link.sh" check
evidence_capture_command network_links ip -details -statistics link show
evidence_capture_command network_routes ip route show table all
[ -z "$evidence_network_peer" ] || evidence_capture_command network_peer_route ip route get "$evidence_network_peer"
if [ -n "$evidence_network_interface" ]; then
    evidence_capture_command network_interface ip -details -statistics link show dev "$evidence_network_interface"
    if command -v ethtool >/dev/null 2>&1; then
        evidence_capture_command network_ethtool ethtool "$evidence_network_interface"
        evidence_capture_command network_driver ethtool -i "$evidence_network_interface"
    else
        evidence_record_status network_ethtool 127
        evidence_record_status network_driver 127
    fi
fi
evidence_capture_command ip_version ip -Version
evidence_capture_command python_version python3 --version
if command -v cyclictest >/dev/null 2>&1; then evidence_capture_command cyclictest_version cyclictest --version; else evidence_record_status cyclictest_version 127; fi
if command -v stress-ng >/dev/null 2>&1; then evidence_capture_command stress_ng_version stress-ng --version; else evidence_record_status stress_ng_version 127; fi
if command -v iperf3 >/dev/null 2>&1; then evidence_capture_command iperf3_version iperf3 --version; else evidence_record_status iperf3_version 127; fi

(
    cd "$evidence_output"
    find . -type f ! -path './SHA256SUMS' -print | LC_ALL=C sort | while IFS= read -r evidence_file; do
        sha256sum "$evidence_file"
    done > SHA256SUMS
)

printf 'evidence collected at %s\n' "$evidence_output"
