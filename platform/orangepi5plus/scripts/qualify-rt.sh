#!/bin/sh
set -eu

qualify_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
qualify_platform_root=$(CDPATH= cd -- "$qualify_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$qualify_platform_root/lib/common.sh"

qualify_usage() {
    printf '%s\n' \
        'Usage: qualify-rt.sh [options]' \
        '' \
        'Required run options:' \
        '  --profile 1khz|500hz' \
        '  --mode baseline|concurrent' \
        '  --duration TIME              cyclictest/stress duration (e.g. 10m, 8h)' \
        '  --output ABSOLUTE_DIR        new immutable evidence directory' \
        '' \
        'Placement/evidence options:' \
        '  --rt-profile FILE            platform profile data file' \
        '  --measurement-cpu N          defaults to first control CPU' \
        '  --main-cpu N                 defaults to first housekeeping CPU' \
        '  --hardware-manifest FILE     filled hardware evidence (required for a pass)' \
        '  --overlay-file FILE          active CAN overlay file (required for a pass)' \
        '  --network-peer HOST          iperf3 peer (required for concurrent)' \
        '  --network-interface IFACE    physical Ethernet interface (required for concurrent)' \
        '  --accelerator-runner FILE    executable NPU/GPU workload (required for concurrent)' \
        '  --storage-path DIR           target storage for stress-ng (required for concurrent)' \
        '  --app-build-id TOKEN         build identity recorded with evidence' \
        '  --sys-root PATH              fixture sysfs root (default: /sys)' \
        '  --proc-root PATH             fixture procfs root (default: /proc)' \
        '  --dry-run                    print commands only' \
        '  --evaluate-json FILE         evaluate fixture JSON only and exit' \
        '  -h, --help                   show this help' \
        '' \
        'A concurrent pass shorter than 8h is marked incomplete. No rate fallback is automatic.'
}

qualify_profile=
qualify_mode=
qualify_duration=
qualify_output=
qualify_rt_profile=$qualify_platform_root/config/rt-profile.env
qualify_measurement_cpu=
qualify_main_cpu=
qualify_hardware_manifest=
qualify_overlay_file=
qualify_network_peer=
qualify_network_interface=
qualify_accelerator_runner=
qualify_storage_path=
qualify_app_build_id=unavailable
qualify_sys_root=/sys
qualify_proc_root=/proc
qualify_dry_run=0
qualify_evaluate_json=
qualify_threshold_ns=100000

while [ "$#" -gt 0 ]; do
    case "$1" in
        --profile|--mode|--duration|--output|--rt-profile|--measurement-cpu|--main-cpu|--hardware-manifest|--overlay-file|--network-peer|--network-interface|--accelerator-runner|--storage-path|--app-build-id|--sys-root|--proc-root|--evaluate-json)
            [ "$#" -ge 2 ] || platform_die "$1 requires a value"
            qualify_option=$1
            qualify_value=$2
            shift
            case "$qualify_option" in
                --profile) qualify_profile=$qualify_value ;;
                --mode) qualify_mode=$qualify_value ;;
                --duration) qualify_duration=$qualify_value ;;
                --output) qualify_output=$qualify_value ;;
                --rt-profile) qualify_rt_profile=$qualify_value ;;
                --measurement-cpu) qualify_measurement_cpu=$qualify_value ;;
                --main-cpu) qualify_main_cpu=$qualify_value ;;
                --hardware-manifest) qualify_hardware_manifest=$qualify_value ;;
                --overlay-file) qualify_overlay_file=$qualify_value ;;
                --network-peer) qualify_network_peer=$qualify_value ;;
                --network-interface) qualify_network_interface=$qualify_value ;;
                --accelerator-runner) qualify_accelerator_runner=$qualify_value ;;
                --storage-path) qualify_storage_path=$qualify_value ;;
                --app-build-id) qualify_app_build_id=$qualify_value ;;
                --sys-root) qualify_sys_root=$qualify_value ;;
                --proc-root) qualify_proc_root=$qualify_value ;;
                --evaluate-json) qualify_evaluate_json=$qualify_value ;;
            esac
            ;;
        --dry-run) qualify_dry_run=1 ;;
        -h|--help)
            qualify_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

if [ -n "$qualify_evaluate_json" ]; then
    exec python3 "$qualify_script_dir/evaluate-cyclictest.py" "$qualify_evaluate_json" --threshold-ns "$qualify_threshold_ns"
fi

case "$qualify_profile" in
    1khz) qualify_interval_us=1000 ;;
    500hz) qualify_interval_us=2000 ;;
    *) platform_die '--profile must be 1khz or 500hz' ;;
esac
case "$qualify_mode" in baseline|concurrent) ;; *) platform_die '--mode must be baseline or concurrent' ;; esac
[ -n "$qualify_duration" ] || platform_die '--duration is required'
printf '%s\n' "$qualify_duration" | grep -Eq '^[1-9][0-9]*[smhd]?$' || platform_die "invalid duration: $qualify_duration"

qualify_duration_seconds() {
    qualify_duration_value=$1
    qualify_duration_suffix=$(printf '%s\n' "$qualify_duration_value" | sed 's/^[0-9]*//')
    qualify_duration_number=$(printf '%s\n' "$qualify_duration_value" | sed 's/[^0-9].*$//')
    case "$qualify_duration_suffix" in
        ''|s) printf '%s\n' "$qualify_duration_number" ;;
        m) printf '%s\n' $((qualify_duration_number * 60)) ;;
        h) printf '%s\n' $((qualify_duration_number * 3600)) ;;
        d) printf '%s\n' $((qualify_duration_number * 86400)) ;;
        *) platform_die "unsupported duration suffix: $qualify_duration_suffix" ;;
    esac
}
qualify_seconds=$(qualify_duration_seconds "$qualify_duration")
qualify_expected_cycles=$((qualify_seconds * 1000000 / qualify_interval_us))
qualify_minimum_cycles=$((qualify_expected_cycles * 95 / 100))
[ "$qualify_minimum_cycles" -gt 0 ] || qualify_minimum_cycles=1

qualify_profile_keys='PROFILE_FORMAT_VERSION PROFILE_NAME PROFILE_QUALIFIED QUALIFICATION_EVIDENCE_SHA256 HOUSEKEEPING_CPUS CAN_IO_CPUS CONTROL_CPUS CAN_IRQ_CPUS RT_GOVERNOR RT_RUNTIME_US RT_PERIOD_US REQUIRE_CONTROL_MAX_CAPACITY REQUIRE_IO_MAX_CAPACITY'
platform_load_data_file "$qualify_rt_profile" "$qualify_profile_keys"
HOUSEKEEPING_CPUS=$(platform_normalize_cpu_list "$HOUSEKEEPING_CPUS")
CONTROL_CPUS=$(platform_normalize_cpu_list "$CONTROL_CPUS")
CAN_IO_CPUS=$(platform_normalize_cpu_list "$CAN_IO_CPUS")

[ -n "$qualify_measurement_cpu" ] || qualify_measurement_cpu=$(printf '%s\n' "$CONTROL_CPUS" | cut -d, -f1)
[ -n "$qualify_main_cpu" ] || qualify_main_cpu=$(printf '%s\n' "$HOUSEKEEPING_CPUS" | cut -d, -f1)
case "$qualify_measurement_cpu:$qualify_main_cpu" in *[!0-9:]*) platform_die 'measurement/main CPU must be numeric' ;; esac
platform_cpu_list_contains "$CONTROL_CPUS" "$qualify_measurement_cpu" || platform_die 'measurement CPU is not in CONTROL_CPUS'
platform_cpu_list_contains "$HOUSEKEEPING_CPUS" "$qualify_main_cpu" || platform_die 'main CPU is not in HOUSEKEEPING_CPUS'

qualify_cyclictest_command="cyclictest --affinity=$qualify_measurement_cpu --mainaffinity=$qualify_main_cpu --interval=$qualify_interval_us --threads=1 --priority=90 --policy=fifo --mlockall --nsecs --quiet --duration=$qualify_duration"

if [ "$qualify_dry_run" -eq 1 ]; then
    printf 'profile=%s\nmode=%s\nduration_seconds=%s\n' "$qualify_profile" "$qualify_mode" "$qualify_seconds"
    printf 'minimum_accepted_cycles=%s\n' "$qualify_minimum_cycles"
    printf '%s --json=<output>/cyclictest.json\n' "$qualify_cyclictest_command"
    if [ "$qualify_mode" = concurrent ]; then
        printf 'taskset --cpu-list %s stress-ng --cpu 0 --vm 2 --vm-bytes 60%% --hdd 2 --timeout %s --temp-path <storage>\n' "$HOUSEKEEPING_CPUS" "$qualify_duration"
        printf 'taskset --cpu-list %s iperf3 --client <peer> --bind-dev <interface> --bidir --time %s\n' "$HOUSEKEEPING_CPUS" "$qualify_seconds"
        printf 'RS_QUALIFICATION_DURATION_SECONDS=%s timeout --foreground --kill-after=10s %ss taskset --cpu-list %s <accelerator-runner>\n' "$qualify_seconds" "$((qualify_seconds + 30))" "$HOUSEKEEPING_CPUS"
    fi
    exit 0
fi

[ -n "$qualify_output" ] || platform_die '--output is required'
platform_require_absolute_path '--output' "$qualify_output"
platform_require_absolute_path '--sys-root' "$qualify_sys_root"
platform_require_absolute_path '--proc-root' "$qualify_proc_root"

platform_require_command python3

qualify_incomplete_reasons=
qualify_add_incomplete() {
    if [ -z "$qualify_incomplete_reasons" ]; then
        qualify_incomplete_reasons=$1
    else
        qualify_incomplete_reasons=$qualify_incomplete_reasons,$1
    fi
}

qualify_cyclictest_available=1
if ! command -v cyclictest >/dev/null 2>&1; then
    qualify_cyclictest_available=0
    qualify_add_incomplete cyclictest_missing
fi
qualify_taskset_available=1
qualify_stress_ng_available=1
qualify_iperf_available=1
qualify_timeout_available=1

qualify_hardware_keys='HARDWARE_EVIDENCE_FORMAT_VERSION BOARD_MODEL BOARD_REVISION POWER_SUPPLY ACTIVE_COOLER CAN_TRANSCEIVER CAN_ISOLATION_RATING CAN_TERMINATION SERIAL_CONSOLE_VERIFIED VENDOR_KERNEL_FALLBACK'
qualify_validate_hardware_manifest() {
    platform_load_data_file "$qualify_hardware_manifest" "$qualify_hardware_keys"
    [ "$HARDWARE_EVIDENCE_FORMAT_VERSION" = 1 ] || platform_die 'unsupported hardware evidence format'
    [ "$BOARD_MODEL" = OrangePi5Plus ] || platform_die "unexpected board model: $BOARD_MODEL"
    [ "$SERIAL_CONSOLE_VERIFIED" = 1 ] || platform_die 'serial console recovery must be verified'
    for qualify_hardware_value in "$BOARD_REVISION" "$POWER_SUPPLY" "$ACTIVE_COOLER" \
        "$CAN_TRANSCEIVER" "$CAN_ISOLATION_RATING" "$CAN_TERMINATION" "$VENDOR_KERNEL_FALLBACK"; do
        [ "$qualify_hardware_value" != unrecorded ] || platform_die 'hardware evidence contains unrecorded values'
    done
}

qualify_can_run_concurrent_loads=0

[ -n "$qualify_hardware_manifest" ] || qualify_add_incomplete missing_hardware_manifest
[ -n "$qualify_overlay_file" ] || qualify_add_incomplete missing_overlay_file
[ "$qualify_app_build_id" != unavailable ] || qualify_add_incomplete missing_app_build_id
if [ -n "$qualify_hardware_manifest" ]; then
    platform_require_absolute_path '--hardware-manifest' "$qualify_hardware_manifest"
    qualify_validate_hardware_manifest
fi
if [ -n "$qualify_overlay_file" ]; then
    platform_require_absolute_path '--overlay-file' "$qualify_overlay_file"
    [ -r "$qualify_overlay_file" ] || platform_die 'overlay file is not readable'
fi

if [ "$qualify_mode" = concurrent ]; then
    if ! command -v taskset >/dev/null 2>&1; then qualify_taskset_available=0; qualify_add_incomplete taskset_missing; fi
    if ! command -v stress-ng >/dev/null 2>&1; then qualify_stress_ng_available=0; qualify_add_incomplete stress_ng_missing; fi
    if ! command -v iperf3 >/dev/null 2>&1; then qualify_iperf_available=0; qualify_add_incomplete iperf3_missing; fi
    if ! command -v timeout >/dev/null 2>&1; then qualify_timeout_available=0; qualify_add_incomplete timeout_missing; fi
    if ! command -v ethtool >/dev/null 2>&1; then qualify_add_incomplete ethtool_missing; fi
    [ -n "$qualify_network_peer" ] || qualify_add_incomplete missing_network_peer
    [ -n "$qualify_network_interface" ] || qualify_add_incomplete missing_network_interface
    [ -n "$qualify_accelerator_runner" ] || qualify_add_incomplete missing_accelerator_runner
    [ -n "$qualify_storage_path" ] || qualify_add_incomplete missing_storage_path
    [ "$qualify_seconds" -ge 28800 ] || qualify_add_incomplete duration_below_8h
    if [ -n "$qualify_network_peer" ]; then
        printf '%s\n' "$qualify_network_peer" | grep -Eq '^[A-Za-z0-9][A-Za-z0-9.:-]*$' ||
            platform_die 'network peer must be a hostname or address token'
    fi
    if [ -n "$qualify_network_interface" ]; then
        printf '%s\n' "$qualify_network_interface" | grep -Eq '^[A-Za-z0-9][A-Za-z0-9_.:-]*$' ||
            platform_die 'network interface must be a link-name token'
    fi
    if [ -n "$qualify_accelerator_runner" ]; then
        platform_require_absolute_path '--accelerator-runner' "$qualify_accelerator_runner"
        [ -x "$qualify_accelerator_runner" ] || platform_die 'accelerator runner is not executable'
    fi
    if [ -n "$qualify_storage_path" ]; then
        platform_require_absolute_path '--storage-path' "$qualify_storage_path"
        [ -d "$qualify_storage_path" ] || platform_die 'storage path is not a directory'
    fi
    if [ -n "$qualify_network_peer" ] && [ -n "$qualify_network_interface" ] &&
        [ -n "$qualify_accelerator_runner" ] && [ -n "$qualify_storage_path" ] &&
        [ "$qualify_cyclictest_available" -eq 1 ] && [ "$qualify_taskset_available" -eq 1 ] &&
        [ "$qualify_stress_ng_available" -eq 1 ] && [ "$qualify_iperf_available" -eq 1 ] &&
        [ "$qualify_timeout_available" -eq 1 ]; then
        qualify_can_run_concurrent_loads=1
    fi
fi

if [ -d "$qualify_output" ] && [ -n "$(find "$qualify_output" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
    platform_die "qualification output must be absent or empty: $qualify_output"
fi
mkdir -p "$qualify_output"

set -- --output "$qualify_output/platform" --profile "$qualify_rt_profile" --can-interface can0 \
    --app-build-id "$qualify_app_build_id" --sys-root "$qualify_sys_root" --proc-root "$qualify_proc_root"
[ -z "$qualify_hardware_manifest" ] || set -- "$@" --hardware-manifest "$qualify_hardware_manifest"
[ -z "$qualify_overlay_file" ] || set -- "$@" --overlay-file "$qualify_overlay_file"
[ -z "$qualify_network_peer" ] || set -- "$@" --network-peer "$qualify_network_peer"
[ -z "$qualify_network_interface" ] || set -- "$@" --network-interface "$qualify_network_interface"
"$qualify_script_dir/collect-evidence.sh" "$@"

qualify_require_evidence_status() {
    qualify_evidence_key=$1
    qualify_evidence_value=$(awk -F= -v key="$qualify_evidence_key" '
        $1 == key { value=$2; count++ }
        END { if (count != 1 || value !~ /^[0-9]+$/) exit 1; print value }
    ' "$qualify_output/platform/collection-status.env") || {
        qualify_add_incomplete "evidence_${qualify_evidence_key}_status_invalid"
        return
    }
    [ "$qualify_evidence_value" -eq 0 ] || qualify_add_incomplete "evidence_${qualify_evidence_key}_missing"
}

for qualify_evidence_key in os_release kernel_cmdline kernel_version kernel_realtime cpu_online interrupts \
    device_tree_model device_tree_compatible cpu_topology thermal_snapshot rt_affinity can_overlay \
    can_contract can_status can_check network_links network_routes cyclictest_version; do
    qualify_require_evidence_status "$qualify_evidence_key"
done
if [ "$qualify_mode" = concurrent ]; then
    for qualify_evidence_key in network_peer_route network_interface network_ethtool network_driver \
        stress_ng_version iperf3_version; do
        qualify_require_evidence_status "$qualify_evidence_key"
    done
    if [ -n "$qualify_network_interface" ] && ! awk -v interface="$qualify_network_interface" '
        { for (field = 1; field < NF; field++) if ($field == "dev" && $(field + 1) == interface) found=1 }
        END { exit !found }
    ' "$qualify_output/platform/network-peer-route.txt"; then
        qualify_add_incomplete network_peer_route_interface_mismatch
    fi
    if ! grep -Fq 'Link detected: yes' "$qualify_output/platform/network-ethtool.txt"; then
        qualify_add_incomplete network_link_not_detected
    fi
    if ! grep -Eq '^driver:[[:space:]]*[^[:space:]]+' "$qualify_output/platform/network-driver.txt"; then
        qualify_add_incomplete network_driver_missing
    fi
fi

if ! "$qualify_script_dir/rt-affinity.sh" check --profile "$qualify_rt_profile" \
    --sys-root "$qualify_sys_root" --proc-root "$qualify_proc_root" > "$qualify_output/rt-affinity-check.txt" 2>&1; then
    qualify_add_incomplete affinity_check_failed
fi

qualify_stress_pids=
qualify_stress_ng_pid=
qualify_iperf_pid=
qualify_accelerator_pid=
qualify_stop_file=$qualify_output/thermal.stop
qualify_operator_event_file=$qualify_output/operator-thermal-event
qualify_cleanup() {
    : > "$qualify_stop_file"
    for qualify_pid in $qualify_stress_pids; do
        kill "$qualify_pid" >/dev/null 2>&1 || true
    done
}
qualify_interrupted() {
    qualify_cleanup
    trap - EXIT INT TERM HUP
    exit 130
}
trap qualify_cleanup EXIT
trap qualify_interrupted INT TERM HUP

"$qualify_script_dir/monitor-thermal.sh" --output "$qualify_output/thermal.csv" \
    --stop-file "$qualify_stop_file" --cpus "$CAN_IO_CPUS,$CONTROL_CPUS" --sys-root "$qualify_sys_root" &
qualify_thermal_pid=$!
qualify_stress_pids=$qualify_thermal_pid

if [ "$qualify_mode" = concurrent ] && [ "$qualify_can_run_concurrent_loads" -eq 1 ]; then
    (
        qualify_load_start=$(date +%s)
        qualify_load_exit=0
        taskset --cpu-list "$HOUSEKEEPING_CPUS" stress-ng --cpu 0 --vm 2 --vm-bytes 60% \
            --hdd 2 --temp-path "$qualify_storage_path" --timeout "$qualify_duration" --metrics-brief \
            > "$qualify_output/stress-ng.txt" 2>&1 || qualify_load_exit=$?
        qualify_load_end=$(date +%s)
        printf 'EXIT_CODE=%s\nELAPSED_SECONDS=%s\n' "$qualify_load_exit" "$((qualify_load_end - qualify_load_start))" \
            > "$qualify_output/stress-ng-status.env"
        exit "$qualify_load_exit"
    ) &
    qualify_stress_ng_pid=$!
    qualify_stress_pids="$qualify_stress_pids $qualify_stress_ng_pid"

    (
        qualify_load_start=$(date +%s)
        qualify_load_exit=0
        taskset --cpu-list "$HOUSEKEEPING_CPUS" iperf3 --client "$qualify_network_peer" \
            --bind-dev "$qualify_network_interface" --bidir \
            --time "$qualify_seconds" > "$qualify_output/iperf3.txt" 2>&1 || qualify_load_exit=$?
        qualify_load_end=$(date +%s)
        printf 'EXIT_CODE=%s\nELAPSED_SECONDS=%s\n' "$qualify_load_exit" "$((qualify_load_end - qualify_load_start))" \
            > "$qualify_output/iperf3-status.env"
        exit "$qualify_load_exit"
    ) &
    qualify_iperf_pid=$!
    qualify_stress_pids="$qualify_stress_pids $qualify_iperf_pid"

    qualify_accelerator_timeout=$((qualify_seconds + 30))
    (
        qualify_load_start=$(date +%s)
        qualify_load_exit=0
        RS_QUALIFICATION_DURATION_SECONDS=$qualify_seconds \
            RS_QUALIFICATION_THERMAL_EVENT_FILE=$qualify_operator_event_file \
            timeout --foreground --kill-after=10s "${qualify_accelerator_timeout}s" \
            taskset --cpu-list "$HOUSEKEEPING_CPUS" "$qualify_accelerator_runner" \
            > "$qualify_output/accelerator.txt" 2>&1 || qualify_load_exit=$?
        qualify_load_end=$(date +%s)
        printf 'EXIT_CODE=%s\nELAPSED_SECONDS=%s\n' "$qualify_load_exit" "$((qualify_load_end - qualify_load_start))" \
            > "$qualify_output/accelerator-status.env"
        exit "$qualify_load_exit"
    ) &
    qualify_accelerator_pid=$!
    qualify_stress_pids="$qualify_stress_pids $qualify_accelerator_pid"
fi

sleep 2
if [ "$qualify_cyclictest_available" -eq 1 ]; then
    set +e
    cyclictest --affinity="$qualify_measurement_cpu" --mainaffinity="$qualify_main_cpu" \
        --interval="$qualify_interval_us" --threads=1 --priority=90 --policy=fifo \
        --mlockall --nsecs --quiet --duration="$qualify_duration" \
        --json="$qualify_output/cyclictest.json" > "$qualify_output/cyclictest.stdout" 2>&1
    qualify_cyclictest_exit=$?
    set -e
else
    qualify_cyclictest_exit=127
    printf '%s\n' 'cyclictest unavailable' > "$qualify_output/cyclictest.stdout"
fi

if [ "$qualify_cyclictest_exit" -ne 0 ] && [ "$qualify_can_run_concurrent_loads" -eq 1 ]; then
    kill "$qualify_stress_ng_pid" "$qualify_iperf_pid" "$qualify_accelerator_pid" >/dev/null 2>&1 || true
fi

: > "$qualify_stop_file"
wait "$qualify_thermal_pid" || qualify_add_incomplete thermal_monitor_failed

if [ "$qualify_mode" = concurrent ] && [ "$qualify_can_run_concurrent_loads" -eq 1 ]; then
    wait "$qualify_stress_ng_pid" || qualify_add_incomplete stress_ng_failed
    wait "$qualify_iperf_pid" || qualify_add_incomplete iperf3_failed
    wait "$qualify_accelerator_pid" || qualify_add_incomplete accelerator_failed
    qualify_minimum_load_seconds=$((qualify_seconds * 95 / 100))
    [ "$qualify_minimum_load_seconds" -gt 0 ] || qualify_minimum_load_seconds=1
    for qualify_load_name in stress-ng iperf3 accelerator; do
        qualify_load_status=$qualify_output/$qualify_load_name-status.env
        if [ ! -r "$qualify_load_status" ]; then
            qualify_add_incomplete "${qualify_load_name}_status_missing"
            continue
        fi
        qualify_load_elapsed=$(sed -n 's/^ELAPSED_SECONDS=//p' "$qualify_load_status")
        case "$qualify_load_elapsed" in
            ''|*[!0-9]*) qualify_add_incomplete "${qualify_load_name}_elapsed_invalid" ;;
            *) [ "$qualify_load_elapsed" -ge "$qualify_minimum_load_seconds" ] || qualify_add_incomplete "${qualify_load_name}_duration_short" ;;
        esac
    done
fi

if [ "$qualify_cyclictest_exit" -ne 0 ]; then
    qualify_add_incomplete cyclictest_process_failed
fi

set +e
python3 "$qualify_script_dir/evaluate-cyclictest.py" "$qualify_output/cyclictest.json" \
    --threshold-ns "$qualify_threshold_ns" --expected-cpu "$qualify_measurement_cpu" \
    --minimum-cycles "$qualify_minimum_cycles" \
    > "$qualify_output/cyclictest-verdict.env"
qualify_evaluator_exit=$?
set -e

qualify_status=pass
qualify_reason=within_threshold
if [ "$qualify_evaluator_exit" -eq 1 ]; then
    qualify_status=fail
    qualify_reason=max_latency_exceeded
elif [ "$qualify_evaluator_exit" -ne 0 ]; then
    qualify_status=incomplete
    qualify_reason=cyclictest_invalid
fi
if [ -e "$qualify_output/thermal.csv.throttled" ]; then
    qualify_status=fail
    qualify_reason=thermal_or_frequency_threshold
fi
if [ -e "$qualify_operator_event_file" ]; then
    qualify_status=fail
    qualify_reason=operator_thermal_event
fi
if [ -e "$qualify_output/thermal.csv.missing" ]; then
    qualify_add_incomplete thermal_evidence_missing
fi
if [ -n "$qualify_incomplete_reasons" ] && [ "$qualify_status" != fail ]; then
    qualify_status=incomplete
    qualify_reason=$qualify_incomplete_reasons
fi

{
    printf 'QUALIFICATION_STATUS=%s\n' "$qualify_status"
    printf 'QUALIFICATION_REASON=%s\n' "$qualify_reason"
    printf 'QUALIFICATION_SCOPE=kernel_platform\n'
    printf 'PROFILE=%s\n' "$qualify_profile"
    printf 'MODE=%s\n' "$qualify_mode"
    printf 'DURATION_SECONDS=%s\n' "$qualify_seconds"
    printf 'INTERVAL_US=%s\n' "$qualify_interval_us"
    printf 'THRESHOLD_NS=%s\n' "$qualify_threshold_ns"
    printf 'MINIMUM_CYCLES=%s\n' "$qualify_minimum_cycles"
    printf 'APPLICATION_DEADLINE_EVIDENCE=pending_motion_rt\n'
} > "$qualify_output/verdict.env"

(
    cd "$qualify_output"
    find . -type f ! -path './SHA256SUMS' -print | LC_ALL=C sort | while IFS= read -r qualify_file; do
        sha256sum "$qualify_file"
    done > SHA256SUMS
)

trap - EXIT INT TERM HUP
case "$qualify_status" in
    pass) exit 0 ;;
    fail) exit 1 ;;
    incomplete) exit 2 ;;
esac
