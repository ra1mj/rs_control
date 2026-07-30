#!/bin/sh
set -eu

mw_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mw_platform_root=$(CDPATH= cd -- "$mw_script_dir/.." && pwd)
. "$mw_platform_root/lib/common.sh"

mw_usage() {
    printf '%s\n' \
        'Usage: qualify-middleware.sh [options]' \
        '  --profile 1khz|500hz --mode baseline|concurrent --duration TIME' \
        '  --output ABSOLUTE_DIR' \
        '  --runner FILE --workspace DIR --app-commit SHA --app-build-id TOKEN' \
        '  --hardware-manifest FILE --overlay-file FILE' \
        '  --horus-source DIR --horus-runner FILE' \
        '  [--capacity N] [--warmup N] [--samples N]' \
        '  [--measurement-cpu N] [--rt-priority N] [--rt-profile FILE]' \
        '  [--network-peer HOST] [--network-interface IFACE]' \
        '  [--accelerator-runner FILE] [--storage-path DIR]' \
        '  [--dry-run]' \
        '' \
        'Actual execution refuses non-aarch64/non-Orange-Pi hosts and never fetches source.'
}

mw_profile=
mw_mode=
mw_duration=
mw_output=
mw_runner=
mw_workspace=
mw_app_commit=
mw_app_build_id=
mw_hardware_manifest=
mw_overlay_file=
mw_horus_source=
mw_horus_runner=
mw_capacity=64
mw_warmup=10000
mw_samples=1000000
mw_measurement_cpu=
mw_rt_priority=80
mw_rt_profile=$mw_platform_root/config/rt-profile.env
mw_network_peer=
mw_network_interface=
mw_accelerator_runner=
mw_storage_path=
mw_sys_root=/sys
mw_proc_root=/proc
mw_dry_run=0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --profile|--mode|--duration|--output|--runner|--workspace|--app-commit|--app-build-id|--hardware-manifest|--overlay-file|--horus-source|--horus-runner|--capacity|--warmup|--samples|--measurement-cpu|--rt-priority|--rt-profile|--network-peer|--network-interface|--accelerator-runner|--storage-path)
            [ "$#" -ge 2 ] || platform_die "$1 requires a value"
            mw_option=$1
            mw_value=$2
            shift
            case "$mw_option" in
                --profile) mw_profile=$mw_value ;;
                --mode) mw_mode=$mw_value ;;
                --duration) mw_duration=$mw_value ;;
                --output) mw_output=$mw_value ;;
                --runner) mw_runner=$mw_value ;;
                --workspace) mw_workspace=$mw_value ;;
                --app-commit) mw_app_commit=$mw_value ;;
                --app-build-id) mw_app_build_id=$mw_value ;;
                --hardware-manifest) mw_hardware_manifest=$mw_value ;;
                --overlay-file) mw_overlay_file=$mw_value ;;
                --horus-source) mw_horus_source=$mw_value ;;
                --horus-runner) mw_horus_runner=$mw_value ;;
                --capacity) mw_capacity=$mw_value ;;
                --warmup) mw_warmup=$mw_value ;;
                --samples) mw_samples=$mw_value ;;
                --measurement-cpu) mw_measurement_cpu=$mw_value ;;
                --rt-priority) mw_rt_priority=$mw_value ;;
                --rt-profile) mw_rt_profile=$mw_value ;;
                --network-peer) mw_network_peer=$mw_value ;;
                --network-interface) mw_network_interface=$mw_value ;;
                --accelerator-runner) mw_accelerator_runner=$mw_value ;;
                --storage-path) mw_storage_path=$mw_value ;;
            esac
            ;;
        --dry-run) mw_dry_run=1 ;;
        -h|--help) mw_usage; exit 0 ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

case "$mw_profile" in 1khz|500hz) ;; *) platform_die '--profile must be 1khz or 500hz' ;; esac
case "$mw_mode" in baseline|concurrent) ;; *) platform_die '--mode must be baseline or concurrent' ;; esac
[ -n "$mw_duration" ] || platform_die '--duration is required'
printf '%s\n' "$mw_duration" | grep -Eq '^[1-9][0-9]*[smhd]?$' || platform_die "invalid duration: $mw_duration"
case "$mw_capacity:$mw_warmup:$mw_samples:$mw_rt_priority" in
    *[!0-9:]*) platform_die 'capacity/warmup/samples/priority must be decimal integers' ;;
esac
[ "$mw_capacity" -ge 3 ] && [ "$mw_capacity" -le 65536 ] || platform_die 'capacity must be in 3..=65536'
[ "$mw_warmup" -le 1000000 ] || platform_die 'warmup exceeds 1000000'
[ "$mw_samples" -ge 1 ] && [ "$mw_samples" -le 10000000 ] || platform_die 'samples must be in 1..=10000000'
[ "$mw_rt_priority" -ge 1 ] && [ "$mw_rt_priority" -le 89 ] || platform_die 'RT priority must be in 1..=89'

if [ "$mw_dry_run" -eq 1 ]; then
    printf 'profile=%s\nmode=%s\nduration=%s\ncapacity=%s\nwarmup=%s\nsamples=%s\n' \
        "$mw_profile" "$mw_mode" "$mw_duration" "$mw_capacity" "$mw_warmup" "$mw_samples"
    printf 'identity_gate=aarch64+xunlong,orangepi-5-plus+rockchip,rk3588\n'
    printf 'memory_policy=lock-required\nsource_fetch=forbidden\n'
    printf 'matrix=local-thread,shm-thread,shm-process,horus_x_latest,ordered_x_64,256,1024\n'
    printf 'platform_envelope=qualify-rt.sh:%s:%s\n' "$mw_mode" "$mw_duration"
    printf 'placement=taskset:CONTROL_CPU+chrt:fifo:%s\n' "$mw_rt_priority"
    exit 0
fi

[ -n "$mw_output" ] || platform_die '--output is required'
[ -n "$mw_runner" ] || platform_die '--runner is required'
[ -n "$mw_workspace" ] || platform_die '--workspace is required'
[ -n "$mw_app_commit" ] || platform_die '--app-commit is required'
[ -n "$mw_app_build_id" ] || platform_die '--app-build-id is required'
[ -n "$mw_hardware_manifest" ] || platform_die '--hardware-manifest is required'
[ -n "$mw_overlay_file" ] || platform_die '--overlay-file is required'
[ -n "$mw_horus_source" ] || platform_die '--horus-source is required'
[ -n "$mw_horus_runner" ] || platform_die '--horus-runner is required'
if [ "$mw_mode" = concurrent ]; then
    [ -n "$mw_network_peer" ] || platform_die '--network-peer is required for concurrent mode'
    [ -n "$mw_network_interface" ] || platform_die '--network-interface is required for concurrent mode'
    [ -n "$mw_accelerator_runner" ] || platform_die '--accelerator-runner is required for concurrent mode'
    [ -n "$mw_storage_path" ] || platform_die '--storage-path is required for concurrent mode'
fi

platform_require_absolute_path '--output' "$mw_output"
platform_require_absolute_path '--runner' "$mw_runner"
platform_require_absolute_path '--workspace' "$mw_workspace"
platform_require_absolute_path '--hardware-manifest' "$mw_hardware_manifest"
platform_require_absolute_path '--overlay-file' "$mw_overlay_file"
platform_require_absolute_path '--horus-source' "$mw_horus_source"
platform_require_absolute_path '--horus-runner' "$mw_horus_runner"
platform_require_absolute_path '--rt-profile' "$mw_rt_profile"
[ -z "$mw_accelerator_runner" ] || platform_require_absolute_path '--accelerator-runner' "$mw_accelerator_runner"
[ -z "$mw_storage_path" ] || platform_require_absolute_path '--storage-path' "$mw_storage_path"
[ ! -e "$mw_output" ] || platform_die "middleware evidence output must be absent: $mw_output"
printf '%s\n' "$mw_app_commit" | grep -Eq '^[0-9a-f]{40}$' || platform_die 'app commit must be a full SHA-1'
printf '%s\n' "$mw_app_build_id" | grep -Eq '^[A-Za-z0-9._:+-]+$' || platform_die 'app build ID must be a data token'

mw_compatible=$mw_proc_root/device-tree/compatible
platform_require_rk3588_target_identity "$(uname -m)" "$mw_compatible"

[ -x "$mw_runner" ] || platform_die 'project runner is not executable'
[ -r "$mw_hardware_manifest" ] || platform_die 'hardware manifest is not readable'
[ -r "$mw_overlay_file" ] || platform_die 'overlay file is not readable'
[ -x "$mw_horus_runner" ] || platform_die 'Horus runner is not executable'
[ -r "$mw_rt_profile" ] || platform_die 'RT profile is not readable'
[ -z "$mw_accelerator_runner" ] || [ -x "$mw_accelerator_runner" ] || platform_die 'accelerator runner is not executable'
[ -z "$mw_storage_path" ] || [ -d "$mw_storage_path" ] || platform_die 'storage path is not a directory'
for mw_command in git taskset chrt cargo python3 sha256sum perf; do platform_require_command "$mw_command"; done
git -C "$mw_workspace" rev-parse --is-inside-work-tree >/dev/null 2>&1 || platform_die 'workspace is not a git worktree'
git -C "$mw_horus_source" rev-parse --is-inside-work-tree >/dev/null 2>&1 || platform_die 'Horus source is not a git worktree'
mw_actual_commit=$(git -C "$mw_workspace" rev-parse HEAD)
[ "$mw_actual_commit" = "$mw_app_commit" ] || platform_die "workspace commit mismatch: $mw_actual_commit"
[ -z "$(git -C "$mw_workspace" status --porcelain)" ] || platform_die 'workspace must be clean for target qualification'
mw_horus_commit=$(awk -F= '$1 == "HORUS_COMMIT" { value=$2; count++ } END { if (count != 1 || value !~ /^[0-9a-f]{40}$/) exit 1; print value }' "$mw_platform_root/sources.lock") || platform_die 'invalid HORUS_COMMIT source lock'
mw_actual_horus_commit=$(git -C "$mw_horus_source" rev-parse HEAD)
[ "$mw_actual_horus_commit" = "$mw_horus_commit" ] || platform_die "Horus commit mismatch: $mw_actual_horus_commit"
[ -z "$(git -C "$mw_horus_source" status --porcelain)" ] || platform_die 'Horus checkout must be clean for target qualification'
mw_hardware_keys='HARDWARE_EVIDENCE_FORMAT_VERSION BOARD_MODEL BOARD_REVISION POWER_SUPPLY ACTIVE_COOLER CAN_TRANSCEIVER CAN_ISOLATION_RATING CAN_TERMINATION SERIAL_CONSOLE_VERIFIED VENDOR_KERNEL_FALLBACK'
platform_load_data_file "$mw_hardware_manifest" "$mw_hardware_keys"
[ "$HARDWARE_EVIDENCE_FORMAT_VERSION" = 1 ] || platform_die 'unsupported hardware evidence format'
[ "$BOARD_MODEL" = OrangePi5Plus ] || platform_die "unexpected board model: $BOARD_MODEL"
[ "$SERIAL_CONSOLE_VERIFIED" = 1 ] || platform_die 'serial console recovery must be verified'
for mw_hardware_value in "$BOARD_REVISION" "$POWER_SUPPLY" "$ACTIVE_COOLER" "$CAN_TRANSCEIVER" \
    "$CAN_ISOLATION_RATING" "$CAN_TERMINATION" "$VENDOR_KERNEL_FALLBACK"; do
    [ "$mw_hardware_value" != unrecorded ] || platform_die 'hardware evidence contains unrecorded values'
done
mw_control_cpus=$(awk -F= '$1 == "CONTROL_CPUS" { value=$2; count++ } END { if (count != 1 || value !~ /^[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*$/) exit 1; print value }' "$mw_rt_profile") || platform_die 'invalid CONTROL_CPUS profile value'
mw_control_cpus=$(platform_normalize_cpu_list "$mw_control_cpus")
[ -n "$mw_measurement_cpu" ] || mw_measurement_cpu=$(printf '%s\n' "$mw_control_cpus" | cut -d, -f1)
case "$mw_measurement_cpu" in ''|*[!0-9]*) platform_die 'measurement CPU must be numeric' ;; esac
platform_cpu_list_contains "$mw_control_cpus" "$mw_measurement_cpu" || platform_die 'measurement CPU is not in CONTROL_CPUS'
"$mw_script_dir/rt-affinity.sh" check --profile "$mw_rt_profile" --sys-root "$mw_sys_root" --proc-root "$mw_proc_root" >/dev/null
perf stat --no-big-num --field-separator ';' \
    --event cycles,instructions,task-clock,context-switches,cpu-migrations,page-faults \
    --output /dev/null -- taskset --cpu-list "$mw_measurement_cpu" chrt --fifo "$mw_rt_priority" "$mw_runner" \
    --backend shm --peer process --kind ordered --payload-bytes 1024 --capacity "$mw_capacity" \
    --memory-policy lock-required --warmup 0 --samples 1 >/dev/null ||
    platform_die 'scheduler/memory-lock/runner preflight failed'

mkdir "$mw_output"
mkdir "$mw_output/results"
cp "$mw_platform_root/sources.lock" "$mw_output/sources.lock"
cp "$mw_rt_profile" "$mw_output/rt-profile.env"
cp "$mw_hardware_manifest" "$mw_output/hardware-evidence.env"
{
    printf 'FORMAT_VERSION=1\nPROFILE=%s\nMODE=%s\nDURATION=%s\n' "$mw_profile" "$mw_mode" "$mw_duration"
    printf 'APP_COMMIT=%s\nAPP_BUILD_ID=%s\n' "$mw_app_commit" "$mw_app_build_id"
    printf 'MEASUREMENT_CPU=%s\nRT_PRIORITY=%s\n' "$mw_measurement_cpu" "$mw_rt_priority"
    printf 'CAPACITY=%s\nWARMUP=%s\nSAMPLES=%s\n' "$mw_capacity" "$mw_warmup" "$mw_samples"
    printf 'HORUS_COMMIT=%s\n' "$mw_horus_commit"
    printf 'RUNNER_SHA256=%s\nHORUS_RUNNER_SHA256=%s\n' "$(platform_sha256 "$mw_runner")" "$(platform_sha256 "$mw_horus_runner")"
    printf 'SOURCES_LOCK_SHA256=%s\nRT_PROFILE_SHA256=%s\n' "$(platform_sha256 "$mw_platform_root/sources.lock")" "$(platform_sha256 "$mw_rt_profile")"
} > "$mw_output/inputs.env"

mw_platform_pid=
mw_stop_platform() { [ -z "$mw_platform_pid" ] || kill "$mw_platform_pid" >/dev/null 2>&1 || true; }
trap mw_stop_platform EXIT INT TERM HUP
mw_run_platform() {
    set -- "$mw_script_dir/qualify-rt.sh" --profile "$mw_profile" --mode "$mw_mode" --duration "$mw_duration" \
        --output "$mw_output/platform" --rt-profile "$mw_rt_profile" --measurement-cpu "$mw_measurement_cpu" \
        --hardware-manifest "$mw_hardware_manifest" --overlay-file "$mw_overlay_file" --app-build-id "$mw_app_build_id" \
        --sys-root "$mw_sys_root" --proc-root "$mw_proc_root"
    [ -z "$mw_network_peer" ] || set -- "$@" --network-peer "$mw_network_peer"
    [ -z "$mw_network_interface" ] || set -- "$@" --network-interface "$mw_network_interface"
    [ -z "$mw_accelerator_runner" ] || set -- "$@" --accelerator-runner "$mw_accelerator_runner"
    [ -z "$mw_storage_path" ] || set -- "$@" --storage-path "$mw_storage_path"
    set +e
    "$@" > "$mw_output/platform.stdout" 2> "$mw_output/platform.stderr"
    mw_exit=$?
    set -e
    printf 'EXIT_CODE=%s\n' "$mw_exit" > "$mw_output/platform-process.env"
    exit "$mw_exit"
}
mw_run_platform &
mw_platform_pid=$!
mw_platform_lost=0
mw_ready_deadline=$(( $(date +%s) + 60 ))
while [ ! -e "$mw_output/platform/thermal.csv" ]; do
    if ! kill -0 "$mw_platform_pid" >/dev/null 2>&1; then
        mw_platform_lost=1
        break
    fi
    if [ "$(date +%s)" -ge "$mw_ready_deadline" ]; then
        mw_platform_lost=1
        mw_stop_platform
        break
    fi
    sleep 1
done

mw_matrix_failed=0
mw_check_platform_live() {
    kill -0 "$mw_platform_pid" >/dev/null 2>&1 || mw_platform_lost=1
}
mw_project_run() {
    mw_backend=$1; mw_peer=$2; mw_kind=$3; mw_payload=$4
    mw_result=$mw_output/results/project-$mw_backend-$mw_peer-$mw_kind-$mw_payload.json
    mw_perf=${mw_result%.json}.perf
    perf stat --no-big-num --field-separator ';' \
        --event cycles,instructions,task-clock,context-switches,cpu-migrations,page-faults \
        --output "$mw_perf" -- taskset --cpu-list "$mw_measurement_cpu" chrt --fifo "$mw_rt_priority" "$mw_runner" \
        --backend "$mw_backend" --peer "$mw_peer" --kind "$mw_kind" --payload-bytes "$mw_payload" \
        --capacity "$mw_capacity" --memory-policy lock-required --warmup "$mw_warmup" --samples "$mw_samples" \
        --output "$mw_result" || mw_matrix_failed=1
    mw_check_platform_live
}
if [ "$mw_platform_lost" -eq 0 ]; then
    for mw_kind in latest ordered; do
        for mw_payload in 64 256 1024; do
            mw_project_run local thread "$mw_kind" "$mw_payload"
            mw_project_run shm thread "$mw_kind" "$mw_payload"
            mw_project_run shm process "$mw_kind" "$mw_payload"
            mw_horus_result=$mw_output/results/horus-$mw_kind-$mw_payload.json
            mw_horus_perf=${mw_horus_result%.json}.perf
            perf stat --no-big-num --field-separator ';' \
                --event cycles,instructions,task-clock,context-switches,cpu-migrations,page-faults \
                --output "$mw_horus_perf" -- taskset --cpu-list "$mw_measurement_cpu" chrt --fifo "$mw_rt_priority" "$mw_horus_runner" benchmark \
                --kind "$mw_kind" --payload-bytes "$mw_payload" --capacity "$mw_capacity" --warmup "$mw_warmup" \
                --samples "$mw_samples" --source-commit "$mw_horus_commit" \
                --output "$mw_horus_result" || mw_matrix_failed=1
            mw_check_platform_live
        done
    done
fi

if [ "$mw_platform_lost" -eq 0 ]; then
    set +e
    taskset --cpu-list "$mw_measurement_cpu" cargo test --offline --locked --manifest-path "$mw_workspace/Cargo.toml" \
        -p middleware-shm --all-features -- --test-threads=1 \
        > "$mw_output/results/project-faults.txt" 2>&1
    mw_project_fault_exit=$?
    set -e
    printf 'EXIT_CODE=%s\n' "$mw_project_fault_exit" > "$mw_output/results/project-faults.env"
    [ "$mw_project_fault_exit" -eq 0 ] || mw_matrix_failed=1
    "$mw_horus_runner" fault-suite --source-commit "$mw_horus_commit" --output "$mw_output/results/horus-faults.json" \
        > "$mw_output/results/horus-faults.txt" 2>&1 || mw_matrix_failed=1
    mw_check_platform_live
fi

set +e
wait "$mw_platform_pid"
mw_platform_exit=$?
set -e
mw_platform_pid=
set +e
python3 -B "$mw_script_dir/evaluate-middleware.py" --results "$mw_output/results" --output "$mw_output/comparison.json" \
    --capacity "$mw_capacity" --warmup "$mw_warmup" --samples "$mw_samples" --expected-arch aarch64 \
    --horus-commit "$mw_horus_commit"
mw_evaluator_exit=$?
set -e

mw_platform_verdict=incomplete
if [ -r "$mw_output/platform/verdict.env" ]; then
    mw_platform_verdict=$(awk -F= '$1 == "QUALIFICATION_STATUS" { value=$2; count++ } END { if (count != 1 || value !~ /^(pass|fail|incomplete)$/) exit 1; print value }' "$mw_output/platform/verdict.env") || mw_platform_verdict=incomplete
fi
mw_status=pass
mw_reason=all_safety_gates_passed
if [ "$mw_matrix_failed" -ne 0 ] || [ "$mw_evaluator_exit" -eq 1 ] || [ "$mw_platform_verdict" = fail ]; then
    mw_status=fail; mw_reason=safety_or_execution_gate_failed
elif [ "$mw_evaluator_exit" -ne 0 ] || [ "$mw_platform_exit" -ne 0 ] || [ "$mw_platform_verdict" != pass ] || [ "$mw_platform_lost" -ne 0 ]; then
    mw_status=incomplete; mw_reason=platform_or_candidate_evidence_incomplete
fi
{
    printf 'QUALIFICATION_STATUS=%s\nQUALIFICATION_REASON=%s\n' "$mw_status" "$mw_reason"
    printf 'QUALIFICATION_SCOPE=rk3588_middleware_candidates\nPLATFORM_STATUS=%s\n' "$mw_platform_verdict"
    printf 'PLATFORM_EXIT_CODE=%s\nEVALUATOR_EXIT_CODE=%s\nMATRIX_EXECUTION_FAILED=%s\n' "$mw_platform_exit" "$mw_evaluator_exit" "$mw_matrix_failed"
    printf 'PLATFORM_ENVELOPE_LOST=%s\n' "$mw_platform_lost"
    printf 'PROFILE_MUTATED=0\n'
} > "$mw_output/verdict.env"
(
    cd "$mw_output"
    find . -type f ! -name SHA256SUMS -print | LC_ALL=C sort | while IFS= read -r mw_file; do sha256sum "$mw_file"; done > SHA256SUMS
)
trap - EXIT INT TERM HUP
case "$mw_status" in pass) exit 0 ;; fail) exit 1 ;; incomplete) exit 2 ;; esac
