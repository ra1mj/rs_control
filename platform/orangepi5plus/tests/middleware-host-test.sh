#!/bin/sh
set -eu

mw_test_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mw_root=$(CDPATH= cd -- "$mw_test_dir/.." && pwd)
mw_tmp_parent=$(CDPATH= cd -- "${TMPDIR:-/tmp}" && pwd)
mw_tmp=$(mktemp -d "$mw_tmp_parent/rs-control-middleware-test.XXXXXX")
mw_count=0
mw_log=$mw_tmp/log
trap 'case "$mw_tmp" in "$mw_tmp_parent"/rs-control-middleware-test.*) rm -rf -- "$mw_tmp" ;; esac' EXIT INT TERM HUP

mw_pass() { mw_count=$((mw_count + 1)); printf 'ok %s - %s\n' "$mw_count" "$1"; }
mw_fail() { printf 'not ok %s - %s\n' "$((mw_count + 1))" "$1" >&2; sed 's/^/  /' "$mw_log" >&2; exit 1; }
mw_run() { mw_name=$1; shift; if "$@" > "$mw_log" 2>&1; then mw_pass "$mw_name"; else mw_fail "$mw_name"; fi; }
mw_expect() {
    mw_expected=$1; mw_name=$2; shift 2
    set +e; "$@" > "$mw_log" 2>&1; mw_actual=$?; set -e
    [ "$mw_actual" -eq "$mw_expected" ] && mw_pass "$mw_name" || mw_fail "$mw_name"
}
mw_contains() { grep -F -- "$3" "$2" >/dev/null 2>&1 && mw_pass "$1" || { printf 'missing: %s in %s\n' "$3" "$2" >> "$mw_log"; mw_fail "$1"; }; }

mw_run 'middleware qualification shell parses' sh -n "$mw_root/scripts/qualify-middleware.sh"
mw_run 'middleware evaluator compiles' env PYTHONPYCACHEPREFIX="$mw_tmp/pycache" python3 -m py_compile "$mw_root/scripts/evaluate-middleware.py"
mw_plan=$mw_tmp/plan
mw_run 'middleware dry-run is side-effect-free' sh -c '"$1" --profile 1khz --mode concurrent --duration 8h --capacity 64 --warmup 2 --samples 4 --dry-run > "$2"' sh "$mw_root/scripts/qualify-middleware.sh" "$mw_plan"
mw_contains 'dry-run pins lock-required mappings' "$mw_plan" 'memory_policy=lock-required'
mw_contains 'dry-run names complete candidate matrix' "$mw_plan" 'local-thread,shm-thread,shm-process,horus'
mw_contains 'dry-run retains eight-hour platform envelope' "$mw_plan" 'platform_envelope=qualify-rt.sh:concurrent:8h'

mw_compatible=$mw_tmp/compatible
printf 'xunlong,orangepi-5-plus\0rockchip,rk3588\0' > "$mw_compatible"
mw_run 'identity validator accepts exact target fixture' sh -c \
    '. "$1"; platform_require_rk3588_target_identity aarch64 "$2"' sh "$mw_root/lib/common.sh" "$mw_compatible"
mw_expect 1 'identity validator refuses wrong architecture' sh -c \
    '. "$1"; platform_require_rk3588_target_identity x86_64 "$2"' sh "$mw_root/lib/common.sh" "$mw_compatible"
printf 'xunlong,orangepi-5\0rockchip,rk3588\0' > "$mw_compatible"
mw_expect 1 'identity validator refuses wrong board device tree' sh -c \
    '. "$1"; platform_require_rk3588_target_identity aarch64 "$2"' sh "$mw_root/lib/common.sh" "$mw_compatible"

mw_expect 1 'middleware wrapper refuses invalid profile' "$mw_root/scripts/qualify-middleware.sh" \
    --profile automatic --mode baseline --duration 1s --dry-run
mw_expect 1 'middleware wrapper refuses invalid duration' "$mw_root/scripts/qualify-middleware.sh" \
    --profile 1khz --mode baseline --duration forever --dry-run
mkdir "$mw_tmp/existing-output"
mw_expect 1 'middleware wrapper refuses existing output before target access' "$mw_root/scripts/qualify-middleware.sh" \
    --profile 1khz --mode baseline --duration 1s --output "$mw_tmp/existing-output" --runner /bin/true \
    --workspace "$mw_tmp" --app-commit 0000000000000000000000000000000000000000 \
    --app-build-id fixture --hardware-manifest /dev/null --overlay-file /dev/null \
    --horus-source "$mw_tmp" --horus-runner /bin/true
mw_expect 1 'middleware wrapper refuses malformed source identity' "$mw_root/scripts/qualify-middleware.sh" \
    --profile 1khz --mode baseline --duration 1s --output "$mw_tmp/new-output" --runner /bin/true \
    --workspace "$mw_tmp" --app-commit not-a-commit --app-build-id fixture \
    --hardware-manifest /dev/null --overlay-file /dev/null --horus-source "$mw_tmp" --horus-runner /bin/true

mw_expect 1 'actual host execution refuses non-aarch64' "$mw_root/scripts/qualify-middleware.sh" \
    --profile 1khz --mode baseline --duration 1s --output "$mw_tmp/actual" --runner /bin/true \
    --workspace "$mw_tmp" --app-commit 0000000000000000000000000000000000000000 \
    --app-build-id fixture --hardware-manifest /dev/null --overlay-file /dev/null \
    --horus-source "$mw_tmp" --horus-runner /bin/true
mw_contains 'host refusal is explicit' "$mw_log" 'actual middleware qualification requires aarch64'

mw_results=$mw_tmp/results
mkdir "$mw_results"
mw_hist='[4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]'
mw_perf_fixture='10;;cycles;1;100.00
20;;instructions;1;100.00
1.00;msec;task-clock;1;100.00
0;;context-switches;1;100.00
0;;cpu-migrations;1;100.00
0;;page-faults;1;100.00'
for mw_backend_peer in local-thread shm-thread shm-process; do
    mw_backend=${mw_backend_peer%-*}
    mw_peer=${mw_backend_peer#*-}
    for mw_kind in latest ordered; do
        for mw_payload in 64 256 1024; do
            cat > "$mw_results/project-$mw_backend-$mw_peer-$mw_kind-$mw_payload.json" <<EOF
{"schema_version":2,"qualification":"host_research","target_os":"linux","target_arch":"aarch64","backend":"$mw_backend","peer":"$mw_peer","peer_execution":"$(if [ "$mw_peer" = process ]; then printf separate_process_paired; else printf separate_thread_paired; fi)","kind":"$mw_kind","payload_bytes":$mw_payload,"capacity":64,"requested_memory_policy":"lock-required","effective_memory_policy":"$(if [ "$mw_backend" = local ]; then printf not-applicable; else printf lock-required; fi)","memory_policy_scope":"shm_mappings","warmup":2,"samples":4,"latency_semantics":"one_way_receive","latency_clock":"CLOCK_MONOTONIC","latency_jitter_semantics":"max_minus_min","latency_log2_ns":$mw_hist,"latency_min_ns":0,"latency_max_ns":0,"latency_jitter_span_ns":0,"operation_semantics":"try_publish_call","operation_clock":"CLOCK_MONOTONIC","operation_log2_ns":$mw_hist,"operation_max_ns":0,"wall_time_ns":1,"cpu_time_ns":1,"peer_cpu_time_ns":$(if [ "$mw_peer" = process ]; then printf 1; else printf 0; fi),"cpu_time_source":"CLOCK_PROCESS_CPUTIME_ID","peak_resident_bytes":1,"peer_peak_resident_bytes":$(if [ "$mw_peer" = process ]; then printf 1; else printf 0; fi),"memory_source":"proc_self_status_VmHWM","allocation_scope":"publisher_and_consumer","allocations":{"count":0,"bytes":0,"current_count":0,"current_bytes":0,"peak_count":0,"peak_bytes":0},"counters":{"published":6,"consumed":6,"overwritten":0,"full":0,"disconnected":0}}
EOF
            printf '%s\n' "$mw_perf_fixture" > "$mw_results/project-$mw_backend-$mw_peer-$mw_kind-$mw_payload.perf"
        done
    done
done
mw_horus_commit=fac3cfb8704e275bde9daec6d554a17c59dd8532
for mw_kind in latest ordered; do
    for mw_payload in 64 256 1024; do
        cat > "$mw_results/horus-$mw_kind-$mw_payload.json" <<EOF
{"schema_version":2,"candidate":"horus","source_commit":"$mw_horus_commit","target_os":"linux","target_arch":"aarch64","peer_execution":"separate_process_paired","effective_memory_policy":"lock-required","kind":"$mw_kind","payload_bytes":$mw_payload,"capacity":64,"warmup":2,"samples":4,"latency_semantics":"one_way_receive","latency_clock":"CLOCK_MONOTONIC","latency_jitter_semantics":"max_minus_min","latency_log2_ns":$mw_hist,"latency_min_ns":0,"latency_max_ns":0,"latency_jitter_span_ns":0,"operation_semantics":"try_publish_call","operation_clock":"CLOCK_MONOTONIC","operation_log2_ns":$mw_hist,"operation_max_ns":0,"wall_time_ns":1,"cpu_time_ns":1,"peer_cpu_time_ns":1,"cpu_time_source":"CLOCK_PROCESS_CPUTIME_ID","peak_resident_bytes":1,"peer_peak_resident_bytes":1,"memory_source":"proc_self_status_VmHWM","allocation_scope":"publisher_and_consumer","allocations":{"count":0,"bytes":0,"current_count":0,"current_bytes":0,"peak_count":0,"peak_bytes":0},"counters":{"published":6,"consumed":6,"overwritten":0,"full":0,"disconnected":0}}
EOF
        printf '%s\n' "$mw_perf_fixture" > "$mw_results/horus-$mw_kind-$mw_payload.perf"
    done
done
printf 'EXIT_CODE=0\n' > "$mw_results/project-faults.env"
printf '%s\n' "{\"schema_version\":1,\"candidate\":\"horus\",\"source_commit\":\"$mw_horus_commit\",\"target_arch\":\"aarch64\",\"producer_bounded\":true,\"no_torn_values\":true,\"crash_reclaim\":true,\"losses_accounted\":true,\"slow_consumer_accounted\":true,\"stopped_consumer_bounded\":true,\"full_queue_explicit\":true,\"clean_exit_reclaim\":true,\"killed_peer_reclaim\":true}" > "$mw_results/horus-faults.json"

mw_report=$mw_tmp/report.json
mw_run 'complete safe matrix evaluates pass' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_results" --output "$mw_report" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"
mw_contains 'passing report is deterministic and safety-first' "$mw_report" '"selection_policy":"safety_gates_then_worst_case_maxima_no_average_ranking"'
mw_report_copy=$mw_tmp/report-copy.json
mw_run 'same complete matrix evaluates a second time' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_results" --output "$mw_report_copy" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"
mw_run 'comparison report bytes are deterministic' cmp "$mw_report" "$mw_report_copy"
mw_expect 2 'comparison output is create-new' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_results" --output "$mw_report" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"

mw_missing=$mw_tmp/missing
cp -R "$mw_results" "$mw_missing"
rm "$mw_missing/horus-latest-64.json"
mw_expect 2 'missing comparison candidate is incomplete' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_missing" --output "$mw_tmp/missing.json" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"

mw_bad=$mw_tmp/bad
cp -R "$mw_results" "$mw_bad"
sed 's/"count":0/"count":1/' "$mw_bad/project-shm-process-latest-64.json" > "$mw_bad/rewrite"
mv "$mw_bad/rewrite" "$mw_bad/project-shm-process-latest-64.json"
mw_expect 1 'nonzero hot allocation fails safety gate' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_bad" --output "$mw_tmp/bad.json" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"
mw_contains 'allocation failure is retained' "$mw_tmp/bad.json" 'hot allocation count is nonzero'

mw_semantic=$mw_tmp/semantic
cp -R "$mw_results" "$mw_semantic"
sed 's/"latency_clock":"CLOCK_MONOTONIC"/"latency_clock":"CLOCK_REALTIME"/' \
    "$mw_semantic/project-local-thread-latest-64.json" > "$mw_semantic/rewrite"
mv "$mw_semantic/rewrite" "$mw_semantic/project-local-thread-latest-64.json"
mw_expect 1 'latency semantic mismatch fails' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_semantic" --output "$mw_tmp/semantic.json" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"

mw_fault=$mw_tmp/fault
cp -R "$mw_results" "$mw_fault"
sed 's/"no_torn_values":true/"no_torn_values":false/' "$mw_fault/horus-faults.json" > "$mw_fault/rewrite"
mv "$mw_fault/rewrite" "$mw_fault/horus-faults.json"
mw_expect 1 'torn-value fault result fails' python3 -B "$mw_root/scripts/evaluate-middleware.py" \
    --results "$mw_fault" --output "$mw_tmp/fault.json" --capacity 64 --warmup 2 --samples 4 \
    --expected-arch aarch64 --horus-commit "$mw_horus_commit"
mw_contains 'fault failure is retained' "$mw_tmp/fault.json" 'horus_fault_no_torn_values_failed'

printf '1..%s\n' "$mw_count"
