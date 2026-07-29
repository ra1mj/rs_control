#!/bin/sh
set -eu

test_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test_platform_root=$(CDPATH= cd -- "$test_script_dir/.." && pwd)
test_tmp_parent=$(CDPATH= cd -- "${TMPDIR:-/tmp}" && pwd)
test_root=$(mktemp -d "$test_tmp_parent/rs-control-platform-test.XXXXXX")
test_log=$test_root/last-command.log
test_count=0

test_cleanup() {
    case "$test_root" in
        "$test_tmp_parent"/rs-control-platform-test.*) rm -rf -- "$test_root" ;;
        *) printf 'refusing to remove unexpected test path: %s\n' "$test_root" >&2 ;;
    esac
}
trap test_cleanup EXIT INT TERM HUP

test_pass() {
    test_count=$((test_count + 1))
    printf 'ok %s - %s\n' "$test_count" "$1"
}

test_fail() {
    printf 'not ok %s - %s\n' "$((test_count + 1))" "$1" >&2
    [ ! -s "$test_log" ] || sed 's/^/  /' "$test_log" >&2
    exit 1
}

test_run() {
    test_name=$1
    shift
    if "$@" > "$test_log" 2>&1; then
        test_pass "$test_name"
    else
        test_fail "$test_name"
    fi
}

test_expect_status() {
    test_expected=$1
    test_name=$2
    shift 2
    set +e
    "$@" > "$test_log" 2>&1
    test_actual=$?
    set -e
    if [ "$test_actual" -eq "$test_expected" ]; then
        test_pass "$test_name"
    else
        printf 'expected status %s, got %s\n' "$test_expected" "$test_actual" >> "$test_log"
        test_fail "$test_name"
    fi
}

test_assert_contains() {
    test_name=$1
    test_file=$2
    test_text=$3
    if grep -F -- "$test_text" "$test_file" >/dev/null 2>&1; then
        test_pass "$test_name"
    else
        printf 'missing text: %s\nfile: %s\n' "$test_text" "$test_file" > "$test_log"
        test_fail "$test_name"
    fi
}

test_assert_not_contains() {
    test_name=$1
    test_file=$2
    test_text=$3
    if grep -F -- "$test_text" "$test_file" >/dev/null 2>&1; then
        printf 'unexpected text: %s\nfile: %s\n' "$test_text" "$test_file" > "$test_log"
        test_fail "$test_name"
    else
        test_pass "$test_name"
    fi
}

test_assert_exists() {
    if [ -e "$2" ]; then test_pass "$1"; else printf 'missing path: %s\n' "$2" > "$test_log"; test_fail "$1"; fi
}

test_assert_absent() {
    if [ ! -e "$2" ]; then test_pass "$1"; else printf 'unexpected path: %s\n' "$2" > "$test_log"; test_fail "$1"; fi
}

test_sys=$test_root/sys
test_proc=$test_root/proc
test_etc=$test_root/etc
test_bin=$test_root/bin
test_markers=$test_root/markers
test_storage=$test_root/storage
mkdir -p "$test_sys/devices/system/cpu" "$test_sys/kernel" "$test_sys/class/net/can0/device" \
    "$test_sys/class/thermal/thermal_zone0" "$test_proc/self" "$test_proc/sys/kernel" \
    "$test_proc/irq/42" "$test_proc/device-tree" "$test_etc" "$test_bin" "$test_markers" "$test_storage"

printf '0-7\n' > "$test_sys/devices/system/cpu/online"
printf '1\n' > "$test_sys/kernel/realtime"
printf '0-7\n' > "$test_proc/self/status.tmp"
printf 'Name:\thost-test\nCpus_allowed_list:\t0-7\n' > "$test_proc/self/status"
printf '950000\n' > "$test_proc/sys/kernel/sched_rt_runtime_us"
printf '1000000\n' > "$test_proc/sys/kernel/sched_rt_period_us"
printf '6\n' > "$test_proc/irq/42/smp_affinity_list"
printf '42\n' > "$test_sys/class/net/can0/device/irq"
printf 'rockchip-thermal\n' > "$test_sys/class/thermal/thermal_zone0/type"
printf '45000\n' > "$test_sys/class/thermal/thermal_zone0/temp"
printf 'passive\n' > "$test_sys/class/thermal/thermal_zone0/trip_point_0_type"
printf '85000\n' > "$test_sys/class/thermal/thermal_zone0/trip_point_0_temp"
printf 'quiet isolcpus=7\n' > "$test_proc/cmdline"
printf 'Linux version 5.10.160-rt89\n' > "$test_proc/version"
: > "$test_proc/interrupts"
printf 'Orange Pi 5 Plus\000' > "$test_proc/device-tree/model"
printf 'xunlong,orangepi-5-plus\000rockchip,rk3588\000' > "$test_proc/device-tree/compatible"
printf 'NAME=Ubuntu\nVERSION_ID=22.04\n' > "$test_etc/os-release"
: > "$test_sys/devices/system/cpu/isolated"
: > "$test_sys/devices/system/cpu/nohz_full"

for test_cpu in 0 1 2 3 4 5 6 7; do
    mkdir -p "$test_sys/devices/system/cpu/cpu$test_cpu/topology"
    case "$test_cpu" in 0|1|2|3) test_capacity=512 ;; *) test_capacity=1024 ;; esac
    printf '%s\n' "$test_capacity" > "$test_sys/devices/system/cpu/cpu$test_cpu/cpu_capacity"
    printf '%s\n' "$test_cpu" > "$test_sys/devices/system/cpu/cpu$test_cpu/topology/core_id"
    printf '0\n' > "$test_sys/devices/system/cpu/cpu$test_cpu/topology/physical_package_id"
done
for test_cpu in 6 7; do
    mkdir -p "$test_sys/devices/system/cpu/cpu$test_cpu/cpufreq"
    printf 'performance\n' > "$test_sys/devices/system/cpu/cpu$test_cpu/cpufreq/scaling_governor"
    printf '2000000\n' > "$test_sys/devices/system/cpu/cpu$test_cpu/cpufreq/scaling_cur_freq"
    printf '2000000\n' > "$test_sys/devices/system/cpu/cpu$test_cpu/cpufreq/scaling_max_freq"
    printf 'cpufreq-dt\n' > "$test_sys/devices/system/cpu/cpu$test_cpu/cpufreq/scaling_driver"
done

test_profile=$test_platform_root/config/rt-profile.env
test_bad_profile=$test_root/overlap.env
sed 's/HOUSEKEEPING_CPUS=0-5/HOUSEKEEPING_CPUS=0-6/' "$test_profile" > "$test_bad_profile"
test_unknown_profile=$test_root/unknown-key.env
cp "$test_profile" "$test_unknown_profile"
printf 'UNEXPECTED_KEY=1\n' >> "$test_unknown_profile"

test_hardware=$test_root/hardware.env
cat > "$test_hardware" <<'EOF'
HARDWARE_EVIDENCE_FORMAT_VERSION=1
BOARD_MODEL=OrangePi5Plus
BOARD_REVISION=v1.0
POWER_SUPPLY=stable_5v4a
ACTIVE_COOLER=installed
CAN_TRANSCEIVER=isolated_3v3
CAN_ISOLATION_RATING=verified
CAN_TERMINATION=two_120ohm_ends
SERIAL_CONSOLE_VERIFIED=1
VENDOR_KERNEL_FALLBACK=boot_verified
EOF
test_overlay=$test_root/rk3588-can0-m0.dtbo
printf 'fixture-dtbo\n' > "$test_overlay"

cat > "$test_bin/cyclictest" <<'SCRIPT'
#!/bin/sh
set -eu
if [ "${1:-}" = --version ]; then printf 'cyclictest V 2.10\n'; exit 0; fi
fake_json=
fake_cpu=7
for fake_arg in "$@"; do
    case "$fake_arg" in
        --json=*) fake_json=${fake_arg#--json=} ;;
        --affinity=*) fake_cpu=${fake_arg#--affinity=} ;;
    esac
done
[ -n "$fake_json" ] || exit 2
cat > "$fake_json" <<EOF
{"return_code":0,"num_threads":1,"resolution_in_ns":1,"thread":{"0":{"cycles":${RS_FAKE_CYCLES:-1000},"min":100,"max":${RS_FAKE_MAX_NS:-100000},"avg":200,"cpu":$fake_cpu,"node":-1}}}
EOF
SCRIPT
cat > "$test_bin/taskset" <<'SCRIPT'
#!/bin/sh
set -eu
[ "${1:-}" = --cpu-list ] || exit 2
shift 2
exec "$@"
SCRIPT
cat > "$test_bin/stress-ng" <<'SCRIPT'
#!/bin/sh
set -eu
if [ "${1:-}" = --version ]; then printf 'stress-ng 0.fixture\n'; exit 0; fi
: > "$RS_TEST_MARKER_DIR/stress-ng-ran"
sleep 1
SCRIPT
cat > "$test_bin/iperf3" <<'SCRIPT'
#!/bin/sh
set -eu
if [ "${1:-}" = --version ]; then printf 'iperf 3.fixture\n'; exit 0; fi
: > "$RS_TEST_MARKER_DIR/iperf3-ran"
sleep 1
SCRIPT
cat > "$test_bin/ip" <<'SCRIPT'
#!/bin/sh
set -eu
case "$*" in
    '-Version') printf 'ip utility, fixture\n' ;;
    *'route get test-peer'*) printf 'test-peer via 192.0.2.1 dev %s src 192.0.2.2\n' "${RS_FAKE_ROUTE_INTERFACE:-eth0}" ;;
    *'-details -statistics link show dev can0'*)
        printf '2: can0: <NOARP,UP,LOWER_UP> mtu 16 state UP\n'
        printf '    can state ERROR-ACTIVE berr-reporting on restart-ms 0\n'
        printf '    bitrate 1000000 sample-point 0.750\n'
        ;;
    *'link show dev can0'*) printf '2: can0: <NOARP,UP,LOWER_UP> mtu 16 state UP\n' ;;
    *'link show dev eth0'*) printf '3: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 state UP\n' ;;
    *) printf 'fixture ip output\n' ;;
esac
SCRIPT
cat > "$test_bin/ethtool" <<'SCRIPT'
#!/bin/sh
set -eu
if [ "${1:-}" = -i ]; then
    printf 'driver: rk_gmac-dwmac\nversion: fixture\n'
else
    printf 'Settings for %s:\n\tSpeed: 1000Mb/s\n\tLink detected: yes\n' "${1:-eth0}"
fi
SCRIPT
test_accelerator=$test_root/accelerator-runner
cat > "$test_accelerator" <<'SCRIPT'
#!/bin/sh
set -eu
: > "$RS_TEST_MARKER_DIR/accelerator-ran"
[ "${RS_FAKE_THERMAL_EVENT:-0}" != 1 ] || : > "$RS_QUALIFICATION_THERMAL_EVENT_FILE"
sleep 1
SCRIPT
chmod +x "$test_bin"/* "$test_accelerator"

test_run 'all platform shell scripts parse' sh -c '
    for candidate in "$1"/lib/*.sh "$1"/kernel/*.sh "$1"/scripts/*.sh "$1"/tests/*.sh; do
        sh -n "$candidate" || exit 1
    done
' sh "$test_platform_root"
test_run 'pinned kernel inputs verify without fetch' "$test_platform_root/kernel/build-kernel.sh" --verify-only
test_run 'pinned kernel build plan is printable' "$test_platform_root/kernel/build-kernel.sh" --print-plan
test_assert_contains 'kernel plan keeps thermal protection' "$test_log" 'configuration=verified-vendor-config + rs-control-rt.fragment + olddefconfig'

test_run 'exact CPU role plan validates' "$test_platform_root/scripts/rt-affinity.sh" plan --profile "$test_profile"
test_assert_contains 'control role remains exact CPU 7' "$test_log" 'control_cpus=7'
test_assert_contains 'affinity semantics are explicit' "$test_log" 'semantics=exact-linux-logical-cpu-sets'
test_run 'fixture RT target and CAN IRQ readback pass' "$test_platform_root/scripts/rt-affinity.sh" check \
    --profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc"
test_expect_status 1 'overlapping CPU roles are rejected' "$test_platform_root/scripts/rt-affinity.sh" plan --profile "$test_bad_profile"
test_expect_status 1 'unexpected profile keys are rejected before sourcing' "$test_platform_root/scripts/rt-affinity.sh" plan --profile "$test_unknown_profile"
test_expect_status 1 'affinity apply requires an explicit disarmed acknowledgement' "$test_platform_root/scripts/rt-affinity.sh" apply \
    --profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc"

printf '900000\n' > "$test_proc/sys/kernel/sched_rt_runtime_us"
printf 'schedutil\n' > "$test_sys/devices/system/cpu/cpu6/cpufreq/scaling_governor"
printf 'schedutil\n' > "$test_sys/devices/system/cpu/cpu7/cpufreq/scaling_governor"
printf '0,1\n' > "$test_proc/irq/42/smp_affinity_list"
test_run 'fixture apply updates and verifies the complete profile' "$test_platform_root/scripts/rt-affinity.sh" apply \
    --profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --confirm-disarmed --fixture-nonroot
test_assert_contains 'fixture apply sets the scheduler budget' "$test_proc/sys/kernel/sched_rt_runtime_us" '950000'
test_assert_contains 'fixture apply sets exact CAN IRQ affinity' "$test_proc/irq/42/smp_affinity_list" '6'

printf '900000\n' > "$test_proc/sys/kernel/sched_rt_runtime_us"
printf 'schedutil\n' > "$test_sys/devices/system/cpu/cpu6/cpufreq/scaling_governor"
printf 'schedutil\n' > "$test_sys/devices/system/cpu/cpu7/cpufreq/scaling_governor"
printf '0,1\n' > "$test_proc/irq/42/smp_affinity_list"
test_expect_status 1 'failed fixture apply returns non-zero after rollback' "$test_platform_root/scripts/rt-affinity.sh" apply \
    --profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --confirm-disarmed --fixture-nonroot --fixture-fail-after-governors
test_assert_contains 'failed apply restores the scheduler budget' "$test_proc/sys/kernel/sched_rt_runtime_us" '900000'
test_assert_contains 'failed apply restores the first governor' \
    "$test_sys/devices/system/cpu/cpu6/cpufreq/scaling_governor" 'schedutil'
printf '950000\n' > "$test_proc/sys/kernel/sched_rt_runtime_us"
printf 'performance\n' > "$test_sys/devices/system/cpu/cpu6/cpufreq/scaling_governor"
printf 'performance\n' > "$test_sys/devices/system/cpu/cpu7/cpufreq/scaling_governor"
printf '6\n' > "$test_proc/irq/42/smp_affinity_list"

test_run 'SocketCAN plan is side-effect-free' "$test_platform_root/scripts/can-link.sh" plan
test_assert_contains 'CAN bitrate is fixed at 1 Mbps' "$test_log" 'bitrate=1000000'
test_assert_contains 'automatic bus-off recovery is disabled' "$test_log" 'restart_ms=0'
test_assert_contains 'CAN plan contains no motor operation' "$test_log" 'motor_operations=none'
test_assert_not_contains 'CAN plan never sends a frame' "$test_log" 'cansend'
test_expect_status 1 'CAN recovery requires disarmed acknowledgement' "$test_platform_root/scripts/can-link.sh" recover --dry-run
test_run 'confirmed CAN recovery can be inspected without execution' "$test_platform_root/scripts/can-link.sh" recover --dry-run --confirm-disarmed
test_run 'effective CAN contract is read back' env PATH="$test_bin:$PATH" "$test_platform_root/scripts/can-link.sh" check

test_pass_json=$test_root/cyclic-pass.json
cat > "$test_pass_json" <<'EOF'
{"return_code":0,"num_threads":1,"resolution_in_ns":1,"thread":{"0":{"cycles":10,"min":1,"max":100000,"avg":2,"cpu":7,"node":-1}}}
EOF
test_fail_json=$test_root/cyclic-fail.json
sed 's/100000/100001/' "$test_pass_json" > "$test_fail_json"
test_bool_json=$test_root/cyclic-bool.json
sed 's/"return_code":0/"return_code":false/' "$test_pass_json" > "$test_bool_json"
test_run '100000 ns cyclictest boundary passes' python3 -B "$test_platform_root/scripts/evaluate-cyclictest.py" \
    "$test_pass_json" --expected-cpu 7
test_assert_contains 'pass verdict preserves maximum latency' "$test_log" 'MAX_LATENCY_NS=100000'
test_expect_status 1 '100001 ns cyclictest boundary fails' python3 -B "$test_platform_root/scripts/evaluate-cyclictest.py" \
    "$test_fail_json" --expected-cpu 7
test_expect_status 2 'unexpected cyclictest CPU is incomplete' python3 -B "$test_platform_root/scripts/evaluate-cyclictest.py" \
    "$test_pass_json" --expected-cpu 6
test_expect_status 2 'boolean return code is not accepted as integer zero' python3 -B "$test_platform_root/scripts/evaluate-cyclictest.py" "$test_bool_json"
test_expect_status 2 'insufficient timing coverage is incomplete' python3 -B "$test_platform_root/scripts/evaluate-cyclictest.py" \
    "$test_pass_json" --minimum-cycles 11

test_run '1 kHz qualification command is explicit' "$test_platform_root/scripts/qualify-rt.sh" \
    --profile 1khz --mode baseline --duration 1s --dry-run
test_assert_contains '1 kHz interval is 1000 us' "$test_log" '--interval=1000'
test_run '500 Hz fallback command is separate' "$test_platform_root/scripts/qualify-rt.sh" \
    --profile 500hz --mode baseline --duration 1s --dry-run
test_assert_contains '500 Hz interval is 2000 us' "$test_log" '--interval=2000'

test_systemd=$test_root/systemd
mkdir -p "$test_systemd"
for test_unit in "$test_platform_root"/systemd/* "$test_platform_root"/examples/rs-control-motion-rt.service; do
    sed 's|^ExecStart=.*|ExecStart=/bin/true|' "$test_unit" > "$test_systemd/$(basename "$test_unit")"
done
test_run 'systemd privilege and ordering units verify' env \
    SYSTEMD_UNIT_PATH="$test_systemd:/usr/lib/systemd/system:/lib/systemd/system" \
    SYSTEMD_GENERATOR_PATH=/dev/null SYSTEMD_ENVIRONMENT_GENERATOR_PATH=/dev/null \
    systemd-analyze verify "$test_systemd"/*.service "$test_systemd"/*.target
test_assert_contains 'production preparation rejects candidate profiles' \
    "$test_platform_root/systemd/rs-control-platform-prepare.service" '--require-qualified'
test_assert_not_contains 'motion example cannot mutate kernel or network state' \
    "$test_platform_root/examples/rs-control-motion-rt.service" 'CAP_SYS_ADMIN'
test_assert_not_contains 'motion example has no network administration capability' \
    "$test_platform_root/examples/rs-control-motion-rt.service" 'CAP_NET_ADMIN'

test_thermal=$test_root/thermal.csv
test_stop=$test_root/thermal.stop
( sleep 2; : > "$test_stop" ) &
test_stopper_pid=$!
test_run 'thermal and frequency fixture can be sampled' "$test_platform_root/scripts/monitor-thermal.sh" \
    --output "$test_thermal" --stop-file "$test_stop" --cpus 6,7 --sys-root "$test_sys"
wait "$test_stopper_pid"
test_assert_absent 'complete thermal fixture has no missing marker' "$test_thermal.missing"
test_assert_absent 'maximum frequency fixture has no throttle marker' "$test_thermal.throttled"
test_assert_contains 'thermal samples retain the passive threshold' "$test_thermal" ',45000,85000'

printf '1800000\n' > "$test_sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq"
test_throttled=$test_root/thermal-throttled.csv
test_throttled_stop=$test_root/thermal-throttled.stop
( sleep 2; : > "$test_throttled_stop" ) &
test_stopper_pid=$!
test_run 'thermal monitor retains a throttled run' "$test_platform_root/scripts/monitor-thermal.sh" \
    --output "$test_throttled" --stop-file "$test_throttled_stop" --cpus 6,7 --sys-root "$test_sys"
wait "$test_stopper_pid"
test_assert_exists 'frequency drop creates a throttle marker' "$test_throttled.throttled"
printf '2000000\n' > "$test_sys/devices/system/cpu/cpu7/cpufreq/scaling_cur_freq"

test_evidence=$test_root/evidence
test_run 'fixture evidence bundle is collected read-only' env PATH="$test_bin:$PATH" \
    "$test_platform_root/scripts/collect-evidence.sh" --output "$test_evidence" --profile "$test_profile" \
    --hardware-manifest "$test_hardware" --overlay-file "$test_overlay" --network-peer test-peer \
    --network-interface eth0 \
    --sys-root "$test_sys" --proc-root "$test_proc" --etc-root "$test_etc" --app-build-id host-test
test_assert_exists 'evidence bundle has a complete checksum manifest' "$test_evidence/SHA256SUMS"
test_assert_exists 'evidence bundle hashes its collection manifest' "$test_evidence/manifest.sha256"
test_assert_exists 'evidence bundle records the source lock hash separately' "$test_evidence/source-lock.sha256"
test_assert_contains 'route to the identified peer is recorded' "$test_evidence/collection-status.env" 'network_peer_route=0'
test_expect_status 1 'non-empty evidence output is never overwritten' env PATH="$test_bin:$PATH" \
    "$test_platform_root/scripts/collect-evidence.sh" --output "$test_evidence"

test_baseline=$test_root/qualification-baseline
test_run 'baseline qualification closes the fixture evidence loop' env PATH="$test_bin:$PATH" RS_TEST_MARKER_DIR="$test_markers" \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode baseline --duration 1s \
    --output "$test_baseline" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --hardware-manifest "$test_hardware" --overlay-file "$test_overlay" --app-build-id host-test
test_assert_contains 'baseline fixture receives a kernel-platform pass' "$test_baseline/verdict.env" 'QUALIFICATION_STATUS=pass'
test_missing_physical=$test_root/qualification-missing-physical-evidence
test_expect_status 2 'baseline without physical identity remains incomplete' env PATH="$test_bin:$PATH" RS_TEST_MARKER_DIR="$test_markers" \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode baseline --duration 1s \
    --output "$test_missing_physical" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --app-build-id host-test
test_assert_contains 'missing physical identity is explicit in the verdict' \
    "$test_missing_physical/verdict.env" 'missing_hardware_manifest'
test_expect_status 1 'qualification output cannot be reused' env PATH="$test_bin:$PATH" RS_TEST_MARKER_DIR="$test_markers" \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode baseline --duration 1s \
    --output "$test_baseline" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc"

test_concurrent=$test_root/qualification-concurrent
test_expect_status 2 'short concurrent research run remains incomplete' env PATH="$test_bin:$PATH" RS_TEST_MARKER_DIR="$test_markers" \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode concurrent --duration 1s \
    --output "$test_concurrent" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --hardware-manifest "$test_hardware" --overlay-file "$test_overlay" --network-peer test-peer \
    --network-interface eth0 \
    --accelerator-runner "$test_accelerator" --storage-path "$test_storage" --app-build-id host-test
test_assert_exists 'short research run still exercises CPU memory and storage pressure' "$test_markers/stress-ng-ran"
test_assert_exists 'short research run still exercises Ethernet pressure' "$test_markers/iperf3-ran"
test_assert_exists 'short research run still exercises accelerator pressure' "$test_markers/accelerator-ran"
test_assert_contains 'short research verdict names the duration gate' "$test_concurrent/verdict.env" 'duration_below_8h'

test_wrong_route=$test_root/qualification-wrong-ethernet-route
test_expect_status 2 'peer routed outside the selected Ethernet link is incomplete' env PATH="$test_bin:$PATH" \
    RS_TEST_MARKER_DIR="$test_markers" RS_FAKE_ROUTE_INTERFACE=eth1 \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode concurrent --duration 1s \
    --output "$test_wrong_route" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --hardware-manifest "$test_hardware" --overlay-file "$test_overlay" --network-peer test-peer \
    --network-interface eth0 --accelerator-runner "$test_accelerator" --storage-path "$test_storage" --app-build-id host-test
test_assert_contains 'route mismatch is retained in the verdict' \
    "$test_wrong_route/verdict.env" 'network_peer_route_interface_mismatch'

test_thermal_event_run=$test_root/qualification-operator-thermal-event
test_expect_status 1 'operator thermal event overrides a partial-run verdict' env PATH="$test_bin:$PATH" \
    RS_TEST_MARKER_DIR="$test_markers" RS_FAKE_THERMAL_EVENT=1 \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode concurrent --duration 1s \
    --output "$test_thermal_event_run" --rt-profile "$test_profile" --sys-root "$test_sys" --proc-root "$test_proc" \
    --hardware-manifest "$test_hardware" --overlay-file "$test_overlay" --network-peer test-peer \
    --network-interface eth0 \
    --accelerator-runner "$test_accelerator" --storage-path "$test_storage" --app-build-id host-test
test_assert_contains 'operator thermal event is retained as the failure reason' \
    "$test_thermal_event_run/verdict.env" 'QUALIFICATION_REASON=operator_thermal_event'

test_expect_status 1 'unrecorded hardware manifest cannot start a full run' env PATH="$test_bin:$PATH" RS_TEST_MARKER_DIR="$test_markers" \
    "$test_platform_root/scripts/qualify-rt.sh" --profile 1khz --mode concurrent --duration 8h \
    --output "$test_root/invalid-hardware-run" --rt-profile "$test_profile" \
    --hardware-manifest "$test_platform_root/config/hardware-evidence.example.env" --overlay-file "$test_overlay" \
    --network-peer test-peer --network-interface eth0 --accelerator-runner "$test_accelerator" \
    --storage-path "$test_storage" --app-build-id host-test

printf '1..%s\n' "$test_count"
