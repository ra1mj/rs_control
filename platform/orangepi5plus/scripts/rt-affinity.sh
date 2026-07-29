#!/bin/sh
set -eu

rt_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
rt_platform_root=$(CDPATH= cd -- "$rt_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$rt_platform_root/lib/common.sh"

rt_usage() {
    printf '%s\n' \
        'Usage: rt-affinity.sh COMMAND [options]' \
        '' \
        'Commands:' \
        '  plan       Validate the profile and print its exact intended state' \
        '  check      Verify target prerequisites and effective governor/IRQ state' \
        '  apply      Apply governor, RT throttling and CAN IRQ affinity' \
        '  snapshot   Print current target state without modifying it' \
        '' \
        'Options:' \
        '  --profile FILE          Profile data file (default: config/rt-profile.env)' \
        '  --sys-root PATH         Test/target sysfs root (default: /sys)' \
        '  --proc-root PATH        Test/target procfs root (default: /proc)' \
        '  --can-interface IFACE   CAN interface used to resolve its IRQ (default: can0)' \
        '  --can-irq N             Explicit IRQ when sysfs cannot resolve it' \
        '  --confirm-disarmed      Required for apply' \
        '  --require-qualified     Reject a candidate profile without evidence hash' \
        '  --fixture-nonroot       Test only; allow apply to non-system fixture roots' \
        '  --fixture-fail-after-governors  Test only; inject rollback path' \
        '  -h, --help              Show this help' \
        '' \
        'apply changes host-wide scheduler/governor/IRQ state and never runs implicitly.'
}

rt_profile_keys='PROFILE_FORMAT_VERSION PROFILE_NAME PROFILE_QUALIFIED QUALIFICATION_EVIDENCE_SHA256 HOUSEKEEPING_CPUS CAN_IO_CPUS CONTROL_CPUS CAN_IRQ_CPUS RT_GOVERNOR RT_RUNTIME_US RT_PERIOD_US REQUIRE_CONTROL_MAX_CAPACITY REQUIRE_IO_MAX_CAPACITY'
rt_profile=$rt_platform_root/config/rt-profile.env
rt_sys_root=/sys
rt_proc_root=/proc
rt_can_interface=can0
rt_can_irq=
rt_confirm_disarmed=0
rt_require_qualified=0
rt_fixture_nonroot=0
rt_fixture_fail_after_governors=0
rt_rollback_active=0
rt_original_runtime=
rt_original_period=
rt_original_irq_affinity=
rt_original_governors=

[ "$#" -gt 0 ] || {
    rt_usage >&2
    exit 2
}
rt_command=$1
shift

while [ "$#" -gt 0 ]; do
    case "$1" in
        --profile)
            [ "$#" -ge 2 ] || platform_die '--profile requires a value'
            rt_profile=$2
            shift
            ;;
        --sys-root)
            [ "$#" -ge 2 ] || platform_die '--sys-root requires a value'
            rt_sys_root=$2
            shift
            ;;
        --proc-root)
            [ "$#" -ge 2 ] || platform_die '--proc-root requires a value'
            rt_proc_root=$2
            shift
            ;;
        --can-interface)
            [ "$#" -ge 2 ] || platform_die '--can-interface requires a value'
            rt_can_interface=$2
            shift
            ;;
        --can-irq)
            [ "$#" -ge 2 ] || platform_die '--can-irq requires a value'
            rt_can_irq=$2
            shift
            ;;
        --confirm-disarmed) rt_confirm_disarmed=1 ;;
        --require-qualified) rt_require_qualified=1 ;;
        --fixture-nonroot) rt_fixture_nonroot=1 ;;
        --fixture-fail-after-governors) rt_fixture_fail_after_governors=1 ;;
        -h|--help)
            rt_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

case "$rt_command" in
    plan|check|apply|snapshot) ;;
    *) platform_die "unknown command: $rt_command" ;;
esac
[ "$rt_fixture_fail_after_governors" -eq 0 ] || [ "$rt_command" = apply ] ||
    platform_die '--fixture-fail-after-governors is valid only with apply'
[ "$rt_fixture_nonroot" -eq 0 ] || [ "$rt_command" = apply ] ||
    platform_die '--fixture-nonroot is valid only with apply'
printf '%s\n' "$rt_can_interface" | grep -Eq '^can[0-9]+$' ||
    platform_die "invalid CAN interface: $rt_can_interface"
platform_require_absolute_path '--sys-root' "$rt_sys_root"
platform_require_absolute_path '--proc-root' "$rt_proc_root"
case "$rt_can_irq" in
    ''|*[!0-9]*)
        [ -z "$rt_can_irq" ] || platform_die "invalid CAN IRQ: $rt_can_irq"
        ;;
esac

platform_load_data_file "$rt_profile" "$rt_profile_keys"

rt_validate_scalar_contract() {
    [ "$PROFILE_FORMAT_VERSION" = 1 ] || platform_die "unsupported profile format: $PROFILE_FORMAT_VERSION"
    case "$PROFILE_QUALIFIED" in 0|1) ;; *) platform_die 'PROFILE_QUALIFIED must be 0 or 1' ;; esac
    case "$REQUIRE_CONTROL_MAX_CAPACITY" in 0|1) ;; *) platform_die 'REQUIRE_CONTROL_MAX_CAPACITY must be 0 or 1' ;; esac
    case "$REQUIRE_IO_MAX_CAPACITY" in 0|1) ;; *) platform_die 'REQUIRE_IO_MAX_CAPACITY must be 0 or 1' ;; esac
    [ "$RT_GOVERNOR" = performance ] || platform_die "qualified baseline requires performance governor, got $RT_GOVERNOR"
    for rt_number in "$RT_RUNTIME_US" "$RT_PERIOD_US"; do
        case "$rt_number" in ''|*[!0-9]*) platform_die "invalid RT throttling value: $rt_number" ;; esac
    done
    [ "$RT_RUNTIME_US" -gt 0 ] || platform_die 'RT_RUNTIME_US must keep a positive RT budget'
    [ "$RT_RUNTIME_US" -lt "$RT_PERIOD_US" ] ||
        platform_die 'RT_RUNTIME_US must be lower than RT_PERIOD_US to retain housekeeping recovery time'
    if [ "$PROFILE_QUALIFIED" = 1 ]; then
        printf '%s\n' "$QUALIFICATION_EVIDENCE_SHA256" | grep -Eq '^[0-9a-f]{64}$' ||
            platform_die 'qualified profile requires a 64-character evidence SHA-256'
    else
        [ "$QUALIFICATION_EVIDENCE_SHA256" = unqualified ] ||
            platform_die 'candidate profile evidence marker must be unqualified'
        [ "$rt_require_qualified" -eq 0 ] || platform_die "profile is not qualified: $PROFILE_NAME"
    fi
}

rt_validate_role_contract() {
    HOUSEKEEPING_CPUS=$(platform_normalize_cpu_list "$HOUSEKEEPING_CPUS")
    CAN_IO_CPUS=$(platform_normalize_cpu_list "$CAN_IO_CPUS")
    CONTROL_CPUS=$(platform_normalize_cpu_list "$CONTROL_CPUS")
    CAN_IRQ_CPUS=$(platform_normalize_cpu_list "$CAN_IRQ_CPUS")

    [ -n "$HOUSEKEEPING_CPUS" ] || platform_die 'at least one housekeeping CPU is required'
    [ -n "$CAN_IO_CPUS" ] || platform_die 'at least one CAN I/O CPU is required'
    [ -n "$CONTROL_CPUS" ] || platform_die 'at least one control CPU is required'
    [ -n "$CAN_IRQ_CPUS" ] || platform_die 'at least one CAN IRQ CPU is required'

    platform_cpu_lists_overlap "$HOUSEKEEPING_CPUS" "$CAN_IO_CPUS" &&
        platform_die 'housekeeping and CAN I/O CPUs overlap'
    platform_cpu_lists_overlap "$HOUSEKEEPING_CPUS" "$CONTROL_CPUS" &&
        platform_die 'housekeeping and control CPUs overlap'
    platform_cpu_lists_overlap "$CAN_IO_CPUS" "$CONTROL_CPUS" &&
        platform_die 'CAN I/O and control CPUs overlap'
    platform_cpu_list_is_subset "$CAN_IRQ_CPUS" "$CAN_IO_CPUS" ||
        platform_die 'CAN IRQ CPUs must be a subset of CAN I/O CPUs'
}

rt_read_single_line() {
    [ -r "$1" ] || platform_die "required target file is not readable: $1"
    sed -n '1p' "$1" | tr -d '[:space:]'
}

rt_read_allowed_cpus() {
    [ -r "$rt_proc_root/self/status" ] || platform_die "missing process status: $rt_proc_root/self/status"
    rt_allowed=$(awk '/^Cpus_allowed_list:/ { print $2; found=1; exit } END { if (!found) exit 1 }' "$rt_proc_root/self/status") ||
        platform_die 'Cpus_allowed_list is missing from process status'
    platform_normalize_cpu_list "$rt_allowed"
}

rt_validate_cpu_capacity() {
    rt_capacity_roles=$1
    rt_capacity_required=$2
    [ "$rt_capacity_required" = 1 ] || return 0

    rt_max_capacity=-1
    rt_old_ifs=$IFS
    IFS=,
    for rt_cpu in $rt_online_cpus; do
        rt_capacity_file=$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpu_capacity
        [ -r "$rt_capacity_file" ] || {
            IFS=$rt_old_ifs
            platform_die "missing CPU capacity evidence: $rt_capacity_file"
        }
        rt_capacity=$(rt_read_single_line "$rt_capacity_file")
        case "$rt_capacity" in ''|*[!0-9]*) IFS=$rt_old_ifs; platform_die "invalid CPU capacity: $rt_capacity_file" ;; esac
        [ "$rt_capacity" -le "$rt_max_capacity" ] || rt_max_capacity=$rt_capacity
    done
    for rt_cpu in $rt_capacity_roles; do
        rt_capacity=$(rt_read_single_line "$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpu_capacity")
        [ "$rt_capacity" = "$rt_max_capacity" ] || {
            IFS=$rt_old_ifs
            platform_die "CPU $rt_cpu capacity $rt_capacity is below target maximum $rt_max_capacity"
        }
    done
    IFS=$rt_old_ifs
}

rt_resolve_can_irq() {
    if [ -n "$rt_can_irq" ]; then
        printf '%s\n' "$rt_can_irq"
        return
    fi
    rt_irq_file=$rt_sys_root/class/net/$rt_can_interface/device/irq
    rt_irq=$(rt_read_single_line "$rt_irq_file")
    case "$rt_irq" in ''|*[!0-9]*) platform_die "invalid CAN IRQ in $rt_irq_file: $rt_irq" ;; esac
    printf '%s\n' "$rt_irq"
}

rt_validate_target() {
    rt_online_cpus=$(platform_normalize_cpu_list "$(rt_read_single_line "$rt_sys_root/devices/system/cpu/online")")
    rt_allowed_cpus=$(rt_read_allowed_cpus)

    for rt_role in "$HOUSEKEEPING_CPUS" "$CAN_IO_CPUS" "$CONTROL_CPUS" "$CAN_IRQ_CPUS"; do
        platform_cpu_list_is_subset "$rt_role" "$rt_online_cpus" ||
            platform_die "profile CPU list is outside online CPUs $rt_online_cpus: $rt_role"
        platform_cpu_list_is_subset "$rt_role" "$rt_allowed_cpus" ||
            platform_die "profile CPU list is outside allowed CPUs $rt_allowed_cpus: $rt_role"
    done

    rt_realtime=$(rt_read_single_line "$rt_sys_root/kernel/realtime")
    [ "$rt_realtime" = 1 ] || platform_die 'target kernel does not report PREEMPT_RT active'

    rt_validate_cpu_capacity "$CONTROL_CPUS" "$REQUIRE_CONTROL_MAX_CAPACITY"
    rt_validate_cpu_capacity "$CAN_IO_CPUS" "$REQUIRE_IO_MAX_CAPACITY"

    rt_effective_irq=$(rt_resolve_can_irq)
    [ -d "$rt_proc_root/irq/$rt_effective_irq" ] ||
        platform_die "CAN IRQ is absent from procfs: $rt_effective_irq"
}

rt_governor_cpus() {
    printf '%s\n' "$CAN_IO_CPUS,$CONTROL_CPUS" | tr ',' '\n' | awk '!seen[$0]++' | paste -sd, -
}

rt_verify_effective() {
    rt_actual_runtime=$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_runtime_us")
    rt_actual_period=$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_period_us")
    [ "$rt_actual_runtime" = "$RT_RUNTIME_US" ] ||
        platform_die "sched_rt_runtime_us mismatch: expected $RT_RUNTIME_US, got $rt_actual_runtime"
    [ "$rt_actual_period" = "$RT_PERIOD_US" ] ||
        platform_die "sched_rt_period_us mismatch: expected $RT_PERIOD_US, got $rt_actual_period"

    rt_old_ifs=$IFS
    IFS=,
    for rt_cpu in $(rt_governor_cpus); do
        rt_governor_file=$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpufreq/scaling_governor
        rt_actual_governor=$(rt_read_single_line "$rt_governor_file")
        [ "$rt_actual_governor" = "$RT_GOVERNOR" ] || {
            IFS=$rt_old_ifs
            platform_die "CPU $rt_cpu governor mismatch: expected $RT_GOVERNOR, got $rt_actual_governor"
        }
    done
    IFS=$rt_old_ifs

    rt_effective_irq=$(rt_resolve_can_irq)
    rt_irq_affinity=$(platform_normalize_cpu_list "$(rt_read_single_line "$rt_proc_root/irq/$rt_effective_irq/smp_affinity_list")")
    [ "$rt_irq_affinity" = "$CAN_IRQ_CPUS" ] ||
        platform_die "CAN IRQ affinity mismatch: expected $CAN_IRQ_CPUS, got $rt_irq_affinity"
}

rt_write_scheduler_values() {
    rt_requested_runtime=$1
    rt_requested_period=$2
    rt_runtime_file=$rt_proc_root/sys/kernel/sched_rt_runtime_us
    rt_period_file=$rt_proc_root/sys/kernel/sched_rt_period_us
    rt_current_period=$(rt_read_single_line "$rt_period_file")
    case "$rt_current_period" in ''|*[!0-9]*) platform_die "invalid current RT period: $rt_current_period" ;; esac
    [ "$rt_current_period" -gt 1 ] || platform_die "current RT period is too small: $rt_current_period"

    rt_intermediate_runtime=1
    if [ "$rt_requested_runtime" != -1 ] && [ "$rt_requested_runtime" -lt "$rt_current_period" ]; then
        rt_intermediate_runtime=$rt_requested_runtime
    fi
    printf '%s\n' "$rt_intermediate_runtime" > "$rt_runtime_file"
    printf '%s\n' "$rt_requested_period" > "$rt_period_file"
    printf '%s\n' "$rt_requested_runtime" > "$rt_runtime_file"
}

rt_rollback_apply() {
    rt_original_exit=$?
    [ "$rt_rollback_active" -eq 1 ] || return "$rt_original_exit"
    rt_rollback_failed=0
    rt_rollback_active=0

    rt_write_scheduler_values "$rt_original_runtime" "$rt_original_period" || rt_rollback_failed=1
    rt_old_ifs=$IFS
    IFS=,
    for rt_original_entry in $rt_original_governors; do
        rt_original_cpu=${rt_original_entry%%=*}
        rt_original_governor=${rt_original_entry#*=}
        printf '%s\n' "$rt_original_governor" \
            > "$rt_sys_root/devices/system/cpu/cpu$rt_original_cpu/cpufreq/scaling_governor" || rt_rollback_failed=1
    done
    IFS=$rt_old_ifs
    printf '%s\n' "$rt_original_irq_affinity" \
        > "$rt_proc_root/irq/$rt_effective_irq/smp_affinity_list" || rt_rollback_failed=1
    [ "$rt_rollback_failed" -eq 0 ] || platform_error 'runtime profile rollback was incomplete; keep motion disarmed'
    return "$rt_original_exit"
}

rt_apply_profile() {
    [ "$rt_confirm_disarmed" -eq 1 ] || platform_die 'apply requires --confirm-disarmed'
    if [ "$rt_fixture_nonroot" -eq 1 ]; then
        [ "$rt_sys_root" != /sys ] && [ "$rt_proc_root" != /proc ] ||
            platform_die '--fixture-nonroot refuses real /sys and /proc roots'
    fi
    if [ "$(id -u)" -ne 0 ]; then
        [ "$rt_fixture_nonroot" -eq 1 ] || platform_die 'apply requires root'
    fi
    if [ "$rt_fixture_fail_after_governors" -eq 1 ]; then
        [ "$rt_sys_root" != /sys ] && [ "$rt_proc_root" != /proc ] ||
            platform_die 'fixture failure injection refuses real /sys and /proc roots'
    fi

    rt_original_runtime=$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_runtime_us")
    rt_original_period=$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_period_us")
    printf '%s\n' "$rt_original_runtime" | grep -Eq '^(-1|[0-9]+)$' ||
        platform_die "invalid original RT runtime: $rt_original_runtime"
    printf '%s\n' "$rt_original_period" | grep -Eq '^[0-9]+$' ||
        platform_die "invalid original RT period: $rt_original_period"
    [ "$rt_original_period" -gt 1 ] || platform_die "original RT period is too small: $rt_original_period"

    rt_effective_irq=$(rt_resolve_can_irq)
    rt_original_irq_affinity=$(platform_normalize_cpu_list "$(rt_read_single_line "$rt_proc_root/irq/$rt_effective_irq/smp_affinity_list")")
    rt_original_governors=
    rt_old_ifs=$IFS
    IFS=,
    for rt_cpu in $(rt_governor_cpus); do
        rt_original_governor=$(rt_read_single_line "$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpufreq/scaling_governor")
        [ -n "$rt_original_governor" ] || {
            IFS=$rt_old_ifs
            platform_die "CPU $rt_cpu has an empty original governor"
        }
        [ -z "$rt_original_governors" ] || rt_original_governors=$rt_original_governors,
        rt_original_governors=$rt_original_governors$rt_cpu=$rt_original_governor
    done
    IFS=$rt_old_ifs

    rt_rollback_active=1
    trap rt_rollback_apply EXIT
    trap 'exit 130' INT TERM HUP

    rt_write_scheduler_values "$RT_RUNTIME_US" "$RT_PERIOD_US"

    rt_old_ifs=$IFS
    IFS=,
    for rt_cpu in $(rt_governor_cpus); do
        rt_governor_file=$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpufreq/scaling_governor
        [ -w "$rt_governor_file" ] || {
            IFS=$rt_old_ifs
            platform_die "CPU governor is not writable: $rt_governor_file"
        }
        printf '%s\n' "$RT_GOVERNOR" > "$rt_governor_file"
    done
    IFS=$rt_old_ifs

    [ "$rt_fixture_fail_after_governors" -eq 0 ] || platform_die 'injected fixture failure after governor writes'

    rt_irq_file=$rt_proc_root/irq/$rt_effective_irq/smp_affinity_list
    [ -w "$rt_irq_file" ] || platform_die "CAN IRQ affinity is not writable: $rt_irq_file"
    printf '%s\n' "$CAN_IRQ_CPUS" > "$rt_irq_file"
    rt_verify_effective
    rt_rollback_active=0
    trap - EXIT INT TERM HUP
}

rt_print_plan() {
    cat <<EOF
profile_name=$PROFILE_NAME
profile_qualified=$PROFILE_QUALIFIED
qualification_evidence_sha256=$QUALIFICATION_EVIDENCE_SHA256
housekeeping_cpus=$HOUSEKEEPING_CPUS
can_io_cpus=$CAN_IO_CPUS
control_cpus=$CONTROL_CPUS
can_irq_cpus=$CAN_IRQ_CPUS
rt_governor=$RT_GOVERNOR
sched_rt_runtime_us=$RT_RUNTIME_US
sched_rt_period_us=$RT_PERIOD_US
can_interface=$rt_can_interface
semantics=exact-linux-logical-cpu-sets
automatic_irq_migration=disabled
automatic_rate_fallback=disabled
EOF
}

rt_print_snapshot() {
    printf 'profile_name=%s\n' "$PROFILE_NAME"
    printf 'online_cpus=%s\n' "$(rt_read_single_line "$rt_sys_root/devices/system/cpu/online")"
    printf 'allowed_cpus=%s\n' "$(rt_read_allowed_cpus)"
    printf 'preempt_rt=%s\n' "$(rt_read_single_line "$rt_sys_root/kernel/realtime")"
    printf 'sched_rt_runtime_us=%s\n' "$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_runtime_us")"
    printf 'sched_rt_period_us=%s\n' "$(rt_read_single_line "$rt_proc_root/sys/kernel/sched_rt_period_us")"
    rt_effective_irq=$(rt_resolve_can_irq)
    printf 'can_irq=%s\n' "$rt_effective_irq"
    printf 'can_irq_affinity=%s\n' "$(rt_read_single_line "$rt_proc_root/irq/$rt_effective_irq/smp_affinity_list")"
    rt_old_ifs=$IFS
    IFS=,
    for rt_cpu in $(rt_governor_cpus); do
        printf 'cpu%s_governor=%s\n' "$rt_cpu" "$(rt_read_single_line "$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpufreq/scaling_governor")"
        printf 'cpu%s_capacity=%s\n' "$rt_cpu" "$(rt_read_single_line "$rt_sys_root/devices/system/cpu/cpu$rt_cpu/cpu_capacity")"
    done
    IFS=$rt_old_ifs
}

rt_validate_scalar_contract
rt_validate_role_contract

case "$rt_command" in
    plan)
        rt_print_plan
        ;;
    check)
        rt_validate_target
        rt_verify_effective
        printf 'rt affinity check: PASS (%s)\n' "$PROFILE_NAME"
        ;;
    apply)
        rt_validate_target
        rt_apply_profile
        printf 'rt affinity apply: PASS (%s)\n' "$PROFILE_NAME"
        ;;
    snapshot)
        rt_print_snapshot
        ;;
esac
