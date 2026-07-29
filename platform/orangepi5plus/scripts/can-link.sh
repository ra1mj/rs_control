#!/bin/sh
set -eu

can_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
can_platform_root=$(CDPATH= cd -- "$can_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$can_platform_root/lib/common.sh"

can_usage() {
    printf '%s\n' \
        'Usage: can-link.sh COMMAND [options]' \
        '' \
        'Commands:' \
        '  plan       Print the fixed SocketCAN contract and intended commands' \
        '  check      Verify the effective link against the fixed contract' \
        '  up         Configure and bring up a down link; idempotent if already correct' \
        '  status     Show detailed link state and counters' \
        '  recover    Explicitly cycle and reconfigure a disarmed link' \
        '  down       Explicitly bring down a disarmed link' \
        '' \
        'Options:' \
        '  --config FILE        Config data file (default: config/can.env)' \
        '  --dry-run            Print commands without changing the link' \
        '  --confirm-disarmed   Required for recover/down and disruptive up' \
        '  -h, --help           Show this help' \
        '' \
        'This tool never sends a CAN frame, enables a motor or auto-recovers bus-off.'
}

can_config_keys='CAN_CONFIG_FORMAT_VERSION CAN_INTERFACE CAN_BITRATE CAN_RESTART_MS CAN_BERR_REPORTING CAN_OVERLAY'
can_config=$can_platform_root/config/can.env
can_dry_run=0
can_confirm_disarmed=0

[ "$#" -gt 0 ] || {
    can_usage >&2
    exit 2
}
can_command=$1
shift

while [ "$#" -gt 0 ]; do
    case "$1" in
        --config)
            [ "$#" -ge 2 ] || platform_die '--config requires a value'
            can_config=$2
            shift
            ;;
        --dry-run) can_dry_run=1 ;;
        --confirm-disarmed) can_confirm_disarmed=1 ;;
        -h|--help)
            can_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

case "$can_command" in plan|check|up|status|recover|down) ;; *) platform_die "unknown command: $can_command" ;; esac
platform_load_data_file "$can_config" "$can_config_keys"

[ "$CAN_CONFIG_FORMAT_VERSION" = 1 ] || platform_die "unsupported CAN config format: $CAN_CONFIG_FORMAT_VERSION"
printf '%s\n' "$CAN_INTERFACE" | grep -Eq '^can[0-9]+$' || platform_die "invalid CAN interface: $CAN_INTERFACE"
[ "$CAN_BITRATE" = 1000000 ] || platform_die "Phase B requires classic CAN at 1000000 bit/s"
[ "$CAN_RESTART_MS" = 0 ] || platform_die 'automatic bus-off restart must remain disabled'
[ "$CAN_BERR_REPORTING" = on ] || platform_die 'CAN bus-error reporting must be enabled'
case "$CAN_OVERLAY" in rk3588-can0-m0|rk3588-can1-m0) ;; *) platform_die "unsupported CAN overlay: $CAN_OVERLAY" ;; esac

can_run() {
    if [ "$can_dry_run" -eq 1 ]; then
        printf 'DRY-RUN:'
        for can_arg in "$@"; do
            printf ' %s' "$can_arg"
        done
        printf '\n'
    else
        "$@"
    fi
}

can_print_plan() {
    cat <<EOF
interface=$CAN_INTERFACE
overlay=$CAN_OVERLAY
can_mode=classic
bitrate=$CAN_BITRATE
berr_reporting=$CAN_BERR_REPORTING
restart_ms=$CAN_RESTART_MS
automatic_bus_off_recovery=disabled
motor_operations=none
EOF
    can_run ip link set dev "$CAN_INTERFACE" down
    can_run ip link set dev "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" berr-reporting "$CAN_BERR_REPORTING" restart-ms "$CAN_RESTART_MS"
    can_run ip link set dev "$CAN_INTERFACE" up
}

can_require_ip() {
    platform_require_command ip
    ip link show dev "$CAN_INTERFACE" >/dev/null 2>&1 || platform_die "CAN interface does not exist: $CAN_INTERFACE"
}

can_details() {
    ip -details -statistics link show dev "$CAN_INTERFACE"
}

can_is_up() {
    ip link show dev "$CAN_INTERFACE" | sed -n '1p' | grep -Eq '<[^>]*UP[,>]'
}

can_is_effective() {
    can_effective_details=$(can_details)
    printf '%s\n' "$can_effective_details" | grep -Eq 'can .*berr-reporting on' || return 1
    printf '%s\n' "$can_effective_details" | grep -Eq "bitrate $CAN_BITRATE([[:space:]]|$)" || return 1
    printf '%s\n' "$can_effective_details" | grep -Eq 'restart-ms 0([[:space:]]|$)' || return 1
    can_is_up
}

can_configure() {
    can_run ip link set dev "$CAN_INTERFACE" down
    can_run ip link set dev "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" berr-reporting "$CAN_BERR_REPORTING" restart-ms "$CAN_RESTART_MS"
    can_run ip link set dev "$CAN_INTERFACE" up
}

can_verify() {
    [ "$can_dry_run" -eq 1 ] && return 0
    can_is_effective || platform_die "effective CAN link does not match the fixed contract: $CAN_INTERFACE"
}

case "$can_command" in
    plan)
        can_dry_run=1
        can_print_plan
        ;;
    status)
        can_require_ip
        can_details
        ;;
    check)
        can_require_ip
        can_verify
        printf 'CAN link check: PASS (%s at %s bit/s)\n' "$CAN_INTERFACE" "$CAN_BITRATE"
        ;;
    up)
        [ "$can_dry_run" -eq 1 ] || can_require_ip
        if [ "$can_dry_run" -eq 0 ] && can_is_effective; then
            printf 'CAN link already matches the fixed contract: %s\n' "$CAN_INTERFACE"
            exit 0
        fi
        if [ "$can_dry_run" -eq 0 ] && can_is_up && [ "$can_confirm_disarmed" -ne 1 ]; then
            platform_die 'reconfiguring an active CAN link requires --confirm-disarmed'
        fi
        can_configure
        can_verify
        ;;
    recover)
        [ "$can_confirm_disarmed" -eq 1 ] || platform_die 'recover requires --confirm-disarmed'
        [ "$can_dry_run" -eq 1 ] || can_require_ip
        can_configure
        can_verify
        ;;
    down)
        [ "$can_confirm_disarmed" -eq 1 ] || platform_die 'down requires --confirm-disarmed'
        [ "$can_dry_run" -eq 1 ] || can_require_ip
        can_run ip link set dev "$CAN_INTERFACE" down
        ;;
esac
