#!/bin/sh

# Stop a command launched by `setsid` together with every process that remains
# in its session's process group. The direct child is always reaped. A bounded
# grace period prevents target cleanup from waiting indefinitely on a process
# that ignores TERM.
platform_stop_session() {
    platform_session_pid=${1:-}
    platform_session_grace_seconds=${2:-5}

    case "$platform_session_pid" in
        ''|*[!0-9]*) platform_die "invalid supervised session PID: $platform_session_pid" ;;
    esac
    [ "$platform_session_pid" -gt 1 ] || platform_die "unsafe supervised session PID: $platform_session_pid"
    case "$platform_session_grace_seconds" in
        ''|*[!0-9]*) platform_die "invalid supervised session grace period: $platform_session_grace_seconds" ;;
    esac
    [ "$platform_session_grace_seconds" -le 30 ] ||
        platform_die "supervised session grace period exceeds 30 seconds: $platform_session_grace_seconds"

    platform_session_target=
    if kill -0 "-$platform_session_pid" >/dev/null 2>&1; then
        platform_session_target=-$platform_session_pid
    elif kill -0 "$platform_session_pid" >/dev/null 2>&1; then
        # `setsid` should make PID equal PGID. Retain bounded cleanup if it
        # failed to establish the group but left the direct child running.
        platform_session_target=$platform_session_pid
    fi

    if [ -n "$platform_session_target" ]; then
        kill -TERM "$platform_session_target" >/dev/null 2>&1 || true
        platform_session_elapsed=0
        while kill -0 "$platform_session_target" >/dev/null 2>&1 &&
            [ "$platform_session_elapsed" -lt "$platform_session_grace_seconds" ]; do
            sleep 1
            platform_session_elapsed=$((platform_session_elapsed + 1))
        done
        if kill -0 "$platform_session_target" >/dev/null 2>&1; then
            kill -KILL "$platform_session_target" >/dev/null 2>&1 || true
        fi
    fi
    wait "$platform_session_pid" >/dev/null 2>&1 || true
}
