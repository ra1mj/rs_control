#!/bin/sh

# Shared, side-effect-free helpers for Orange Pi platform scripts.

platform_error() {
    printf 'error: %s\n' "$*" >&2
}

platform_die() {
    platform_error "$*"
    exit 1
}

platform_require_command() {
    command -v "$1" >/dev/null 2>&1 || platform_die "required command not found: $1"
}

platform_require_absolute_path() {
    case "$2" in
        /*) ;;
        *) platform_die "$1 must be an absolute path: $2" ;;
    esac
}

platform_require_rk3588_target_identity() {
    platform_target_arch=$1
    platform_target_compatible=$2
    [ "$platform_target_arch" = aarch64 ] || platform_die 'actual middleware qualification requires aarch64'
    [ -r "$platform_target_compatible" ] || platform_die 'device-tree compatible identity is unavailable'
    tr '\000' '\n' < "$platform_target_compatible" | grep -Fx 'xunlong,orangepi-5-plus' >/dev/null ||
        platform_die 'target is not Orange Pi 5 Plus'
    tr '\000' '\n' < "$platform_target_compatible" | grep -Fx 'rockchip,rk3588' >/dev/null ||
        platform_die 'target is not RK3588'
}

platform_validate_data_file() {
    platform_data_file=$1
    platform_allowed_keys=$2

    [ -r "$platform_data_file" ] || platform_die "data file is not readable: $platform_data_file"

    awk -F= -v allowed="$platform_allowed_keys" '
        BEGIN {
            count = split(allowed, keys, " ")
            for (i = 1; i <= count; i++) {
                expected[keys[i]] = 1
            }
            failed = 0
        }
        /^[[:space:]]*#/ || /^[[:space:]]*$/ { next }
        {
            if ($0 !~ /^[A-Z][A-Z0-9_]*=[A-Za-z0-9._,:/@+-]+$/) {
                printf "invalid data record at %s:%d: %s\n", FILENAME, NR, $0 > "/dev/stderr"
                failed = 1
                next
            }
            key = $1
            if (!(key in expected)) {
                printf "unexpected key at %s:%d: %s\n", FILENAME, NR, key > "/dev/stderr"
                failed = 1
            }
            if (seen[key]++) {
                printf "duplicate key at %s:%d: %s\n", FILENAME, NR, key > "/dev/stderr"
                failed = 1
            }
        }
        END {
            for (key in expected) {
                if (!seen[key]) {
                    printf "missing key in %s: %s\n", FILENAME, key > "/dev/stderr"
                    failed = 1
                }
            }
            exit failed
        }
    ' "$platform_data_file" || platform_die "invalid data file: $platform_data_file"
}

platform_load_data_file() {
    platform_validate_data_file "$1" "$2"
    # The validator above permits assignments containing data tokens only.
    # shellcheck disable=SC1090
    . "$1"
}

platform_sha256() {
    platform_require_command sha256sum
    sha256sum "$1" | awk '{print $1}'
}

platform_verify_sha256() {
    platform_expected_sha=$1
    platform_sha_file=$2
    platform_actual_sha=$(platform_sha256 "$platform_sha_file")
    [ "$platform_actual_sha" = "$platform_expected_sha" ] ||
        platform_die "SHA-256 mismatch for $platform_sha_file: expected $platform_expected_sha, got $platform_actual_sha"
}

platform_normalize_cpu_list() {
    platform_cpu_spec=$1
    awk -v spec="$platform_cpu_spec" '
        BEGIN {
            if (spec !~ /^[0-9]+(-[0-9]+)?(,[0-9]+(-[0-9]+)?)*$/) {
                exit 2
            }
            parts_count = split(spec, parts, ",")
            max_cpu = -1
            for (i = 1; i <= parts_count; i++) {
                range_count = split(parts[i], range, "-")
                first = range[1] + 0
                last = range_count == 2 ? range[2] + 0 : first
                if (first > last || last > 4095) {
                    exit 2
                }
                for (cpu = first; cpu <= last; cpu++) {
                    present[cpu] = 1
                    if (cpu > max_cpu) max_cpu = cpu
                }
            }
            separator = ""
            for (cpu = 0; cpu <= max_cpu; cpu++) {
                if (present[cpu]) {
                    printf "%s%d", separator, cpu
                    separator = ","
                }
            }
            printf "\n"
        }
    ' || platform_die "invalid CPU list: $platform_cpu_spec"
}

platform_cpu_list_contains() {
    platform_cpu_haystack=$(platform_normalize_cpu_list "$1")
    platform_cpu_needle=$2
    case ",$platform_cpu_haystack," in
        *",$platform_cpu_needle,"*) return 0 ;;
        *) return 1 ;;
    esac
}

platform_cpu_lists_overlap() {
    platform_cpu_left=$(platform_normalize_cpu_list "$1")
    platform_cpu_right=$(platform_normalize_cpu_list "$2")
    platform_old_ifs=$IFS
    IFS=,
    for platform_cpu_item in $platform_cpu_left; do
        if platform_cpu_list_contains "$platform_cpu_right" "$platform_cpu_item"; then
            IFS=$platform_old_ifs
            return 0
        fi
    done
    IFS=$platform_old_ifs
    return 1
}

platform_cpu_list_is_subset() {
    platform_cpu_subset=$(platform_normalize_cpu_list "$1")
    platform_cpu_superset=$(platform_normalize_cpu_list "$2")
    platform_old_ifs=$IFS
    IFS=,
    for platform_cpu_item in $platform_cpu_subset; do
        if ! platform_cpu_list_contains "$platform_cpu_superset" "$platform_cpu_item"; then
            IFS=$platform_old_ifs
            return 1
        fi
    done
    IFS=$platform_old_ifs
    return 0
}
