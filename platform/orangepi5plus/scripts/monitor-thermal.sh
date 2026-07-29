#!/bin/sh
set -eu

thermal_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
thermal_platform_root=$(CDPATH= cd -- "$thermal_script_dir/.." && pwd)

# shellcheck source=../lib/common.sh
. "$thermal_platform_root/lib/common.sh"

thermal_usage() {
    printf '%s\n' \
        'Usage: monitor-thermal.sh --output FILE --stop-file FILE --cpus LIST [--sys-root PATH] [--interval N]' \
        '' \
        'Writes long-form CSV samples. A missing sensor or performance-governor' \
        'frequency drop creates FILE.missing or FILE.throttled respectively.'
}

thermal_output=
thermal_stop_file=
thermal_cpus=
thermal_sys_root=/sys
thermal_interval=1

while [ "$#" -gt 0 ]; do
    case "$1" in
        --output)
            [ "$#" -ge 2 ] || platform_die '--output requires a value'
            thermal_output=$2
            shift
            ;;
        --stop-file)
            [ "$#" -ge 2 ] || platform_die '--stop-file requires a value'
            thermal_stop_file=$2
            shift
            ;;
        --cpus)
            [ "$#" -ge 2 ] || platform_die '--cpus requires a value'
            thermal_cpus=$2
            shift
            ;;
        --sys-root)
            [ "$#" -ge 2 ] || platform_die '--sys-root requires a value'
            thermal_sys_root=$2
            shift
            ;;
        --interval)
            [ "$#" -ge 2 ] || platform_die '--interval requires a value'
            thermal_interval=$2
            shift
            ;;
        -h|--help)
            thermal_usage
            exit 0
            ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

[ -n "$thermal_output" ] || platform_die '--output is required'
[ -n "$thermal_stop_file" ] || platform_die '--stop-file is required'
[ -n "$thermal_cpus" ] || platform_die '--cpus is required'
platform_require_absolute_path '--sys-root' "$thermal_sys_root"
thermal_cpus=$(platform_normalize_cpu_list "$thermal_cpus")
case "$thermal_interval" in ''|*[!0-9]*) platform_die '--interval must be a positive integer' ;; esac
[ "$thermal_interval" -gt 0 ] || platform_die '--interval must be greater than zero'
[ ! -e "$thermal_output" ] || platform_die "thermal output already exists: $thermal_output"

printf '%s\n' 'epoch_s,kind,id,value,limit' > "$thermal_output"

while [ ! -e "$thermal_stop_file" ]; do
    thermal_epoch=$(date +%s)
    thermal_zone_count=0
    for thermal_zone in "$thermal_sys_root"/class/thermal/thermal_zone*; do
        [ -d "$thermal_zone" ] || continue
        thermal_zone_count=$((thermal_zone_count + 1))
        thermal_id=$(basename "$thermal_zone")
        if [ -r "$thermal_zone/type" ] && [ -r "$thermal_zone/temp" ]; then
            thermal_type=$(LC_ALL=C tr -cd 'A-Za-z0-9_.\n-' < "$thermal_zone/type" | sed -n '1p')
            thermal_temp=$(LC_ALL=C tr -cd '0-9\n-' < "$thermal_zone/temp" | sed -n '1p')
            if [ -n "$thermal_type" ] && printf '%s\n' "$thermal_temp" | grep -Eq '^-?[0-9]+$'; then
                thermal_limit=na
                for thermal_trip_type_file in "$thermal_zone"/trip_point_*_type; do
                    [ -r "$thermal_trip_type_file" ] || continue
                    thermal_trip_type=$(tr -d '[:space:]' < "$thermal_trip_type_file")
                    case "$thermal_trip_type" in passive|hot|critical) ;; *) continue ;; esac
                    thermal_trip_temp_file=${thermal_trip_type_file%_type}_temp
                    [ -r "$thermal_trip_temp_file" ] || continue
                    thermal_trip_temp=$(tr -d '[:space:]' < "$thermal_trip_temp_file")
                    printf '%s\n' "$thermal_trip_temp" | grep -Eq '^[0-9]+$' || continue
                    if [ "$thermal_limit" = na ] || [ "$thermal_trip_temp" -lt "$thermal_limit" ]; then
                        thermal_limit=$thermal_trip_temp
                    fi
                done
                printf '%s,temperature,%s-%s,%s,%s\n' "$thermal_epoch" "$thermal_id" "$thermal_type" "$thermal_temp" "$thermal_limit" >> "$thermal_output"
                [ "$thermal_limit" = na ] || [ "$thermal_temp" -lt "$thermal_limit" ] || : > "$thermal_output.throttled"
            else
                : > "$thermal_output.missing"
            fi
        else
            : > "$thermal_output.missing"
        fi
    done
    [ "$thermal_zone_count" -gt 0 ] || : > "$thermal_output.missing"

    thermal_old_ifs=$IFS
    IFS=,
    for thermal_cpu in $thermal_cpus; do
        thermal_cpufreq=$thermal_sys_root/devices/system/cpu/cpu$thermal_cpu/cpufreq
        if [ ! -r "$thermal_cpufreq/scaling_cur_freq" ] || [ ! -r "$thermal_cpufreq/scaling_max_freq" ]; then
            : > "$thermal_output.missing"
            continue
        fi
        thermal_current=$(tr -d '[:space:]' < "$thermal_cpufreq/scaling_cur_freq")
        thermal_maximum=$(tr -d '[:space:]' < "$thermal_cpufreq/scaling_max_freq")
        if printf '%s\n%s\n' "$thermal_current" "$thermal_maximum" | grep -Eqv '^[0-9]+$' ||
            [ "$thermal_maximum" -eq 0 ]; then
            : > "$thermal_output.missing"
        else
            printf '%s,frequency,cpu%s,%s,%s\n' "$thermal_epoch" "$thermal_cpu" "$thermal_current" "$thermal_maximum" >> "$thermal_output"
            [ "$thermal_current" -ge "$thermal_maximum" ] || : > "$thermal_output.throttled"
        fi
    done
    IFS=$thermal_old_ifs
    sleep "$thermal_interval"
done
