#!/usr/bin/env bash
set -euo pipefail

output_dir="${1:-}"
cgroup_dir="${2:-/sys/fs/cgroup/rs-control-navigation.slice}"
sample_seconds="${RS_NAVIGATION_EVIDENCE_SAMPLE_SECONDS:-1}"
[[ "${output_dir}" == /* && "${output_dir}" != "/" ]] || {
  echo "usage: collect-host-evidence.sh ABSENT_ABSOLUTE_OUTPUT_DIR [CGROUP_DIR|-] [PID ...]" >&2
  exit 2
}
shift "$(( $# > 0 ? 1 : 0 ))"
shift "$(( $# > 0 ? 1 : 0 ))"
[[ "${cgroup_dir}" == "-" || "${cgroup_dir}" == /* ]] || {
  echo "CGROUP_DIR must be absolute or '-'" >&2
  exit 2
}
[[ "${sample_seconds}" =~ ^[1-9][0-9]*$ && "${sample_seconds}" -le 60 ]] || {
  echo "RS_NAVIGATION_EVIDENCE_SAMPLE_SECONDS must be 1..60" >&2
  exit 2
}
[[ ! -e "${output_dir}" ]] || {
  echo "refusing to overwrite evidence directory: ${output_dir}" >&2
  exit 1
}
mkdir -m 0700 "${output_dir}"

{
  echo "scope=host_software_measurement_only"
  echo "hardware_output=forbidden"
  echo "aarch64_qualification=pending"
  echo "production_slam_qualification=pending"
  echo "whole_robot_qualification=pending"
  echo "collected_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "kernel=$(uname -srvm)"
  echo "cgroup=${cgroup_dir}"
  echo "explicit_pids=$*"
  echo "sample_interval_seconds=${sample_seconds}"
} >"${output_dir}/metadata.txt"

if [[ "${cgroup_dir}" != "-" ]]; then
  for name in memory.current memory.peak memory.stat cpu.stat io.stat pids.current pids.peak cgroup.procs; do
    if [[ -r "${cgroup_dir}/${name}" ]]; then
      cp "${cgroup_dir}/${name}" "${output_dir}/cgroup.${name}"
    fi
  done
fi

declare -A selected=()
select_pid() {
  local pid="$1"
  [[ "${pid}" =~ ^[1-9][0-9]*$ && -d "/proc/${pid}" ]] || return 0
  selected["${pid}"]=1
}

if [[ "${cgroup_dir}" != "-" && -r "${cgroup_dir}/cgroup.procs" ]]; then
  while IFS= read -r pid; do
    select_pid "${pid}"
  done <"${cgroup_dir}/cgroup.procs"
fi
for pid in "$@"; do
  [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || {
    echo "invalid PID: ${pid}" >&2
    exit 2
  }
  select_pid "${pid}"
done

# Include descendants so a ros2 launch PID also captures every Nav2 node.
changed=1
while (( changed )); do
  changed=0
  while read -r pid parent; do
    if [[ -n "${selected[${parent}]:-}" && -z "${selected[${pid}]:-}" ]]; then
      select_pid "${pid}"
      changed=1
    fi
  done < <(ps -e -o pid= -o ppid=)
done

if (( ${#selected[@]} > 0 )); then
  while IFS= read -r pid; do
    process_dir="${output_dir}/process-${pid}"
    mkdir "${process_dir}"
    for name in cmdline comm status stat sched schedstat smaps_rollup; do
      if [[ -r "/proc/${pid}/${name}" ]]; then
        cp "/proc/${pid}/${name}" "${process_dir}/${name}"
      fi
    done
  done < <(printf '%s\n' "${!selected[@]}" | sort -n)
fi

sleep "${sample_seconds}"

if [[ "${cgroup_dir}" != "-" ]]; then
  for name in memory.current memory.peak cpu.stat io.stat pids.current pids.peak; do
    if [[ -r "${cgroup_dir}/${name}" ]]; then
      cp "${cgroup_dir}/${name}" "${output_dir}/cgroup.${name}.after"
    fi
  done
fi
for pid in "${!selected[@]}"; do
  process_dir="${output_dir}/process-${pid}"
  [[ -d "${process_dir}" ]] || continue
  for name in status stat schedstat; do
    if [[ -r "/proc/${pid}/${name}" ]]; then
      cp "/proc/${pid}/${name}" "${process_dir}/${name}.after"
    fi
  done
done

(
  cd "${output_dir}"
  find . -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum >SHA256SUMS
)
echo "evidence=${output_dir}"
