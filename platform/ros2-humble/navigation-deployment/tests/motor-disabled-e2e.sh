#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
bridge_bin="${2:-}"
harness_bin="${3:-}"
case "${mode}" in
  success|cancel|fault|nav2-fault) ;;
  *) echo "usage: motor-disabled-e2e.sh success|cancel|fault|nav2-fault BRIDGE_BIN HARNESS_BIN" >&2; exit 2 ;;
esac
[[ "${bridge_bin}" == /* && -x "${bridge_bin}" ]] || exit 2
[[ "${harness_bin}" == /* && -x "${harness_bin}" ]] || exit 2

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
rs_control_root="$(cd "${test_dir}/../../../.." && pwd -P)"
manifest="${rs_control_root}/config/navigation/rs_control_navigation_plugin.yaml"
digest="ec8784456ae3bb285d888ad57206af8a36926543e6f559ca65f1c85f8dfad1f7"
runtime_dir="$(mktemp -d --tmpdir rs-navigation-e2e.XXXXXXXX)"
server_pid=""
bridge_pid=""
harness_pid=""
cleanup() {
  for pid in "${bridge_pid}" "${server_pid}" "${harness_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${bridge_pid}" "${server_pid}" "${harness_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
default_domain=$((100 + BASHPID % 100))
export ROS_DOMAIN_ID="${RS_NAVIGATION_ROS_DOMAIN_ID:-${default_domain}}"
export ROS_LOG_DIR="${runtime_dir}/ros-log"
mkdir "${ROS_LOG_DIR}"
python3 "${test_dir}/fake_nav2_action_server.py" >"${runtime_dir}/server.log" 2>&1 &
server_pid=$!
for _ in $(seq 1 200); do
  timeout 2 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose && break
  sleep 0.05
done
timeout 2 ros2 action list 2>/dev/null | grep -Fxq /navigate_to_pose || {
  cat "${runtime_dir}/server.log" >&2
  exit 1
}

cancel_args=()
if [[ "${mode}" == "cancel" ]]; then
  cancel_args=(--cancel-after-ms 250)
fi
harness_args=()
if [[ "${mode}" != "fault" && "${mode}" != "nav2-fault" ]]; then
  harness_args=(--exit-on-terminal)
fi
"${harness_bin}" --manifest "${manifest}" --expected-sha256 "${digest}" run \
  --ipc-root "${runtime_dir}/ipc" --process-epoch 101 \
  "${harness_args[@]}" "${cancel_args[@]}" >"${runtime_dir}/harness.log" 2>&1 &
harness_pid=$!

for _ in $(seq 1 100); do
  grep -Fq 'motor_output=forbidden can_transmit=forbidden' \
    "${runtime_dir}/harness.log" && break
  sleep 0.02
done
grep -Fq 'motor_output=forbidden can_transmit=forbidden' \
  "${runtime_dir}/harness.log" || {
  cat "${runtime_dir}/harness.log" >&2
  exit 1
}

"${bridge_bin}" --manifest "${manifest}" --expected-sha256 "${digest}" \
  --ipc-root "${runtime_dir}/ipc" --process-epoch 202 \
  >"${runtime_dir}/bridge.log" 2>&1 &
bridge_pid=$!

if [[ "${mode}" == "fault" ]]; then
  for _ in $(seq 1 300); do
    grep -Fq 'status=Succeeded' "${runtime_dir}/harness.log" && break
    sleep 0.05
  done
  grep -Fq 'status=Succeeded' "${runtime_dir}/harness.log"
  fault_started_ns="$(date +%s%N)"
  kill "${bridge_pid}"
  wait "${bridge_pid}" 2>/dev/null || true
  bridge_pid=""
  for _ in $(seq 1 200); do
    grep -Fq 'stop=PeerUnavailable' "${runtime_dir}/harness.log" && break
    sleep 0.02
  done
  grep -Fq 'stop=PeerUnavailable' "${runtime_dir}/harness.log"
  fault_observed_ns="$(date +%s%N)"
  stop_latency_ms=$(( (fault_observed_ns - fault_started_ns + 999999) / 1000000 ))
  (( stop_latency_ms <= 100 )) || {
    echo "bridge fault stop exceeded 100 ms: ${stop_latency_ms} ms" >&2
    exit 1
  }
  kill -0 "${harness_pid}"
  echo "motor-disabled bridge fault produced bounded stop while harness stayed alive stop_latency_observed_ms=${stop_latency_ms}"
  exit 0
fi

if [[ "${mode}" == "nav2-fault" ]]; then
  for _ in $(seq 1 300); do
    grep -Fq 'proposal=Accepted' "${runtime_dir}/harness.log" && break
    sleep 0.02
  done
  grep -Fq 'proposal=Accepted' "${runtime_dir}/harness.log"
  fault_started_ns="$(date +%s%N)"
  kill "${server_pid}"
  wait "${server_pid}" 2>/dev/null || true
  server_pid=""
  for _ in $(seq 1 200); do
    grep -Fq 'stop=ProposalExpired' "${runtime_dir}/harness.log" && break
    sleep 0.02
  done
  grep -Fq 'stop=ProposalExpired' "${runtime_dir}/harness.log"
  fault_observed_ns="$(date +%s%N)"
  stop_latency_ms=$(( (fault_observed_ns - fault_started_ns + 999999) / 1000000 ))
  proposal_age_us="$(sed -n 's/.*stop=ProposalExpired.*proposal_age_us=\([0-9][0-9]*\).*/\1/p' \
    "${runtime_dir}/harness.log" | tail -n 1)"
  [[ "${proposal_age_us}" =~ ^[0-9]+$ ]] || exit 1
  (( proposal_age_us <= 110000 )) || {
    echo "proposal expiry exceeded 100 ms TTL plus 10 ms observation allowance: ${proposal_age_us} us" >&2
    exit 1
  }
  (( stop_latency_ms <= 200 )) || {
    echo "Nav2 process fault stop exceeded 200 ms: ${stop_latency_ms} ms" >&2
    exit 1
  }
  kill -0 "${harness_pid}"
  kill -0 "${bridge_pid}"
  echo "motor-disabled Nav2 fault produced proposal expiry while Rust processes stayed alive proposal_age_us=${proposal_age_us} stop_latency_observed_ms=${stop_latency_ms}"
  exit 0
fi

if ! timeout 20 tail --pid="${harness_pid}" -f /dev/null; then
  cat "${runtime_dir}/harness.log" >&2
  cat "${runtime_dir}/bridge.log" >&2
  cat "${runtime_dir}/server.log" >&2
  exit 1
fi
wait "${harness_pid}"
harness_pid=""
if [[ "${mode}" == "success" ]]; then
  grep -Fq 'status=Succeeded' "${runtime_dir}/harness.log"
else
  grep -Fq 'decision=Accepted' "${runtime_dir}/harness.log"
  grep -Fq 'status=Cancelled' "${runtime_dir}/harness.log"
fi
grep -Fq 'motor_output=forbidden can_transmit=forbidden' \
  "${runtime_dir}/harness.log"
echo "motor-disabled ${mode} action loop passed"
