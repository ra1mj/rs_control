#!/usr/bin/env bash
set -euo pipefail

bridge_bin="${1:-}"
harness_bin="${2:-}"
chassis_source="${3:-}"
chassis_install="${4:-}"
evidence_dir="${5:-}"
mode="${6:-success}"
case "${mode}" in
  success|localization-fault) ;;
  *) echo "mode must be success or localization-fault" >&2; exit 2 ;;
esac
for path in "${bridge_bin}" "${harness_bin}" "${chassis_source}" \
  "${chassis_install}" "${evidence_dir}"; do
  [[ "${path}" == /* && "${path}" != "/" ]] || {
    echo "usage: nav2-host-resource-smoke.sh BRIDGE HARNESS CHASSIS_SOURCE CHASSIS_INSTALL ABSENT_EVIDENCE_DIR" >&2
    exit 2
  }
done
[[ -x "${bridge_bin}" && -x "${harness_bin}" ]] || exit 2
[[ -d "${chassis_source}" && -r "${chassis_install}/setup.bash" ]] || exit 2
[[ ! -e "${evidence_dir}" ]] || {
  echo "refusing to overwrite evidence directory: ${evidence_dir}" >&2
  exit 1
}

test_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
deployment_dir="$(cd "${test_dir}/.." && pwd -P)"
rs_control_root="$(cd "${deployment_dir}/../../.." && pwd -P)"
runtime_dir="$(mktemp -d --tmpdir rs-navigation-nav2-resource.XXXXXXXX)"
harness_pid=""
nav2_pid=""
bridge_pid=""
sensor_pid=""
cleanup() {
  for pid in "${bridge_pid}" "${nav2_pid}" "${sensor_pid}" "${harness_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  for pid in "${bridge_pid}" "${nav2_pid}" "${sensor_pid}" "${harness_pid}"; do
    if [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rm -rf -- "${runtime_dir}"
}
trap cleanup EXIT

manifest="${rs_control_root}/config/navigation/rs_control_navigation_plugin.yaml"
digest="798fb5358dc3b9375b1102c92db75cf558491d8db1841785e3c2286bcda4b8ca"
map_root="${chassis_source}/src/chassis_nav_mecanum_sim/assets/localization_map/warehouse_sim_map/0.1.0"
bringup_root="${chassis_source}/src/chassis_nav_bringup"
export RS_CONTROL_BIN="$(dirname "${harness_bin}")"
export NAV_MANIFEST="${manifest}"
export NAV_MANIFEST_SHA256="${digest}"
export NAV_PEER_MANIFEST="${bringup_root}/config/rs_control_navigation_plugin.yaml"
export NAV_MAP_MANIFEST="${map_root}/manifest.yaml"
export NAV_MAP_YAML="${map_root}/maps/occupancy.yaml"
export NAV_MAP_IMAGE="${map_root}/maps/occupancy.pgm"
export NAV_BASE_PARAMS="${bringup_root}/config/rs_control_navigation_base.yaml"
export NAV_TUNING_PARAMS="${bringup_root}/config/rs_control_navigation_tuning.yaml"
export NAV_INTEGRATION_PARAMS="${bringup_root}/config/rs_control_navigation.yaml"
export NAV_POSE_BT="${bringup_root}/behavior_trees/navigate_to_pose_fail_closed.xml"
export NAV_THROUGH_BT="${bringup_root}/behavior_trees/navigate_through_poses_disabled.xml"
export NAV_NAVIGATE_TO_POSE_INTERFACE=/opt/ros/humble/share/nav2_msgs/action/NavigateToPose.action
export NAV_CONTROLLER_PROPOSAL_INTERFACE=/opt/ros/humble/share/geometry_msgs/msg/Twist.msg
export NAV_PROPOSAL_TOPIC=/rs_control/navigation/controller_proposal
export NAV_ODOMETRY_TOPIC=/rs_control/navigation/odometry
export NAV_DYNAMIC_TF_TOPIC=/tf

set +u
source /opt/ros/humble/setup.bash
source "${chassis_install}/setup.bash"
set -u
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
default_domain=$((100 + BASHPID % 100))
export ROS_DOMAIN_ID="${RS_NAVIGATION_ROS_DOMAIN_ID:-${default_domain}}"
export ROS_LOG_DIR="${runtime_dir}/ros-log"
mkdir "${ROS_LOG_DIR}"

python3 "${test_dir}/synthetic_tracker_publisher.py" \
  >"${runtime_dir}/synthetic-tracker.log" 2>&1 &
sensor_pid=$!

harness_args=()
if [[ "${mode}" == "localization-fault" ]]; then
  harness_args=(--stop-feedback-after-first-proposal-ms 250)
fi
"${harness_bin}" --manifest "${manifest}" --expected-sha256 "${digest}" run \
  --ipc-root "${runtime_dir}/ipc" --process-epoch 301 "${harness_args[@]}" \
  >"${runtime_dir}/harness.log" 2>&1 &
harness_pid=$!
for _ in $(seq 1 100); do
  grep -Fq 'motor_output=forbidden can_transmit=forbidden' \
    "${runtime_dir}/harness.log" && break
  sleep 0.02
done
grep -Fq 'motor_output=forbidden can_transmit=forbidden' \
  "${runtime_dir}/harness.log"

"${bridge_bin}" --manifest "${manifest}" --expected-sha256 "${digest}" \
  --ipc-root "${runtime_dir}/ipc" --process-epoch 302 \
  --nav2-ready-file "${runtime_dir}/nav2.ready" \
  >"${runtime_dir}/bridge.log" 2>&1 &
bridge_pid=$!
for _ in $(seq 1 200); do
  if timeout 2 ros2 node list 2>/dev/null | grep -Fxq /rs_navigation_bridge; then
    break
  fi
  sleep 0.05
done
kill -0 "${bridge_pid}"

"${deployment_dir}/run-nav2-headless.sh" \
  >"${runtime_dir}/nav2.log" 2>&1 &
nav2_pid=$!

if ! "${deployment_dir}/wait_nav2_ready.py" /navigate_to_pose 60 \
  /lifecycle_manager_navigation; then
  cat "${runtime_dir}/nav2.log" >&2
  exit 1
fi
touch "${runtime_dir}/nav2.ready"
for _ in $(seq 1 400); do
  grep -Fq 'Received a goal, begin computing control effort.' \
    "${runtime_dir}/nav2.log" && break
  sleep 0.05
done
grep -Fq 'Received a goal, begin computing control effort.' \
  "${runtime_dir}/nav2.log"
kill -0 "${harness_pid}"
kill -0 "${nav2_pid}"
kill -0 "${bridge_pid}"
kill -0 "${sensor_pid}"

"${deployment_dir}/collect-host-evidence.sh" "${evidence_dir}" - \
  "${harness_pid}" "${nav2_pid}" "${bridge_pid}" "${sensor_pid}"
ros2 node list >"${evidence_dir}/ros2-node-list.txt"
ros2 action list >"${evidence_dir}/ros2-action-list.txt"
cp "${runtime_dir}/harness.log" "${evidence_dir}/harness.log"
cp "${runtime_dir}/nav2.log" "${evidence_dir}/nav2.log"
cp "${runtime_dir}/bridge.log" "${evidence_dir}/bridge.log"
cp "${runtime_dir}/synthetic-tracker.log" \
  "${evidence_dir}/synthetic-tracker.log"
{
  echo "workload=real_nav2_static_map_with_rs_control_synthetic_localization"
  echo "nav2_profile=map_server,controller_server,planner_server,bt_navigator,lifecycle_manager"
  echo "nav2_supervisor=direct_shell_no_ros2_launch"
  echo "production_slam_process=not_run"
  echo "obstacle_input=test_only_fresh_range_filtered_pointcloud"
  echo "initial_pose_map=1.5,-1.5,0.0"
  echo "goal_pose_map=1.5,-3.0,0.0"
  echo "snapshot_phase=active_controller"
  echo "test_mode=${mode}"
  echo "bridge_binary_sha256=$(sha256sum "${bridge_bin}" | awk '{print $1}')"
  echo "harness_binary_sha256=$(sha256sum "${harness_bin}" | awk '{print $1}')"
  echo "qualification=measurement_only"
} >>"${evidence_dir}/metadata.txt"

for node in map_server controller_server planner_server bt_navigator \
  lifecycle_manager_navigation rs_navigation_bridge; do
  grep -Fxq "/${node}" "${evidence_dir}/ros2-node-list.txt"
done
grep -Fxq /navigate_to_pose "${evidence_dir}/ros2-action-list.txt"
expected_status=Succeeded
if [[ "${mode}" == "localization-fault" ]]; then
  expected_status=Failed
  grep -Fq 'feedback=Unavailable reason=InjectedLocalizationLoss' \
    "${runtime_dir}/harness.log"
fi
for _ in $(seq 1 1200); do
  grep -Fq "status=${expected_status}" "${runtime_dir}/harness.log" && break
  sleep 0.05
done
grep -Fq "status=${expected_status}" "${runtime_dir}/harness.log" || {
  cat "${runtime_dir}/harness.log" >&2
  cat "${runtime_dir}/bridge.log" >&2
  exit 1
}
if [[ "${mode}" == "localization-fault" ]]; then
  for _ in $(seq 1 40); do
    grep -Fq 'stop=ProposalExpired' "${runtime_dir}/harness.log" && break
    sleep 0.005
  done
  grep -Fq 'stop=ProposalExpired' "${runtime_dir}/harness.log"
  proposal_age_us="$(sed -n 's/.*stop=ProposalExpired.*proposal_age_us=\([0-9][0-9]*\).*/\1/p' \
    "${runtime_dir}/harness.log" | tail -n 1)"
  [[ "${proposal_age_us}" =~ ^[0-9]+$ ]]
  (( proposal_age_us <= 110000 ))
  echo "localization_fault_proposal_age_us=${proposal_age_us}" \
    >>"${evidence_dir}/metadata.txt"
  kill -0 "${harness_pid}"
fi
echo "result=passed" >>"${evidence_dir}/metadata.txt"
cp "${runtime_dir}/harness.log" "${evidence_dir}/harness.log"
cp "${runtime_dir}/nav2.log" "${evidence_dir}/nav2.log"
cp "${runtime_dir}/bridge.log" "${evidence_dir}/bridge.log"
(
  cd "${evidence_dir}"
  find . -type f ! -name SHA256SUMS -print0 \
    | sort -z \
    | xargs -0 sha256sum >SHA256SUMS
)
echo "real Nav2 motor-disabled ${mode} smoke passed evidence=${evidence_dir}"
