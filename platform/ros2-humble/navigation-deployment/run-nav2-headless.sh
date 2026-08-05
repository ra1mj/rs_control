#!/usr/bin/env bash
set -euo pipefail

required_environment=(
  RS_CONTROL_BIN NAV_MANIFEST NAV_MANIFEST_SHA256 NAV_PEER_MANIFEST
  NAV_MAP_MANIFEST NAV_MAP_YAML NAV_MAP_IMAGE NAV_BASE_PARAMS
  NAV_TUNING_PARAMS NAV_INTEGRATION_PARAMS NAV_POSE_BT NAV_THROUGH_BT
  NAV_NAVIGATE_TO_POSE_INTERFACE NAV_CONTROLLER_PROPOSAL_INTERFACE
  NAV_PROPOSAL_TOPIC NAV_ODOMETRY_TOPIC NAV_DYNAMIC_TF_TOPIC
)
for name in "${required_environment[@]}"; do
  [[ -n "${!name:-}" ]] || {
    echo "missing navigation environment: ${name}" >&2
    exit 2
  }
done
[[ "${ROS_DISTRO:-}" == humble ]] || {
  echo "ROS_DISTRO must be humble" >&2
  exit 2
}
[[ "${RMW_IMPLEMENTATION:-}" == rmw_fastrtps_cpp ]] || {
  echo "RMW_IMPLEMENTATION must be rmw_fastrtps_cpp" >&2
  exit 2
}
[[ "${NAV_PROPOSAL_TOPIC}" == /rs_control/navigation/controller_proposal \
  && "${NAV_ODOMETRY_TOPIC}" == /rs_control/navigation/odometry \
  && "${NAV_DYNAMIC_TF_TOPIC}" == /tf ]] || {
  echo "navigation endpoints differ from the qualified graph" >&2
  exit 2
}

plugin_validator="${RS_CONTROL_BIN}/rs-navigation-plugin"
[[ -x "${plugin_validator}" ]] || {
  echo "navigation manifest validator is not executable: ${plugin_validator}" >&2
  exit 2
}
actual_digest="$("${plugin_validator}" fingerprint "${NAV_MANIFEST}")"
[[ "${actual_digest}" == "${NAV_MANIFEST_SHA256}" ]] || {
  echo "navigation manifest fingerprint mismatch" >&2
  exit 2
}
"${plugin_validator}" validate "${NAV_MANIFEST}" \
  --peer-manifest "${NAV_PEER_MANIFEST}" \
  --asset "map_manifest=${NAV_MAP_MANIFEST}" \
  --asset "occupancy_yaml=${NAV_MAP_YAML}" \
  --asset "occupancy_image=${NAV_MAP_IMAGE}" \
  --asset "base_parameters=${NAV_BASE_PARAMS}" \
  --asset "tuning_parameters=${NAV_TUNING_PARAMS}" \
  --asset "integration_parameters=${NAV_INTEGRATION_PARAMS}" \
  --asset "navigate_to_pose_bt=${NAV_POSE_BT}" \
  --asset "navigate_through_poses_bt=${NAV_THROUGH_BT}" \
  --asset "navigate_to_pose_interface=${NAV_NAVIGATE_TO_POSE_INTERFACE}" \
  --asset "controller_proposal_interface=${NAV_CONTROLLER_PROPOSAL_INTERFACE}"

common_ros_args=(
  --ros-args
  --params-file "${NAV_BASE_PARAMS}"
  --params-file "${NAV_TUNING_PARAMS}"
  --params-file "${NAV_INTEGRATION_PARAMS}"
)
pids=()
stopping=false

start_node() {
  "$@" &
  pids+=("$!")
}

cleanup() {
  local status=$?
  trap - EXIT TERM INT
  for pid in "${pids[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  if [[ "${stopping}" == true ]]; then
    exit 0
  fi
  exit "${status}"
}
trap cleanup EXIT
trap 'stopping=true; exit 0' TERM INT

start_node /opt/ros/humble/lib/nav2_map_server/map_server \
  "${common_ros_args[@]}" -r __node:=map_server \
  -p use_sim_time:=false -p "yaml_filename:=${NAV_MAP_YAML}"
start_node /opt/ros/humble/lib/nav2_controller/controller_server \
  "${common_ros_args[@]}" -r __node:=controller_server \
  -r "cmd_vel:=${NAV_PROPOSAL_TOPIC}" \
  -p use_sim_time:=false -p FollowPath.visualize:=false
start_node /opt/ros/humble/lib/nav2_planner/planner_server \
  "${common_ros_args[@]}" -r __node:=planner_server \
  -p use_sim_time:=false
start_node /opt/ros/humble/lib/nav2_bt_navigator/bt_navigator \
  "${common_ros_args[@]}" -r __node:=bt_navigator \
  -p use_sim_time:=false \
  -p "default_nav_to_pose_bt_xml:=${NAV_POSE_BT}" \
  -p "default_nav_through_poses_bt_xml:=${NAV_THROUGH_BT}"
start_node /opt/ros/humble/lib/nav2_lifecycle_manager/lifecycle_manager \
  "${common_ros_args[@]}" -r __node:=lifecycle_manager_navigation \
  -p use_sim_time:=false -p autostart:=true \
  -p 'node_names:=[map_server,controller_server,planner_server,bt_navigator]'

set +e
wait -n "${pids[@]}"
status=$?
set -e
echo "critical Nav2 process exited status=${status}" >&2
exit 1
