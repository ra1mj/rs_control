#!/usr/bin/env bash
set -euo pipefail

deployment_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
test_root="$(mktemp -d --tmpdir rs-navigation-systemd.XXXXXXXX)"
cleanup() {
  rm -rf -- "${test_root}"
}
trap cleanup EXIT

for unit in "${deployment_dir}"/*.service "${deployment_dir}"/*.target "${deployment_dir}"/*.slice; do
  sed -e 's|^EnvironmentFile=.*|EnvironmentFile=-/dev/null|' \
    -e 's|^ExecStart=.*|ExecStart=/bin/true|' \
    -e 's|^ExecStartPost=.*|ExecStartPost=/bin/true|' \
    "${unit}" >"${test_root}/$(basename "${unit}")"
done

SYSTEMD_UNIT_PATH="${test_root}:/usr/lib/systemd/system:/lib/systemd/system" \
  systemd-analyze verify "${test_root}"/*.service "${test_root}"/*.target \
  "${test_root}"/*.slice

grep -Fq 'BindsTo=rs-control-navigation-harness.service' \
  "${deployment_dir}/rs-control-navigation.target"
grep -Fq 'Slice=rs-control-navigation.slice' \
  "${deployment_dir}/rs-control-navigation-harness.service"
grep -Fq '/opt/rs-control/libexec/run-nav2-headless.sh' \
  "${deployment_dir}/rs-control-navigation-nav2.service"
for role in map_manifest occupancy_yaml occupancy_image base_parameters \
  tuning_parameters integration_parameters navigate_to_pose_bt \
  navigate_through_poses_bt navigate_to_pose_interface \
  controller_proposal_interface; do
  grep -Fq -- "--asset \"${role}=" "${deployment_dir}/run-nav2-headless.sh"
done
for executable in map_server controller_server planner_server bt_navigator \
  lifecycle_manager; do
  grep -Fq "/${executable}" "${deployment_dir}/run-nav2-headless.sh"
done
grep -Fq 'wait_nav2_ready.py /navigate_to_pose 60 /lifecycle_manager_navigation' \
  "${deployment_dir}/rs-control-navigation-nav2.service"
grep -Fq -- '--nav2-ready-file /run/rs-control-navigation/nav2.ready' \
  "${deployment_dir}/rs-control-navigation-bridge.service"
grep -Fq '/usr/bin/touch /run/rs-control-navigation/nav2.ready' \
  "${deployment_dir}/rs-control-navigation-nav2.service"
PYTHONPYCACHEPREFIX="${test_root}/pycache" \
  python3 -m py_compile "${deployment_dir}/wait_nav2_ready.py"
for script in "${deployment_dir}/run-nav2-headless.sh" \
  "${deployment_dir}/tests/nav2-host-resource-smoke.sh"; do
  bash -n "${script}"
done
if rg -n 'can0|socketcan|hardware-module' "${deployment_dir}"/*.service; then
  echo "forbidden hardware binding found in navigation units" >&2
  exit 1
fi
echo "navigation supervision units verified"
