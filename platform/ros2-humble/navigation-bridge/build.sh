#!/usr/bin/env bash
set -euo pipefail

bridge_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
rs_control_root="$(cd "${bridge_dir}/../../.." && pwd -P)"
ros_prefix="${AMENT_PREFIX_PATH%%:*}"
verify_only=false
output=""

usage() {
  echo "usage: build.sh [--verify-only | --output ABSENT_ABSOLUTE_FILE]" >&2
}

while (($# > 0)); do
  case "$1" in
    --verify-only)
      verify_only=true
      shift
      ;;
    --output)
      (($# >= 2)) || { usage; exit 2; }
      output="$2"
      shift 2
      ;;
    *)
      usage
      exit 2
      ;;
  esac
done

if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "ROS_DISTRO must be humble; source /opt/ros/humble/setup.bash first" >&2
  exit 1
fi
if [[ "${RMW_IMPLEMENTATION:-}" != "rmw_fastrtps_cpp" ]]; then
  echo "RMW_IMPLEMENTATION must be rmw_fastrtps_cpp" >&2
  exit 1
fi
if [[ -z "${ros_prefix}" || ! -d "${ros_prefix}/share/nav2_msgs/rust" ]]; then
  echo "AMENT_PREFIX_PATH does not expose the pinned ROS Rust message tree" >&2
  exit 1
fi

lock_value() {
  local key="$1"
  sed -n "s/^${key}=//p" "${bridge_dir}/ros-sources.lock"
}

verify_hash() {
  local key="$1"
  local path="$2"
  local expected actual
  expected="$(lock_value "${key}")"
  actual="$(sha256sum "${path}" | awk '{print $1}')"
  if [[ -z "${expected}" || "${actual}" != "${expected}" ]]; then
    echo "source lock mismatch: ${path}" >&2
    exit 1
  fi
}

[[ "$(lock_value ROS_PREFIX)" == "/opt/ros/humble" ]] || {
  echo "ros-sources.lock contains an unexpected ROS prefix" >&2
  exit 1
}
[[ "$(lock_value RCLRS_VERSION)" == "0.7.0" ]] || {
  echo "ros-sources.lock contains an unexpected rclrs version" >&2
  exit 1
}
verify_hash NAV2_CARGO_SHA256 "${ros_prefix}/share/nav2_msgs/rust/Cargo.toml"
verify_hash GEOMETRY_CARGO_SHA256 "${ros_prefix}/share/geometry_msgs/rust/Cargo.toml"
verify_hash NAV_CARGO_SHA256 "${ros_prefix}/share/nav_msgs/rust/Cargo.toml"
verify_hash TF2_CARGO_SHA256 "${ros_prefix}/share/tf2_msgs/rust/Cargo.toml"
verify_hash NAVIGATE_TO_POSE_SHA256 "${ros_prefix}/share/nav2_msgs/action/NavigateToPose.action"
verify_hash TWIST_SHA256 "${ros_prefix}/share/geometry_msgs/msg/Twist.msg"

test -f "${bridge_dir}/Cargo.lock" || {
  echo "checked Cargo.lock is missing" >&2
  exit 1
}
if rg -n '/opt/ros|@ROS_PREFIX@|@RS_CONTROL_ROOT@' "${bridge_dir}/Cargo.lock"; then
  echo "Cargo.lock must not retain rendered absolute source paths" >&2
  exit 1
fi
if rg -n 'hardware-module|motor-damiao|transport-socketcan|socketcan' \
  "${bridge_dir}/Cargo.toml.in" "${bridge_dir}/src"; then
  echo "forbidden hardware dependency found in navigation bridge" >&2
  exit 1
fi

if ${verify_only}; then
  [[ -z "${output}" ]] || { usage; exit 2; }
  echo "navigation bridge source lock verified"
  exit 0
fi

[[ -n "${output}" && "${output}" == /* ]] || { usage; exit 2; }
[[ ! -e "${output}" ]] || {
  echo "refusing to overwrite output: ${output}" >&2
  exit 1
}
[[ -d "$(dirname "${output}")" ]] || {
  echo "output parent directory does not exist" >&2
  exit 1
}

build_dir="$(mktemp -d --tmpdir rs-navigation-bridge.XXXXXXXX)"
cleanup() {
  rm -rf -- "${build_dir}"
}
trap cleanup EXIT

mkdir -p "${build_dir}/src"
cp "${bridge_dir}/Cargo.lock" "${build_dir}/Cargo.lock"
cp "${bridge_dir}/src/bridge.rs" "${bridge_dir}/src/main.rs" \
  "${bridge_dir}/src/tests.rs" "${build_dir}/src/"
sed -e "s|@RS_CONTROL_ROOT@|${rs_control_root}|g" \
  -e "s|@ROS_PREFIX@|${ros_prefix}|g" \
  "${bridge_dir}/Cargo.toml.in" >"${build_dir}/Cargo.toml"

cargo test --manifest-path "${build_dir}/Cargo.toml" --offline --locked
cargo clippy --manifest-path "${build_dir}/Cargo.toml" --offline --locked \
  --all-targets -- -D warnings
cargo build --manifest-path "${build_dir}/Cargo.toml" --offline --locked --release
install -m 0755 "${build_dir}/target/release/rs-navigation-bridge" "${output}"
echo "built ${output}"
