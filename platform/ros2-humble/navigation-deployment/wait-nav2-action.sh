#!/usr/bin/env bash
set -euo pipefail

action_name="${1:-}"
timeout_s="${2:-}"
[[ "${action_name}" == /* && "${timeout_s}" =~ ^[1-9][0-9]*$ ]] || {
  echo "usage: wait-nav2-action.sh /absolute_action TIMEOUT_SECONDS [/lifecycle_node ...]" >&2
  exit 2
}
shift 2
for node in "$@"; do
  [[ "${node}" == /* ]] || {
    echo "lifecycle node must be absolute: ${node}" >&2
    exit 2
  }
done

deadline=$((SECONDS + timeout_s))
while ((SECONDS < deadline)); do
  ready=true
  for node in "$@"; do
    if ! timeout 2 ros2 lifecycle get "${node}" 2>/dev/null \
      | grep -Eq '^active([[:space:]]|$)'; then
      ready=false
      break
    fi
  done
  if [[ "${ready}" == true ]] \
    && timeout 2 ros2 action list 2>/dev/null \
      | grep -Fxq -- "${action_name}"; then
    exit 0
  fi
  sleep 1
done
echo "Nav2 action/lifecycle did not become ready: ${action_name} nodes=$*" >&2
exit 1
