#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
default_rel="maps/map_$(date +%Y%m%d_%H%M%S).pcd"
target="${1:-$default_rel}"
resolution="${2:-0.1}"

if [[ "$target" = /* ]]; then
  host_path="$target"
else
  host_path="$repo_root/$target"
fi

mkdir -p "$(dirname "$host_path")"

container_path="${host_path/#$repo_root/\/spot_bringup}"
if [[ "$container_path" == "$host_path" ]]; then
  echo "ERROR: target path must stay under $repo_root" >&2
  exit 1
fi

printf -v request '{file_path: %s, resolution: %s}' "$container_path" "$resolution"

docker exec mrg_slam /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /mrg_ws/install/setup.bash && \
   export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-17} && \
   ros2 service call /mrg_slam/save_map mrg_slam_msgs/srv/SaveMap \"$request\""

ls -lh "$host_path"
