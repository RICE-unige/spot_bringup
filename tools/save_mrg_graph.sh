#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
default_rel="graphs/graph_$(date +%Y%m%d_%H%M%S)"
target="${1:-$default_rel}"

if [[ "$target" = /* ]]; then
  host_dir="$target"
else
  host_dir="$repo_root/$target"
fi

mkdir -p "$host_dir"

container_dir="${host_dir/#$repo_root/\/spot_bringup}"
if [[ "$container_dir" == "$host_dir" ]]; then
  echo "ERROR: target directory must stay under $repo_root" >&2
  exit 1
fi

printf -v request '{directory: %s}' "$container_dir"

docker exec mrg_slam /bin/bash -lc \
  "source /opt/ros/humble/setup.bash && \
   source /mrg_ws/install/setup.bash && \
   export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-17} && \
   ros2 service call /mrg_slam/save_graph mrg_slam_msgs/srv/SaveGraph \"$request\""

find "$host_dir" -maxdepth 2 -type f | sort
