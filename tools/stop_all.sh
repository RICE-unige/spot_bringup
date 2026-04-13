#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"

dry_run=0
if [[ "${1:-}" == "--dry-run" ]]; then
  dry_run=1
fi

run() {
  if [[ "$dry_run" == "1" ]]; then
    printf '+'
    for arg in "$@"; do
      printf ' %q' "$arg"
    done
    printf '\n'
  else
    "$@"
  fi
}

compose_profiles=(
  --profile slam
  --profile rviz
  --profile rtabmap
  --profile rtabmap-rviz
  --profile mrg-hybrid
  --profile mrg-hybrid-rviz
  --profile liosam
  --profile liosam-bridge
  --profile liosam-rviz
  --profile fastlio
  --profile fastlio-bridge
  --profile fastlio-rviz
)

bringup_containers=(
  spot_driver
  spot_rviz
  spot_velodyne
  spot_velodyne_liosam
  spot_sdk_imu
  lio_sam
  lio_sam_map_odom_bridge
  lio_sam_rviz
  mrg_slam
  map_odom_bridge
  mrg_rviz
  spot_velodyne_mrg_hybrid
  mrg_slam_hybrid
  mrg_slam_toolbox
  mrg_hybrid_map_odom_bridge
  mrg_hybrid_map_align_bridge
  mrg_hybrid_rviz
  spot_velodyne_rtabmap
  rtabmap
  rtabmap_map_odom_bridge
  rtabmap_rviz
  mola_lo
  spot_velodyne_fastlio
  fastlio_imu
  fast_lio
  fastlio_map_odom_bridge
  fastlio_rviz
)

echo "Stopping spot_bringup Compose services from: $repo_dir"
run docker compose "${compose_profiles[@]}" down --remove-orphans

running_containers=()
for name in "${bringup_containers[@]}"; do
  if docker ps --format '{{.Names}}' | grep -Fxq "$name"; then
    running_containers+=("$name")
  fi
done

if (( ${#running_containers[@]} > 0 )); then
  echo "Stopping leftover spot_bringup containers: ${running_containers[*]}"
  run docker stop "${running_containers[@]}"
fi

if [[ "$dry_run" == "1" ]]; then
  echo "Dry run complete; no containers were stopped."
  exit 0
fi

remaining=()
for name in "${bringup_containers[@]}"; do
  if docker ps --format '{{.Names}}' | grep -Fxq "$name"; then
    remaining+=("$name")
  fi
done

if (( ${#remaining[@]} > 0 )); then
  echo "Still running: ${remaining[*]}" >&2
  exit 1
fi

echo "All spot_bringup containers are stopped."
