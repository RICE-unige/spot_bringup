#!/usr/bin/env bash
set -euo pipefail

config="${MRG_SLAM_CONFIG:-/spot_bringup/config/mrg_slam_spot.yaml}"

args=(
  ros2 launch mrg_slam mrg_slam.launch.py
  "config:=$config"
  "use_sim_time:=${MRG_USE_SIM_TIME:-False}"
  "x:=${MRG_X:-0.0}"
  "y:=${MRG_Y:-0.0}"
  "z:=${MRG_Z:-0.0}"
  "roll:=${MRG_ROLL:-0.0}"
  "pitch:=${MRG_PITCH:-0.0}"
  "yaw:=${MRG_YAW:-0.0}"
  "init_odom_topic:=${MRG_INIT_ODOM_TOPIC:-NONE}"
  "init_pose_topic:=${MRG_INIT_POSE_TOPIC:-NONE}"
)

if [ -n "${MRG_RESULT_DIR:-}" ]; then
  args+=("result_dir:=${MRG_RESULT_DIR}")
fi

exec "${args[@]}"
