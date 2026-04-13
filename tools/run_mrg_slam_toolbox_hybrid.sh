#!/usr/bin/env bash
set -euo pipefail

config="${MRG_HYBRID_SLAM_TOOLBOX_CONFIG:-/spot_bringup/config/slam_toolbox_mrg_hybrid.yaml}"

args=(
  ros2 launch spot_velodyne_config mrg_slam_toolbox_hybrid.launch.py
  "slam_toolbox_config:=$config"
  "use_sim_time:=${MRG_HYBRID_USE_SIM_TIME:-False}"
  "cloud_topic:=${MRG_HYBRID_CLOUD_TOPIC:-/velodyne/points}"
  "raw_scan_topic:=${MRG_HYBRID_RAW_SCAN_TOPIC:-/slam_toolbox/scan_raw}"
  "scan_topic:=${MRG_HYBRID_SCAN_TOPIC:-/slam_toolbox/scan}"
  "restamp_scan:=${MRG_HYBRID_RESTAMP_SCAN:-True}"
  "target_frame:=${MRG_HYBRID_LASERSCAN_TARGET_FRAME:-mrg_velodyne}"
  "scan_parent_frame:=${MRG_HYBRID_SCAN_PARENT_FRAME:-body}"
  "scan_frame:=${MRG_HYBRID_SCAN_FRAME:-slam_toolbox_velodyne}"
  "base_frame:=${MRG_HYBRID_BASE_FRAME:-body}"
  "odom_frame:=${MRG_HYBRID_ODOM_FRAME:-odom}"
  "map_frame:=${MRG_HYBRID_MAP_FRAME:-map}"
  "scan_x:=${MRG_HYBRID_SCAN_X:-0.0}"
  "scan_y:=${MRG_HYBRID_SCAN_Y:-0.0}"
  "scan_z:=${MRG_HYBRID_SCAN_Z:-0.25}"
  "scan_roll:=${MRG_HYBRID_SCAN_ROLL:-0.0}"
  "scan_pitch:=${MRG_HYBRID_SCAN_PITCH:-0.0}"
  "scan_yaw:=${MRG_HYBRID_SCAN_YAW:-0.0}"
  "min_height:=${MRG_HYBRID_SCAN_MIN_HEIGHT:--0.15}"
  "max_height:=${MRG_HYBRID_SCAN_MAX_HEIGHT:-0.35}"
  "angle_increment:=${MRG_HYBRID_SCAN_ANGLE_INCREMENT:-0.008726646}"
  "scan_time:=${MRG_HYBRID_SCAN_TIME:-0.1}"
  "range_min:=${MRG_HYBRID_SCAN_RANGE_MIN:-0.8}"
  "range_max:=${MRG_HYBRID_SCAN_RANGE_MAX:-45.0}"
  "transform_tolerance:=${MRG_HYBRID_SCAN_TRANSFORM_TOLERANCE:-0.25}"
  "transform_publish_period:=${MRG_HYBRID_TRANSFORM_PUBLISH_PERIOD:-0.05}"
)

exec "${args[@]}"
