#!/usr/bin/env bash
set -euo pipefail

args=(
  ros2 launch spot_velodyne_config rtabmap_spot.launch.py
  "namespace:=${RTABMAP_NAMESPACE:-rtabmap}"
  "scan_cloud_topic:=${RTABMAP_SCAN_CLOUD_TOPIC:-/rtabmap/velodyne_points}"
  "frame_id:=${RTABMAP_FRAME_ID:-rtabmap_body}"
  "odom_frame_id:=${RTABMAP_ODOM_FRAME_ID:-rtabmap_odom}"
  "map_frame_id:=${RTABMAP_MAP_FRAME_ID:-map}"
  "database_path:=${RTABMAP_DATABASE_PATH:-/spot_bringup/maps/rtabmap_spot.db}"
  "delete_db_on_start:=${RTABMAP_DELETE_DB_ON_START:-True}"
  "detection_rate:=${RTABMAP_DETECTION_RATE:-2.0}"
  "qos:=${RTABMAP_QOS:-2}"
  "wait_for_transform:=${RTABMAP_WAIT_FOR_TRANSFORM:-0.3}"
  "voxel_size:=${RTABMAP_VOXEL_SIZE:-0.15}"
  "max_correspondence_distance:=${RTABMAP_MAX_CORRESPONDENCE_DISTANCE:-1.5}"
  "range_min:=${RTABMAP_RANGE_MIN:-0.9}"
  "range_max:=${RTABMAP_RANGE_MAX:-50.0}"
  "global_map_assembler_enabled:=${RTABMAP_GLOBAL_MAP_ASSEMBLER_ENABLED:-True}"
  "global_map_regenerate_local_grids:=${RTABMAP_GLOBAL_MAP_REGENERATE_LOCAL_GRIDS:-False}"
  "global_cloud_topic:=${RTABMAP_GLOBAL_CLOUD_TOPIC:-/rtabmap/global_cloud_map}"
  "global_grid_topic:=${RTABMAP_GLOBAL_GRID_TOPIC:-/rtabmap/global_grid_prob_map}"
  "octomap_tree_depth:=${RTABMAP_OCTOMAP_TREE_DEPTH:-16}"
  "live_cloud_assembler_enabled:=${RTABMAP_LIVE_CLOUD_ASSEMBLER_ENABLED:-False}"
  "live_cloud_topic:=${RTABMAP_LIVE_CLOUD_TOPIC:-/rtabmap/live_cloud_map}"
  "live_cloud_max_clouds:=${RTABMAP_LIVE_CLOUD_MAX_CLOUDS:-20}"
  "live_cloud_skip_clouds:=${RTABMAP_LIVE_CLOUD_SKIP_CLOUDS:-1}"
  "live_cloud_linear_update:=${RTABMAP_LIVE_CLOUD_LINEAR_UPDATE:-0.10}"
  "live_cloud_angular_update:=${RTABMAP_LIVE_CLOUD_ANGULAR_UPDATE:-0.0872665}"
  "live_cloud_wait_for_transform:=${RTABMAP_LIVE_CLOUD_WAIT_FOR_TRANSFORM:-1.0}"
  "live_cloud_voxel_size:=${RTABMAP_LIVE_CLOUD_VOXEL_SIZE:-0.10}"
  "live_cloud_noise_radius:=${RTABMAP_LIVE_CLOUD_NOISE_RADIUS:-0.0}"
  "live_cloud_noise_min_neighbors:=${RTABMAP_LIVE_CLOUD_NOISE_MIN_NEIGHBORS:-5}"
  "log_level:=${RTABMAP_LOG_LEVEL:-info}"
)

exec "${args[@]}"
