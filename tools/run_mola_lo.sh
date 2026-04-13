#!/usr/bin/env bash
set -euo pipefail

pipeline="${MOLA_LO_PIPELINE:-}"
if [ -z "$pipeline" ]; then
  pipeline="$(ros2 pkg prefix mola_lidar_odometry)/share/mola_lidar_odometry/pipelines/lidar3d-gicp.yaml"
fi

exec ros2 launch mola_lidar_odometry ros2-lidar-odometry.launch.py \
  lidar_topic_name:="${MOLA_LIDAR_TOPIC:-/velodyne/points}" \
  mola_tf_base_link:="${MOLA_TF_BASE_LINK:-body}" \
  ignore_lidar_pose_from_tf:="${MOLA_IGNORE_LIDAR_POSE_FROM_TF:-False}" \
  publish_localization_following_rep105:="${MOLA_PUBLISH_REP105:-True}" \
  generate_simplemap:="${MOLA_GENERATE_SIMPLEMAP:-True}" \
  use_mola_gui:="${MOLA_USE_MOLA_GUI:-False}" \
  use_rviz:="${MOLA_USE_RVIZ:-False}" \
  mola_lo_pipeline:="$pipeline"
