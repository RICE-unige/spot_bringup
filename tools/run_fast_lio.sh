#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
source /fastlio_ws/install/setup.bash
set -u

exec ros2 launch spot_velodyne_config fast_lio_spot.launch.py \
  config_file:="${FAST_LIO_CONFIG:-/spot_bringup/config/fast_lio_spot.yaml}" \
  lidar_topic:="${FAST_LIO_LIDAR_TOPIC:-/velodyne/points}" \
  imu_topic:="${FAST_LIO_IMU_TOPIC:-/fast_lio/imu}"
