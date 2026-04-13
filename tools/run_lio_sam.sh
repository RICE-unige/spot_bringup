#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
source /liosam_ws/install/setup.bash
set -u

exec ros2 launch spot_velodyne_config lio_sam_spot.launch.py \
  params_file:="${LIO_SAM_CONFIG:-/spot_bringup/config/lio_sam_spot.yaml}"
