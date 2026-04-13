#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi
if [ -f /mrg_ws/install/setup.bash ]; then
  source /mrg_ws/install/setup.bash
fi
if [ -f /liosam_ws/install/setup.bash ]; then
  source /liosam_ws/install/setup.bash
fi
if [ -f /fastlio_ws/install/setup.bash ]; then
  source /fastlio_ws/install/setup.bash
fi
set -u

export CYCLONEDDS_URI
CYCLONEDDS_URI="$(render_cyclonedds_config)"

exec "$@"
