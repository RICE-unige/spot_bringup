#!/usr/bin/env bash
set -eo pipefail

set +u
source /opt/ros/humble/setup.bash
if [ -f /ros_ws/install/setup.bash ]; then
  source /ros_ws/install/setup.bash
fi
set -u

export CYCLONEDDS_URI
CYCLONEDDS_URI="$(render_cyclonedds_config)"

render_spot_config="${RENDER_SPOT_CONFIG:-1}"
case "${render_spot_config,,}" in
  0|false|no)
    exec "$@"
    ;;
esac

export SPOT_HOSTNAME="${SPOT_HOSTNAME:-${SPOT_IP:-192.168.80.3}}"
export SPOT_USERNAME="${SPOT_USERNAME:-${BOSDYN_CLIENT_USERNAME:-user}}"
export SPOT_NAME="${SPOT_NAME:-}"
export SPOT_FRAME_PREFIX="${SPOT_FRAME_PREFIX:-}"

if [ -n "${SPOT_PASSWORD_FILE:-}" ]; then
  if [ ! -f "$SPOT_PASSWORD_FILE" ]; then
    echo "ERROR: SPOT_PASSWORD_FILE is set to '$SPOT_PASSWORD_FILE' but the file does not exist." >&2
    exit 1
  fi
  export SPOT_PASSWORD
  SPOT_PASSWORD="$(tr -d '\r\n' < "$SPOT_PASSWORD_FILE")"
elif [ -n "${SPOT_PASSWORD:-}" ]; then
  export SPOT_PASSWORD
elif [ -n "${BOSDYN_CLIENT_PASSWORD:-}" ]; then
  export SPOT_PASSWORD="$BOSDYN_CLIENT_PASSWORD"
else
  echo "ERROR: Set SPOT_PASSWORD_FILE or SPOT_PASSWORD before starting spot-driver." >&2
  exit 1
fi

template="${SPOT_CONFIG_TEMPLATE:-/ros_ws/config/spot_config.yaml}"
generated="${SPOT_CONFIG_GENERATED:-/tmp/spot_config.generated.yaml}"
envsubst '${SPOT_USERNAME} ${SPOT_PASSWORD} ${SPOT_HOSTNAME} ${SPOT_NAME} ${SPOT_FRAME_PREFIX}' < "$template" > "$generated"
chmod 600 "$generated"

exec "$@"
