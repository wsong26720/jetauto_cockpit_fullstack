#!/usr/bin/env bash
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/ros_env.sh"
export JETAUTO_AUTO_ROTATE="${JETAUTO_AUTO_ROTATE:-true}"
echo "JetAuto cockpit: ROS_DOMAIN_ID=$ROS_DOMAIN_ID MACHINE_TYPE=$MACHINE_TYPE"
# Start infrastructure only. Use the Live UI to start the drivers.
if ! ss -lnt | awk '{print $4}' | grep -Eq '(^|:)9090$'; then
    nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /tmp/rosbridge_9090.log 2>&1 &
    echo "rosbridge requested; log: /tmp/rosbridge_9090.log"
else
    echo "9090 already listening; verify its ROS_DOMAIN_ID matches $ROS_DOMAIN_ID."
fi
exec python3 "$SCRIPT_DIR/agent_server.py"
