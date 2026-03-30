#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Expand env vars in the CycloneDDS XML before CycloneDDS reads it
if [ -n "$CYCLONEDDS_URI" ]; then
    XML_PATH="${CYCLONEDDS_URI#file://}"
    envsubst < "$XML_PATH" > /tmp/cyclonedds_resolved.xml
    export CYCLONEDDS_URI="file:///tmp/cyclonedds_resolved.xml"
fi

cd "$ROS_WS"

if [ ! -d "$ROS_WS/install" ] && [ -d "$ROS_WS/src" ] && [ -n "$(ls -A $ROS_WS/src)" ]; then
    echo "[entrypoint] install/ not found — running colcon build …"
    colcon build --symlink-install
fi

if [ -f "$ROS_WS/install/setup.bash" ]; then
    source "$ROS_WS/install/setup.bash"
fi

exec "$@"