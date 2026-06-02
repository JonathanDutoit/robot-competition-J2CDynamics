#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

if [ -n "$CYCLONEDDS_URI" ]; then
    XML_PATH="${CYCLONEDDS_URI#file://}"
    envsubst < "$XML_PATH" > /tmp/cyclonedds_resolved.xml
    export CYCLONEDDS_URI="file:///tmp/cyclonedds_resolved.xml"
fi

cd "$ROS_WS"
echo "[entrypoint] Building workspace..."
colcon build --symlink-install

if [ -f "$ROS_WS/install/setup.bash" ]; then
    source "$ROS_WS/install/setup.bash"
else
    echo "[entrypoint] WARNING: /$ROS_WS/install/setup.bash not found, skipping"
fi

exec "$@"