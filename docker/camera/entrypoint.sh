#!/bin/bash
set -e
source /opt/ros/humble/setup.bash

if [ -n "$CYCLONEDDS_URI" ]; then
    XML_PATH="${CYCLONEDDS_URI#file://}"
    envsubst < "$XML_PATH" > /tmp/cyclonedds_resolved.xml
    export CYCLONEDDS_URI="file:///tmp/cyclonedds_resolved.xml"
fi

if [ -f /app/install/setup.bash ]; then
    source /app/install/setup.bash
else
    echo "[entrypoint] WARNING: /app/install/setup.bash not found, skipping"
fi

exec "$@"