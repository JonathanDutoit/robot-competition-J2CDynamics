#!/bin/bash
set -e

# Source the ROS2 base installation
source /opt/ros/humble/setup.bash

# ── Auto-rebuild ─────────────────────────────────────────────────────────────
# When ros2_ws is bind-mounted the pre-built install/ from the image is
# replaced by the host directory.  If install/ is absent (first run, fresh
# clone, or after `rm -rf install build log`) we rebuild automatically.
cd "$ROS_WS"

if [ ! -d "$ROS_WS/install" ]; then
    echo "[entrypoint] install/ not found — running colcon build …"
    colcon build --symlink-install
fi

# Source the workspace overlay
if [ -f "$ROS_WS/install/setup.bash" ]; then
    source "$ROS_WS/install/setup.bash"
fi

# ── Execute the container command ────────────────────────────────────────────
exec "$@"