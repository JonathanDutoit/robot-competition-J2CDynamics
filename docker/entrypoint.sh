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

# ── Device symlinks (failsafe) ────────────────────────────────────────────────
LIDAR_BY_ID="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
ARDUINO_BY_ID="/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95736323632351D040D1-if00"

link_device() {
  local src="$1"
  local dst="$2"
  local name="$3"
  if [ -e "$src" ]; then
    ln -sf "$src" "$dst"
    echo "[entrypoint] OK: $name linked $src → $dst"
  else
    echo "[entrypoint] WARNING: $name not found at $src — skipping"
  fi
}

link_device "$LIDAR_BY_ID"   /dev/lidar   "LiDAR (CP2102)"
link_device "$ARDUINO_BY_ID" /dev/arduino "Arduino"
# ─────────────────────────────────────────────────────────────────────────────

exec "$@"