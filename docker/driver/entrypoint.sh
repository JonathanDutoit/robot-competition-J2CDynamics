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
echo "[entrypoint] Building workspace..."
colcon build --symlink-install

if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
  echo "[entrypoint] Workspace sourced"
fi


# ── Device symlinks (failsafe) ────────────────────────────────────────────────
ARDUINO_BY_ID=$(ls /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_* 2>/dev/null | head -n1)

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

link_device "$ARDUINO_BY_ID" /dev/arduino "Arduino"
# ─────────────────────────────────────────────────────────────────────────────

exec "$@"