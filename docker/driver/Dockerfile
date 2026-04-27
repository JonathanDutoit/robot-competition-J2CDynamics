# ── Stage 1: base ────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS ros2_base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    ros-humble-rmw-cyclonedds-cpp \ 
    iproute2 \
    gettext \        
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

# ── Stage 2: robot ───────────────────────────────────────────────────────────
FROM ros2_base AS robot

RUN apt update && apt install -y \
    python3-serial \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src /ros2_ws/src
RUN cd /ros2_ws && colcon build --symlink-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]