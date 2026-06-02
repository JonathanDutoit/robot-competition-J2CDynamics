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
    ros-humble-rplidar-ros \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-laser-filters \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/j2cdynamics_bringup /ros2_ws/src/j2cdynamics_bringup

COPY docker/lidar/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]