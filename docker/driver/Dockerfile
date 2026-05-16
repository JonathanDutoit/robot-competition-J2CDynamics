# ── Stage 1: base ────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS ros2_base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    cmake \ 
    libserial-dev \
    ros-humble-rmw-cyclonedds-cpp \ 
    iproute2 \
    gettext \        
    && rm -rf /var/lib/apt/lists/*

WORKDIR $ROS_WS

# ── Stage 2: robot ───────────────────────────────────────────────────────────
FROM ros2_base AS robot

RUN apt update && apt install -y \
    ros-humble-ament-cmake \
    ros-humble-joint-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src/j2cdynamics_bringup /ros2_ws/src/j2cdynamics_bringup
COPY ros2_ws/src/j2cdynamics_driver /ros2_ws/src/j2cdynamics_driver
COPY ros2_ws/src/do_description /ros2_ws/src/do_description

COPY docker/driver/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]