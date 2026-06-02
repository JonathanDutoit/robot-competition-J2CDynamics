# ── Stage 1: base ────────────────────────────────────────────────────────────
FROM ros:humble-ros-base AS ros2_base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    cmake \ 
    iproute2 \
    gettext \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

# ── Stage 2: dependencies ─────────────────────────────────────────────────────
FROM ros2_base AS dependencies

RUN apt update

# Install in separate layers (no apt update needed)
RUN apt install -y ros-humble-ament-cmake
RUN apt install -y ros-humble-nav2-bringup
RUN apt install -y ros-humble-slam-toolbox
RUN apt install -y ros-humble-joy ros-humble-teleop-twist-joy
RUN apt install -y ros-humble-tf2-ros ros-humble-tf2-geometry-msgs

# Clean up at the end
RUN rm -rf /var/lib/apt/lists/*


FROM dependencies AS builder

COPY docker/navigation/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]