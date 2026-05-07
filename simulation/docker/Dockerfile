FROM tiryoh/ros2-desktop-vnc:humble

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*