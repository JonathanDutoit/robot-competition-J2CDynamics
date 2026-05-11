FROM tiryoh/ros2-desktop-vnc:humble

RUN apt-get update && apt-get install -y \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*