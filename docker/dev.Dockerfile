FROM my_robot_base

RUN apt update && apt install -y \
    ros-humble-rviz2 \
    ros-humble-rqt \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/ $ROS_WS/

RUN . /opt/ros/humble/setup.sh && colcon build

CMD ["bash"]