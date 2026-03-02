FROM my_robot_base

COPY ros2_ws/src/ $ROS_WS/src/
COPY ros2_ws/config/ $ROS_WS/config/
COPY ros2_ws/launch/ $ROS_WS/launch/

RUN . /opt/ros/humble/setup.sh && colcon build

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]