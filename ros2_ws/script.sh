rm -rf install log build 
colcon build 
source install/setup.bash
ros2 launch j2cdynamics_bringup turtlebot_simulation.launch.py 
