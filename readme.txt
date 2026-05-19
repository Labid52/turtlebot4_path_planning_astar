
#Terminal 1 (launch file)

cd ~/ros2_ws
colcon build --packages-select tour_guide
source install/setup.bash
ros2 launch tour_guide tour_guide_launch.py


# Terminal 2 (rviz)

ros2 launch turtlebot4_viz view_navigation.launch.py

