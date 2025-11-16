#!/usr/bin/env bash
set -e

# Go to project root
cd "$(dirname "$0")"

# Clean and set up ROS environment
unset $(env | awk -F= '/^(AMENT|COLCON|ROS_|RMW_|CYCLONEDDS)/{print $1}')
source /opt/ros/humble/setup.bash

# Clean, build, and source the project
echo "Cleaning and building project..."
rm -rf build install log
colcon build
source install/setup.bash

# Launch the GPIO bringup
ros2 launch ros2_mapping_project bringup_gpio.launch.py
