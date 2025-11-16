#!/usr/bin/env bash
set -e

# Go to project root
cd "$(dirname "$0")"

# Clean and set up ROS environment
unset $(env | awk -F= '/^(AMENT|COLCON|ROS_|RMW_|CYCLONEDDS)/{print $1}')
source /opt/ros/humble/setup.bash

# Build the project if the install directory is missing
if [ ! -d "install" ]; then
  echo "Building project..."
  colcon build
fi

source install/setup.bash

# Launch the GPIO bringup
ros2 launch ros2_mapping_project bringup_gpio.launch.py
