#!/usr/bin/env bash
set -e

###############################
# 0. Go to project root
###############################
cd "$(dirname "$0")"

###############################
# 1. Clean + setup ROS env
###############################
unset $(env | awk -F= '/^(AMENT|COLCON|ROS_|RMW_|CYCLONEDDS)/{print $1}')
source /opt/ros/humble/setup.bash

###############################
# 2. Launch the SLAM Stack
###############################
echo "[bringup] Launching the full SLAM and sensor stack..."
ros2 launch launch/slam_bringup.launch.py

echo ""
echo "==============================="
echo "  SLAM stack bringup complete"
echo "==============================="
echo "Ctrl+C here will kill the launch process and all nodes."
echo ""
