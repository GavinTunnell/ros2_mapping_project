#!/usr/bin/env bash
set -e

###############################
# 0. Go to project root
###############################
cd "$(dirname "$0")"

###############################
# 1. Source local workspace
###############################
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi

###############################
# 2. Launch the stack
###############################
echo "[bringup] Launching SLAM mapping stack..."
ros2 launch ros2_mapping_project mapping_launch.py
