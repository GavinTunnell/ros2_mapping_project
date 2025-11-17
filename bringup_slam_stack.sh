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
export FASTDDS_TRANSPORT_SHARED_MEM=off

echo "[bringup] Launching SLAM stack via ros2_mapping_project/bringup.launch.py..."
echo "          (EKF uses odom_params.yaml, SLAM uses slam_params.yaml)"
echo ""

# This single launch file starts:
#  - Static TFs (base_link->laser, base_link->imu_link)
#  - RPLidar node
#  - IMU node + Madgwick filter (/imu/raw -> /imu/data)
#  - Encoders node (/wheel_twist)
#  - EKF (robot_localization) with odom_params.yaml
#  - SLAM Toolbox (sync_slam_toolbox_node)
#  - RViz2
ros2 launch ros2_mapping_project bringup.launch.py
