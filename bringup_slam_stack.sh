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

echo "[bringup] Starting static TFs (base_link->laser, base_link->imu_link)..."
ros2 run tf2_ros static_transform_publisher 0 0 0.10 0 0 0 base_link laser &
TF_LASER_PID=$!
ros2 run tf2_ros static_transform_publisher 0 0 0.05  0 0 0 base_link imu_link &
TF_IMU_PID=$!

###############################
# 2. LiDAR (RPLidar on /dev/rplidar)
###############################
echo "[bringup] Starting RPLidar node..."
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true &
RPLIDAR_PID=$!

###############################
# 3. IMU (BNO055 -> /imu/raw)
###############################
echo "[bringup] Starting IMU node..."
python3 imu_bno055_smbus.py --ros-args \
  -p bus:=7 -p address:=0x28 &
IMU_PID=$!

###############################
# 3b. IMU Filter (/imu/raw -> /imu/data)
###############################
echo "[bringup] Starting IMU filter..."
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args \
  --params-file "$(pwd)/imu_filter_params.yaml" \
  -p use_mag:=false \
  -p publish_tf:=false \
  -r /imu/data_raw:=/imu/raw \
  -r /imu/data:=/imu/data &
IMU_FILTER_PID=$!

###############################
# 4. Encoders -> /wheel_twist
###############################
echo "[bringup] Starting encoders node..."
python3 encoders_node.py --ros-args \
  -p left_A:=7  -p left_B:=11 \
  -p right_A:=12 -p right_B:=16 \
  -p left_active_low:=true  -p right_active_low:=true \
  -p left_only_A_edges:=false -p right_only_A_edges:=false \
  -p edge_min_us:=200 -p debounce_ms:=0 \
  -p publish_rate_hz:=20.0 \
  -p wheel_radius:=0.05 -p wheel_base:=0.24 -p ticks_per_rev:=333.3333 &
ENC_PID=$!

###############################
# 5. Robot Localization EKF
###############################
echo "[bringup] Starting EKF (robot_localization)..."
ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node \
  --params-file "$(pwd)/odom_params.yaml" &
EKF_PID=$!

###############################
# 6. SLAM Toolbox (mapping mode)
###############################
echo "[bringup] Starting SLAM Toolbox..."
ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
  --params-file "$(pwd)/slam_params.yaml" &
SLAM_PID=$!

###############################
# 7. RViz2 (for visualization)
###############################
echo "[bringup] Starting RViz2..."
rviz2 &
RVIZ_PID=$!

echo ""
echo "==============================="
echo "  SLAM stack bringup complete"
echo "  PIDs:"
echo "    TF laser   = $TF_LASER_PID"
echo "    TF imu     = $TF_IMU_PID"
echo "    RPLidar    = $RPLIDAR_PID"
echo "    IMU        = $IMU_PID"
echo "    IMU Filter = $IMU_FILTER_PID"
echo "    Encoders   = $ENC_PID"
echo "    EKF        = $EKF_PID"
echo "    SLAM       = $SLAM_PID"
echo "    RViz       = $RVIZ_PID"
echo "==============================="
echo "Ctrl+C here will NOT auto-kill everything; kill nodes from their terminals or with ps/htop if needed."
echo ""

# Keep script alive so you can see logs if you run it directly
wait
