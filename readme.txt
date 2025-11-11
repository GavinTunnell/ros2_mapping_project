//setup

1. Clean Enviornment and TF: (Terminal A)
unset $(env | awk -F= '/^(AMENT|COLCON|ROS_|RMW_|CYCLONEDDS)/{print $1}')
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 run tf2_ros static_transform_publisher 0 0 0.10 0 0 0 base_link laser &
ros2 run tf2_ros static_transform_publisher 0 0 0.05  0 0 0 base_link imu_link &


2. Run simulated Odometer: (Terminal B)

python3 ~/Desktop/ros2_mapping_project/pseudo_odom_node.py

3. Run IMU: (Terminal C)

source /opt/ros/humble/setup.bash
python3 ~/Desktop/ros2_mapping_project/imu_bno055_smbus.py --ros-args -p bus:=7 -p address:=0x28

4. Run LiDAR: (Terminal D)

source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true

5. Run Slam: (Terminal E)

source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$HOME/Desktop/ros2_mapping_project/slam_params.yaml


6. Run RVIZ: (Terminal F)

source /opt/ros/humble/setup.bash
rviz2