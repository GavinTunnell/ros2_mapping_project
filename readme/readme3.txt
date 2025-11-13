readme
//set up LiDAR

source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true

//set up imu

source /opt/ros/humble/setup.bash
python3 ~/Desktop/ros2_mapping_project/imu_bno055_smbus.py --ros-args -p bus:=7 -p address:=0x28

//set up encoders

source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off
python3 encoders_node.py --ros-args \
  -p left_A:=7 -p left_B:=11 -p right_A:=12 -p right_B:=16 \
  -p left_active_low:=true -p right_active_low:=true \
  -p left_only_A_edges:=false -p right_only_A_edges:=false \
  -p edge_min_us:=200  -p debounce_ms:=0 \
  -p publish_rate_hz:=20.0 \
  -p wheel_radius:=0.05 -p wheel_base:=0.24 -p ticks_per_rev:=333.3333

//set up robot locolization

d ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node \
  --params-file ~/Desktop/ros2_mapping_project/config/ekf.yaml

//setup slam mapping

cd ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 run slam_toolbox sync_slam_toolbox_node --ros-args \
  --params-file ~/Desktop/ros2_mapping_project/slam_params.yaml

//set up link

unset $(env | awk -F= '/^(AMENT|COLCON|ROS_|RMW_|CYCLONEDDS)/{print $1}')
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 run tf2_ros static_transform_publisher 0 0 0.10 0 0 0 base_link laser &
ros2 run tf2_ros static_transform_publisher 0 0 0.05  0 0 0 base_link imu_link &

//set up ekf

cd ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 run robot_localization ekf_node --ros-args \
  -r __node:=ekf_filter_node \
  --params-file ~/Desktop/ros2_mapping_project/config/ekf.yaml


//set up rviz

source /opt/ros/humble/setup.bash
rviz2