export FASTDDS_TRANSPORT_SHARED_MEM=off
ros2 launch ros2_mapping_project bringup_gpio.launch.py \
  use_nav2:=true \
  use_motor_driver:=false \
  use_diff_drive_odometry:=false \
  left_A_pin:=12 left_B_pin:=16 \
  right_A_pin:=7  right_B_pin:=11



ros2 topic echo /left_wheel/velocity
ros2 topic echo /left_wheel/direction
ros2 topic echo /right_wheel/velocity
ros2 topic echo /right_wheel/direction
ros2 topic echo /left_wheel/ticks
ros2 topic echo /right_wheel/ticks
ros2 topic echo /wheel_odom



export FASTDDS_TRANSPORT_SHARED_MEM=off
ros2 launch ros2_mapping_project bringup_gpio.launch.py \
  use_nav2:=true \
  use_motor_driver:=true \
  use_diff_drive_odometry:=false \
  left_A_pin:=12 left_B_pin:=16 \
  right_A_pin:=7  right_B_pin:=11

  
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off
python3 encoders_node.py --ros-args \
  -p left_A:=12 -p left_B:=16 -p right_A:=7 -p right_B:=11 \
  -p wheel_radius:=0.05 -p wheel_base:=0.24 -p ticks_per_rev:=333.3333

   # more sensitive: accept closer edges, count both channels

python3 encoders_node.py --ros-args \
  -p left_A:=16 -p left_B:=12 -p right_A:=7 -p right_B:=11 \
  -p left_active_low:=true -p right_active_low:=true \
  -p left_only_A_edges:=false -p right_only_A_edges:=false \
  -p edge_min_us:=200  -p debounce_ms:=0 \
  -p publish_rate_hz:=20.0 \
  -p wheel_radius:=0.05 -p wheel_base:=0.24 -p ticks_per_rev:=333.3333


