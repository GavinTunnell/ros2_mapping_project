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
  -p left_A:=7 -p left_B:=11 -p right_A:=12 -p right_B:=16 \
  -p left_active_low:=true -p right_active_low:=true \
  -p left_only_A_edges:=false -p right_only_A_edges:=false \
  -p edge_min_us:=200  -p debounce_ms:=0 \
  -p publish_rate_hz:=20.0 \
  -p wheel_radius:=0.05 -p wheel_base:=0.24 -p ticks_per_rev:=333.3333


#MOTOR DRIVER



 # Terminal 2 – teleop (your WASD script from before)
source /opt/ros/humble/setup.bash
python3 wasd_teleop.py    # publishes to /cmd_vel





./bringup_slam_stack.sh




 source /opt/ros/humble/setup.bash
python3 motor_driver_pca_reg_dual.py --ros-args \
  -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
  -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
  -p enb_channel:=0 -p in3_channel:=1 -p in4_channel:=2 \
  -p pwm_freq_hz:=1000.0 \
  -p max_lin:=0.8 -p max_ang_cmd:=1.2 -p deadband:=0.03 \
  -p min_duty_pct:=35.0 -p brake_on_zero:=false \
  -p invert_right:=true -p invert_left:=false \
  -p map_enA_to_left:=true




#run nav2_bringup

cd ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/team4/Desktop/ros2_mapping_project/nav2_min.yaml








[INFO] [1762304820.177712912] [motor_driver_pca_dual]: PCA A=0x41 (EnA=0, In1=1, In2=2) | PCA B=0x60 (EnB=0, In3=1, In4=2) | freq=1000.0Hz min_duty=35%
