1. 

cd ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 launch ros2_mapping_project bringup.launch.py


2. 


 source /opt/ros/humble/setup.bash
python3 motor_driver_pca_reg_dual.py --ros-args \
  -p ena_addr:="'0x41'" -p enb_addr:="'0x60'" \
  -p ena_channel:=0 -p in1_channel:=1 -p in2_channel:=2 \
  -p enb_channel:=0 -p in3_channel:=1 -p in4_channel:=2 \
  -p pwm_freq_hz:=1000.0 \
  -p max_lin:=0.8 -p max_ang_cmd:=1.2 -p deadband:=0.03 \
  -p min_duty_pct:=80.0 -p brake_on_zero:=false \
  -p invert_right:=true -p invert_left:=false \
  -p map_enA_to_left:=true


3. 

cd ~/Desktop/ros2_mapping_project
source /opt/ros/humble/setup.bash
export FASTDDS_TRANSPORT_SHARED_MEM=off

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=false \
  autostart:=true \
  params_file:=/home/team4/Desktop/ros2_mapping_project/nav2_min.yaml
