#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_mapping_project')

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true',
                                         description='Whether to start RViz2')

    # Nodes
    static_tf_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']
    )

    static_tf_imu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'angle_compensate': True
        }]
    )

    imu_node = Node(
        package='ros2_mapping_project',
        executable='imu_bno055_smbus.py',
        name='imu_node',
        parameters=[{
            'bus': 7,
            'address': 0x28
        }]
    )

    encoders_node = Node(
        package='ros2_mapping_project',
        executable='encoders_node.py',
        name='encoders_node',
        parameters=[{
            'left_A': 7, 'left_B': 11,
            'right_A': 12, 'right_B': 16,
            'left_active_low': True, 'right_active_low': True,
            'edge_min_us': 200, 'debounce_ms': 0,
            'publish_rate_hz': 20.0,
            'wheel_radius': 0.05, 'wheel_base': 0.24, 'ticks_per_rev': 333.3333
        }]
    )

    motor_driver_node = Node(
        package='ros2_mapping_project',
        executable='motor_driver_pca_reg_dual.py',
        name='motor_driver_node',
        parameters=[{
            'ena_addr': '0x41', 'enb_addr': '0x60',
            'ena_channel': 0, 'in1_channel': 1, 'in2_channel': 2,
            'enb_channel': 0, 'in3_channel': 1, 'in4_channel': 2,
            'pwm_freq_hz': 1000.0,
            'max_lin': 0.8, 'max_ang_cmd': 1.2, 'deadband': 0.03,
            'min_duty_pct': 80.0, 'brake_on_zero': False,
            'invert_right': True, 'invert_left': False,
            'map_enA_to_left': True
        }]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(pkg_share, 'odom_params.yaml')]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(pkg_share, 'slam_params.yaml')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        static_tf_laser_node,
        static_tf_imu_node,
        rplidar_node,
        imu_node,
        encoders_node,
        motor_driver_node,
        ekf_node,
        # Add a small delay before starting slam_toolbox
        LogInfo(msg="Waiting 2s for TFs to stabilize before starting SLAM..."),
        Node(
            package='rclcpp_components',
            executable='component_container',
            name='slam_toolbox_container',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
            on_exit=[
                LogInfo(msg="SLAM Toolbox container exited.")
            ],
            ros_arguments=['--ros-args', '--log-level', 'info'],
            # The following is a trick to add a delay
            prefix="bash -c 'sleep 2; $0 $@'",
            composable_node_descriptions=[
                slam_toolbox_node
            ]
        ),
        rviz_node
    ])
