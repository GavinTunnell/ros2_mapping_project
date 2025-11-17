#!/usr/bin/env python3
"""Bringup for mapping stack: TFs, RPLidar, IMU+filter, encoders, EKF, SLAM, RViz."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ros2_mapping_project'
    share_dir = get_package_share_directory(package_name)

    odom_params = os.path.join(share_dir, 'odom_params.yaml')
    slam_params = os.path.join(share_dir, 'slam_params.yaml')
    imu_filter_params = os.path.join(share_dir, 'imu_filter_params.yaml')

    return LaunchDescription([
        # Ensure FastDDS SHM off (same as your scripts)
        SetEnvironmentVariable(name='FASTDDS_TRANSPORT_SHARED_MEM', value='off'),

        # ----- Static TFs -----
        # Lidar a bit forward of the rotation center (adjust 0.12 to your real offset)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_base_tf',
            arguments=['0.12', '0.0', '0.10', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_base_tf',
            arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen',
        ),

        # ----- RPLidar -----
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'angle_compensate': True,
            }],
        ),

        # ----- IMU + Madgwick filter: /imu/raw -> /imu/data -----
        Node(
            package=package_name,
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{'bus': 7, 'address': 0x28}],
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_filter_params, {
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }],
            remappings=[
                ('/imu/data_raw', '/imu/raw'),
                ('/imu/data', '/imu/data'),
            ],
        ),

        # ----- Encoders: /wheel_twist -----
        Node(
            package=package_name,
            executable='encoders_node',
            name='encoders_node',
            output='screen',
            parameters=[{
                'left_A': 12,
                'left_B': 16,
                'right_A': 7,
                'right_B': 11,
                'wheel_radius': 0.05,
                'wheel_base': 0.24,
                'ticks_per_rev': 333.3333,
                'publish_rate_hz': 20.0,
            }],
        ),

        # ----- EKF (robot_localization) -----
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[odom_params],
        ),

        # ----- SLAM Toolbox (sync, mapping mode) -----
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params],
        ),

        # ----- RViz2 -----
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])
