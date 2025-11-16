#!/usr/bin/env python3
"""Launch file for SLAM Toolbox and EKF."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch the EKF and SLAM Toolbox nodes in the correct order."""
    pkg_share = get_package_share_directory('ros2_mapping_project')

    ekf_config_path = os.path.join(pkg_share, 'odom_params.yaml')
    slam_params_path = os.path.join(pkg_share, 'slam_params.yaml')

    ekf_config = LaunchConfiguration('ekf_config', default=ekf_config_path)
    slam_params_file = LaunchConfiguration('slam_params_file', default=slam_params_path)

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ekf_config',
            default_value=ekf_config_path,
            description='Path to the EKF parameters file.'
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_path,
            description='Path to the SLAM Toolbox parameters file.'
        ),
        ekf_node,
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=ekf_node,
                on_start=[slam_toolbox_node]
            )
        )
    ])