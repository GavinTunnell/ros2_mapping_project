#!/usr/bin/env python3
"""Start Nav2 core servers without map_server/AMCL (pair with live SLAM)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'ros2_mapping_project'
    default_params = os.path.join(get_package_share_directory(package_name), 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Full path to the Nav2 parameters YAML file.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation time if true.'),
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_planner', executable='planner_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_smoother', executable='smoother_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_behaviors', executable='behavior_server', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', output='screen',
             parameters=[params_file, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                         'autostart': True,
                         'node_names': lifecycle_nodes}]),
    ])
