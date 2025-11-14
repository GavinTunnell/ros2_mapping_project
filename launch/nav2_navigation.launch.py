#!/usr/bin/env python3
"""Minimal Nav2 launch hooking to ros2_mapping_project/nav2_params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_mapping_project')
    default_nav2_params = os.path.join(pkg_share, 'nav2_params.yaml')

    nav2_params = LaunchConfiguration('nav2_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_params',
            default_value=default_nav2_params,
            description='Full path to the Nav2 params YAML file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                )
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'params_file': nav2_params,
            }.items(),
        ),
    ])
