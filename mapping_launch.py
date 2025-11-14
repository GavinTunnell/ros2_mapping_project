import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_from_launch_config(context, launch_config: LaunchConfiguration) -> bool:
    return launch_config.perform(context).lower() in ('1', 'true', 'yes', 'on')


def _build_launch_nodes(context, *args, **kwargs):
    use_internal_lidar = _bool_from_launch_config(context, LaunchConfiguration('use_internal_lidar'))
    use_ekf = _bool_from_launch_config(context, LaunchConfiguration('use_ekf'))

    share_dir = get_package_share_directory('ros2_mapping_project')

    nodes = []

    if use_internal_lidar:
        nodes.append(Node(
            package='ros2_mapping_project', executable='lidar_node',
            name='lidar_node', output='screen'))
    else:
        nodes.append(Node(
            package='sllidar_ros2', executable='sllidar_node',
            name='sllidar_node', output='screen',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser',
                         'inverted': False,
                         'angle_compensate': True}]
        ))

    # IMU raw → Madgwick → /imu/data
    nodes.extend([
        Node(package='ros2_mapping_project', executable='imu_node',
             name='imu_node', output='screen'),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             name='imu_filter',
             parameters=[os.path.join(share_dir, 'imu_filter_params.yaml'),
                         {'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
             remappings=[('/imu/data_raw', '/imu/raw'), ('/imu/data', '/imu/data')])
    ])

    # Encoders (enable when hardware is connected)
    nodes.append(Node(package='ros2_mapping_project', executable='encoders_node',
                      name='encoders_node', output='screen'))

    if use_ekf:
        # EKF: fuse twist + IMU -> /odometry/filtered and TF
        nodes.append(Node(package='robot_localization', executable='ekf_node',
                          name='ekf_filter_node', output='screen',
                          parameters=[os.path.join(share_dir, 'odom_params.yaml')]))

    # Static TFs: adjust offsets to your mounts
    nodes.extend([
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link'])
    ])

    # SLAM toolbox online sync
    slam_params_file = 'slam_params.yaml' if use_ekf else 'slam_params_no_ekf.yaml'
    nodes.append(Node(package='slam_toolbox', executable='sync_slam_toolbox_node',
                      name='slam_toolbox',
                      parameters=[os.path.join(share_dir, slam_params_file)]))

    # RViz
    nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                      arguments=['-d', os.path.join(share_dir, 'slam.rviz')]))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_internal_lidar', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        OpaqueFunction(function=_build_launch_nodes)
    ])
