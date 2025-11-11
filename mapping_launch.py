from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    use_internal_lidar = LaunchConfiguration('use_internal_lidar')
    use_imu_velocity = LaunchConfiguration('use_imu_velocity')  # if true, override twist to /imu_odom

    return LaunchDescription([
        DeclareLaunchArgument('use_internal_lidar', default_value='false'),
        DeclareLaunchArgument('use_imu_velocity', default_value='true'),

        OpaqueFunction(function=lambda ctx: [  # LIDAR
            Node(
                package='sllidar_ros2', executable='sllidar_node',
                name='sllidar_node', output='screen',
                parameters=[{'channel_type': 'serial',
                             'serial_port': '/dev/ttyUSB0',
                             'serial_baudrate': 115200,
                             'frame_id': 'laser',
                             'inverted': False,
                             'angle_compensate': True}]
            )] if ctx.launch_configurations['use_internal_lidar'] == 'false' else [
            Node(package='ros2_mapping_project', executable='lidar_node', name='lidar_node', output='screen')
        ]),

        # IMU raw → Madgwick → /imu/data
        Node(package='ros2_mapping_project', executable='imu_node', name='imu_node', output='screen'),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
             parameters=[os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ros2_mapping_project', 'imu_filter_params.yaml'),
                         {'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
             remappings=[('/imu/data_raw', '/imu/raw'), ('/imu/data', '/imu/data')]),

        # Optional IMU-derived odom when encoders are offline
        Node(package='ros2_mapping_project', executable='imu_twist_odom_node',
             name='imu_twist_odom_node', output='screen'),

        # Encoders (enable when hardware is connected)
        Node(package='ros2_mapping_project', executable='encoders_node', name='encoders_node', output='screen'),

        # EKF: fuse twist + IMU -> /odometry/filtered and TF
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[
                 os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ros2_mapping_project', 'odom_params.yaml'),
                 # If using IMU velocity, override twist0 to /imu_odom
                 {'twist0': '/imu_odom'}
             ] if str(use_imu_velocity) == 'true' else
             [os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ros2_mapping_project', 'odom_params.yaml')]
        ),

        # Static TFs: adjust offsets to your mounts
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.10', '0', '0', '0', 'base_link', 'laser']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']),

        # SLAM toolbox online sync
        Node(package='slam_toolbox', executable='sync_slam_toolbox_node', name='slam_toolbox',
             parameters=[os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ros2_mapping_project', 'slam_params.yaml')]),

        # RViz
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d',
                        os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ros2_mapping_project', 'slam.rviz')]),
    ])
